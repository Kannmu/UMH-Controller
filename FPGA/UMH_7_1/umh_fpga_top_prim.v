// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Mon Sep 14 18:31:31 2026
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
    wire sck_N_3045 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
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
    
    wire n14025;
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[51:71])
    wire [31:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:35])
    wire [31:0]spi_expected_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[37:56])
    wire [31:0]accepted_sequence_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[58:79])
    wire [6:0]spi_channel_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(122[17:34])
    wire [1:0]spi_channel_field;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(123[17:34])
    wire [87:0]spi_bitmap;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(124[17:27])
    
    wire frame_toggle_spi, stop_toggle_spi, invalid_frame_spi, ws2812_toggle_spi, 
        n23587, pll_clk_enable_347, n23132, pll_clk_enable_316, n23130;
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
    wire [6:0]ev_ch;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(196[17:22])
    wire [83:0]ev_bit;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(197[17:23])
    wire [83:0]init_shadow;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(197[25:36])
    wire [7:0]build_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(198[17:28])
    wire [8:0]build_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(199[17:26])
    wire [6:0]staging_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(201[17:32])
    
    wire frame_req, swap_pending, active_bank;
    wire [15:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[17:26])
    wire [7:0]ev_rd_slot;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[17:27])
    wire [8:0]event_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[17:30])
    
    wire ev_we;
    wire [8:0]ev_wr_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(223[17:27])
    wire [83:0]ev_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(226[17:27])
    wire [83:0]ev_rd_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(227[17:27])
    wire [83:0]ev_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(228[17:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen, ws2812_toggle_meta, ws2812_toggle_sync, 
        ws2812_toggle_seen, invalid_frame_meta, n23688;
    wire [31:0]accepted_sequence_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(246[17:39])
    wire [31:0]accepted_sequence_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(246[41:63])
    wire [31:0]pending_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(247[17:33])
    wire [31:0]accepted_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(247[35:52])
    wire [15:0]update_flags_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(248[17:34])
    wire [15:0]update_flags_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(248[36:53])
    wire [3:0]frame_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(249[17:29])
    
    wire n40, n39, n38, n37, n36, n35, n34, n13044, n13151, 
        n13157, n13163, n13145, n13139, n40_adj_3159, n39_adj_3160, 
        n38_adj_3161, n37_adj_3162, n36_adj_3163, n35_adj_3164, n34_adj_3165;
    wire [95:0]rgb_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(250[17:25])
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(257[18:29])
    
    wire n23586;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(265[17:33])
    wire [15:0]expected_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[17:30])
    
    wire frame_end;
    wire [8:0]ev_wr_addr_8__N_912;
    
    wire n20258, n23687, n23663, n23735, n7, n23118, n23668, n22948;
    wire [15:0]status_flags_wire_15__N_1385;
    wire [15:0]status_flags_wire_15__N_1401;
    wire [6:0]spi1_miso_N_2513;
    
    wire spi1_miso_N_2512, fpga_cs_n_N_2521, n8, n22867, n23726;
    wire [15:0]expected_next_15__N_1458;
    wire [15:0]expected_next_15__N_1417;
    wire [31:0]frame_end_N_2612;
    wire [15:0]rd_data_15__N_2647;
    
    wire frame_end_N_2611, spi1_sck_c_enable_104, spi1_sck_c_enable_40, 
        n13050, n38_adj_3166, n39_adj_3167, n40_adj_3168, n41, n42, 
        n43, n44, n45, n63, n15, n142, n141;
    wire [15:0]spi_byte_count_15__N_1694;
    
    wire ws2812_toggle_spi_N_2556, n12716, n12722, n12728, stop_toggle_spi_N_2537, 
        n12734, n12740, n12746, frame_toggle_spi_N_2526, n12752, n12758, 
        n12764, n12770, n12776, n12782, invalid_frame_spi_N_2544, 
        n12788, n12794, n12800, n12806, n12812, n13038, n13121, 
        n13115, n13127, n13133, n13109, n40_adj_3169, n39_adj_3170, 
        n38_adj_3171, n37_adj_3172, n36_adj_3173, n35_adj_3174, n34_adj_3175, 
        n13098, n13092, n14, n13086, n13032, n13026, spi1_sck_c_enable_199, 
        n12818, n12824, n12830, n12836, n12842, n12848, n13020, 
        n28, spi1_sck_c_enable_236, wrap_s2_N_2572, active_bank_N_910, 
        n15614, n23092, n13008, n9, spi1_sck_c_enable_182, spi1_sck_c_enable_238, 
        n23088, n12854, n12860, pll_clk_enable_453, pll_clk_enable_362, 
        n13014, n12866, n12872, pll_clk_enable_731, n23686, n13175, 
        n13181, n13213, n13219, n13187, n13225, n13193, n23084, 
        pll_clk_enable_217, pll_clk_enable_7, n12878, n12884, n12890, 
        n12896, n12902, n12908, n12914, n12920, n12926, n12932, 
        n12938, n12944, n12950, n12956, n12962, n12974, n12980, 
        n12986, n12994, n13002, n13080, n9817;
    wire [3:0]frame_settle_3__N_1832;
    
    wire n23661, n13074, n13068, n13207;
    wire [31:0]accepted_sequence_31__N_1122;
    
    wire swap_pending_N_2586, n22273, n22295, pll_clk_enable_705, spi1_sck_c_enable_89, 
        spi1_sck_c_enable_96, spi1_sck_c_enable_97, n17128, n6, n6_adj_3176, 
        pll_clk_enable_12, n26, n23684, n23064, n25, n24, n22, 
        pll_clk_enable_13, n22929, n23058, n23056, n140, n139;
    wire [7:0]ev_clear_addr_7__N_2237;
    
    wire ev_clear_done_N_2575;
    wire [3:0]ev_state_3__N_1940;
    wire [8:0]build_sum_8__N_2053;
    wire [6:0]ev_ch_6__N_1948;
    wire [3:0]ev_state_3__N_697;
    wire [6:0]ev_ch_6__N_709;
    
    wire n133, n23048, n23046, n23042, n22343, n12, n18609, n16711;
    wire [6:0]staging_rd_addr_6__N_901;
    
    wire n18, n16748, n22294, n22293, n23026, n22292, n23020, 
        n22876, n22771, n22291, n23018, n22290, n10188, n10187, 
        n10186, n10185, n10184, n10183, n10182, n10181, n10180, 
        n10179, n10178, n10177, n10176, n10175, n10174, n10173, 
        n10172, n10171, n10170, n10169, n10168, n10167, n10166, 
        n10165, n10164, n10163, n10162, n10161, n10160, n10159, 
        n10158, n10157, n10156, n10155, n10153, n10152, n10151, 
        n10150, n10149, n10148, n10147, n10146, n10145, n10144, 
        n10143, n10142, n10141, n14340, n30, n29, n28_adj_3177, 
        n27, n26_adj_3178, n13062, n20, n19, n18_adj_3179, n13056, 
        n14_adj_3180, n18593, n23010, n15_adj_3181, mic_tick_N_2573, 
        n23662, n40_adj_3182, n39_adj_3183, n38_adj_3184, n37_adj_3185, 
        n36_adj_3186, n35_adj_3187, n34_adj_3188, n13199, n138, n137, 
        n165, n164, n163, n162, n161, n23303, spi1_sck_c_enable_111, 
        n22133, n22289, n136, n22891, mic_clk_N_2522, n22939, n22288, 
        n34_adj_3189, spi1_sck_c_enable_57, n22914, n22287, n10140, 
        spi1_sck_N_416_enable_7, n6_adj_3190, n5, n3, n22840, spi1_sck_c_enable_230;
    wire [1:0]state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[15:20])
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[16:30])
    
    wire spi1_sck_c_enable_29, pll_clk_enable_736, pll_clk_enable_233, 
        n160, n159, n158, n157, n156, n155, n154, n153, n152, 
        n151, n150, n149, n148, n147, n146, n145, n144, n143, 
        n135, n134;
    wire [23:0]shift_register_23__N_2908;
    
    wire n15506, n22992, n6_adj_3191, n23951, n15587, n15590, n10, 
        n16216, n22286, n22285, pll_clk_enable_439, pll_clk_enable_642, 
        spi1_sck_c_enable_73, n23667, n70, n22845, n23333, n23332, 
        n23331, n23330, n23329, n23328, n23327, n22271, n23326, 
        n23325, n23324, n23323, n23322, n23321, n23320, n23319, 
        n23318, pll_clk_enable_111, n23317, n14465, n23316, n76, 
        pll_clk_enable_739, n23315, n22284, n23314, n23260, n23258, 
        n23313, n23312, n23311, n23252, n30_adj_3192, n23248, n91, 
        pll_clk_enable_282, n12968, n23240, n23310, pll_clk_enable_183, 
        n23309, n23236, n8_adj_3193, n23232, n22362, n23308, n17, 
        pll_clk_enable_744, n44_adj_3194, spi1_sck_c_enable_65, n23307, 
        pll_clk_enable_18, n23228, n23306, n23305, n23304, n23302, 
        n23301, pll_clk_enable_743, spi1_sck_c_enable_82, spi1_sck_c_enable_239, 
        n23224, n23220, n23300, n23299, n23298, n23218, n23297, 
        n12_adj_3195, n23216, n23214, n11584, n23210, n11572, n23080, 
        n11570, n11568, n11566, n11564, n11562, n11560, n11558, 
        n11556, n23208, n11554, n17_adj_3196, n22925, n11552, n11550, 
        n11548, n11546, n11544, n11542, n11540, n11538, n11536, 
        n23206, n11534, n11532, n11530, n11528, n11526, n11524, 
        n11522, n11520, n23204, n11518, n11516, n11514, n11512, 
        n11510, n11508, n11506, n11504, n11502, n6_adj_3197, n23296, 
        n11500, n11498, n11496, n11494, n11492, n11490, n11488, 
        n11486, n11484, n5_adj_3198, n23733, n11482, n22270, n22966, 
        n11480, n11478, n11476, n11474, n11472, n11470, n11468, 
        n11466, n11464, n22283, n11462, n11460, n11458, n11456, 
        n11454, n11452, n11450, n11448, n11446, n23295, n11444, 
        n11442, n11440, n11438, n11436, n11434, n11432, n11430, 
        n11428, n11426, n11424, n11422, n11420, n11418, n11416, 
        n11414, n11412, n11410, n23294, n11408, n23293, n23292, 
        n22976, n11385, n23200, n23291, n23290, n23198, n23289, 
        n23288, pll_clk_enable_484, n23287, n23286, n23196, pll_clk_enable_464, 
        n23285, n10_adj_3199, n22282, n22269, n22281, n22267, n22266, 
        n23192, spi1_sck_c_enable_38, n23284, n8_adj_3200, n22960, 
        n23283, n23282, n23188, n22878, n23281, n23184, n23182, 
        n23180, n22280, n22265, n22279, n22263, n22262, n23280, 
        n23279, n22278, n22261, n22260, n22277, n22259, n22258, 
        n23176, n22276, n23278, n22257, n22235, spi1_sck_c_enable_44, 
        n22227, n22226, n22234, n22242, n22256, n22241, n22233, 
        n22225, n22219, n22232, n22240, n22275, n22255, n22239, 
        n22231, n22220, n22217, n22221, n22223, n22218, n22230, 
        n22238, n22254, n22237, n22229, n23277, n22216, n23276, 
        n22137, n22215, n22224, n22136, n22134, n22135, n22222, 
        n22228, n22236, n23172, n23168, n23166, n23164, n23653, 
        n22274, n174, n23956, pll_clk_enable_610, n15697, n10777, 
        pll_clk_enable_24, n22897, n23679, pll_clk_enable_119, n23275, 
        n23274, n23976, n23154, n13, n23974, n23273, n23271, n23972, 
        n23270, pll_clk_enable_576, n23269, n23722, n23720, n23678, 
        n23148, n23719, n23718, n23717, n23716, n23715, n23714, 
        n23713, n23712, n14612, n23711, n23710, n23708, n22922, 
        n14169, n23675, n23707, n23665, n13494, n166, n23706, 
        n23705, spi1_sck_c_enable_197, n23674, n23673, n23703, pll_clk_enable_27, 
        pll_clk_enable_6, n20216, n23700, spi1_sck_c_enable_237, n23730, 
        n23729, pll_clk_enable_727, pll_clk_enable_29, n23487, n23698, 
        n23727, n23696, n23272, n23695, n14163, n23725, n13590, 
        n23486, pll_clk_enable_729, n23694, pll_clk_enable_45, n23724, 
        n23723, n23692, n23664, n23691, n23588;
    
    VHI i2 (.Z(VCC_net));
    INV i14817 (.A(spi_mic_sck_c), .Z(sck_N_3045));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:35])
    CCU2D expected_next_15__I_0_647_13 (.A0(spi_rx_shift[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22259), .COUT(n22260), .S0(expected_next[13]), 
          .S1(expected_next[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_13.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_647_13.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_647_13.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_13.INJECT1_1 = "NO";
    LUT4 mux_1408_i9_3_lut (.A(n10173), .B(n10174), .C(n10156), .Z(rd_data_15__N_2647[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i9_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut (.A(n14465), .B(init_shadow[57]), .C(ev_bit[57]), .Z(n13056)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut.init = 16'hecec;
    FD1P3AX spi_bitmap_i0_i15 (.D(spi_bitmap[7]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i15.GSR = "ENABLED";
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
    FD1P3IX init_shadow_i29 (.D(n12884), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i29.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    LUT4 run_addr_s3_8__I_0_i3_3_lut (.A(run_addr_s3[2]), .B(ev_rd_slot[2]), 
         .C(n22343), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i3_3_lut.init = 16'hacac;
    LUT4 i1_4_lut_rep_94 (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n23694), .Z(pll_clk_enable_739)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i1_4_lut_rep_94.init = 16'h5150;
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    LUT4 mux_1408_i10_3_lut (.A(n10175), .B(n10176), .C(n10156), .Z(rd_data_15__N_2647[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i10_3_lut.init = 16'hcaca;
    LUT4 i6809_2_lut_4_lut (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n23694), .Z(n15614)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i6809_2_lut_4_lut.init = 16'h0100;
    LUT4 mux_1408_i11_3_lut (.A(n10177), .B(n10178), .C(n10156), .Z(rd_data_15__N_2647[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i11_3_lut.init = 16'hcaca;
    LUT4 mux_1408_i12_3_lut (.A(n10179), .B(n10180), .C(n10156), .Z(rd_data_15__N_2647[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i12_3_lut.init = 16'hcaca;
    LUT4 i4351_4_lut (.A(ev_ch[3]), .B(staging_rd_addr[3]), .C(n22929), 
         .D(n22914), .Z(staging_rd_addr_6__N_901[3])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i4351_4_lut.init = 16'hac0c;
    LUT4 i1965_2_lut_rep_91_4_lut (.A(frame_settle[0]), .B(pll_clk_enable_27), 
         .C(n23703), .D(n23700), .Z(pll_clk_enable_24)) /* synthesis lut_function=(A (B (D)+!B ((D)+!C))+!A (D)) */ ;
    defparam i1965_2_lut_rep_91_4_lut.init = 16'hff02;
    LUT4 spi_payload_byte_I_0_4_lut (.A(n22876), .B(n3), .C(expected_next_15__N_1417[7]), 
         .D(n23695), .Z(spi_write)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(134[32:79])
    defparam spi_payload_byte_I_0_4_lut.init = 16'h0020;
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3IX ev_bit_i45 (.D(ev_bit[44]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i45.GSR = "DISABLED";
    FD1P3IX ev_bit_i44 (.D(ev_bit[43]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i44.GSR = "DISABLED";
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    LUT4 i1_3_lut_rep_74 (.A(spi1_sck_c_enable_197), .B(n14340), .C(n23692), 
         .Z(n23663)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;
    defparam i1_3_lut_rep_74.init = 16'h0202;
    LUT4 mux_1408_i13_3_lut (.A(n10181), .B(n10182), .C(n10156), .Z(rd_data_15__N_2647[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i13_3_lut.init = 16'hcaca;
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_update_flags_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1417[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_update_flags_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(expected_next[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i14 (.D(spi_bitmap[6]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i14.GSR = "ENABLED";
    LUT4 run_addr_s3_8__I_0_i4_3_lut (.A(run_addr_s3[3]), .B(ev_rd_slot[3]), 
         .C(n22343), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i4_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_60 (.A(n14465), .B(init_shadow[9]), .C(ev_bit[9]), 
         .Z(n12764)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_60.init = 16'hecec;
    FD1P3AX spi_bitmap_i0_i13 (.D(spi_bitmap[5]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i12 (.D(spi_bitmap[4]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i12.GSR = "ENABLED";
    FD1S3IX frame_settle__i0 (.D(n14612), .CK(pll_clk), .CD(n11584), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    FD1P3IX ev_bit_i43 (.D(ev_bit[42]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i43.GSR = "DISABLED";
    FD1P3AX spi_bitmap_i0_i11 (.D(spi_bitmap[3]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i11.GSR = "ENABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    LUT4 mux_1408_i14_3_lut (.A(n10183), .B(n10184), .C(n10156), .Z(rd_data_15__N_2647[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i14_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_4_lut (.A(spi_byte_count[1]), .B(n23705), .C(spi_byte_count[5]), 
         .D(spi_byte_count[4]), .Z(n22960)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_3_lut_4_lut.init = 16'hffdf;
    LUT4 i4347_4_lut (.A(ev_ch[1]), .B(staging_rd_addr[1]), .C(n22929), 
         .D(n22914), .Z(staging_rd_addr_6__N_901[1])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i4347_4_lut.init = 16'hac0c;
    FD1P3AX spi_byte_count_i0_i0 (.D(spi_byte_count_15__N_1694[0]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i0.GSR = "ENABLED";
    LUT4 mux_1408_i15_3_lut (.A(n10185), .B(n10186), .C(n10156), .Z(rd_data_15__N_2647[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i15_3_lut.init = 16'hcaca;
    FD1P3AX status_hold__i1 (.D(accepted_sequence[24]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i1.GSR = "DISABLED";
    FD1P3IX ev_bit_i52 (.D(ev_bit[51]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i52.GSR = "DISABLED";
    LUT4 i4349_4_lut (.A(ev_ch[2]), .B(staging_rd_addr[2]), .C(n22929), 
         .D(n22914), .Z(staging_rd_addr_6__N_901[2])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i4349_4_lut.init = 16'hac0c;
    LUT4 spi_channel_field_1__I_0_i3_2_lut (.A(spi_channel_field[0]), .B(spi_channel_field[1]), 
         .Z(n3)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(134[52:79])
    defparam spi_channel_field_1__I_0_i3_2_lut.init = 16'hdddd;
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX stop_toggle_seen_524 (.D(stop_toggle_sync), .SP(pll_clk_enable_6), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam stop_toggle_seen_524.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_78_4_lut (.A(n18609), .B(spi_byte_count[8]), .C(n23675), 
         .D(n23722), .Z(n23667)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_rep_78_4_lut.init = 16'hfffe;
    LUT4 i13510_3_lut_4_lut (.A(mic_sample_count[2]), .B(n23715), .C(mic_sample_count[3]), 
         .D(mic_sample_count[4]), .Z(n26_adj_3178)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam i13510_3_lut_4_lut.init = 16'h7f80;
    LUT4 mux_1408_i16_3_lut (.A(n10187), .B(n10188), .C(n10156), .Z(rd_data_15__N_2647[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i16_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_4_lut (.A(ev_state[3]), .B(n23706), .C(pll_clk_enable_6), 
         .D(n23684), .Z(pll_clk_enable_729)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_4_lut.init = 16'hfffe;
    FD1P3AX ws2812_toggle_seen_523 (.D(ws2812_toggle_sync), .SP(pll_clk_enable_7), 
            .CK(pll_clk), .Q(ws2812_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ws2812_toggle_seen_523.GSR = "DISABLED";
    FD1S3AX phase_step_s1_486 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_s1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s1_486.GSR = "DISABLED";
    FD1S3AX phase_step_s2_487 (.D(phase_step_s1), .CK(pll_clk), .Q(phase_step_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s2_487.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0 (.D(accepted_sequence_31__N_1122[0]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s3_490 (.D(phase_step_s2), .CK(pll_clk), .Q(phase_step_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s3_490.GSR = "DISABLED";
    FD1S3AX swap_now_s3_491 (.D(pll_clk_enable_29), .CK(pll_clk), .Q(swap_now_s3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam swap_now_s3_491.GSR = "DISABLED";
    FD1S3AX active_bank_492 (.D(active_bank_N_910), .CK(pll_clk), .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam active_bank_492.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i0 (.D(global_phase_s2[0]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i0.GSR = "DISABLED";
    FD1S3AX phase_step_s4_497 (.D(phase_step_s3), .CK(pll_clk), .Q(phase_step_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s4_497.GSR = "DISABLED";
    FD1S3AX swap_now_s4_498 (.D(swap_now_s3), .CK(pll_clk), .Q(swap_now_s4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam swap_now_s4_498.GSR = "DISABLED";
    FD1S3AX phase_step_s5_499 (.D(phase_step_s4), .CK(pll_clk), .Q(phase_step_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s5_499.GSR = "DISABLED";
    FD1S3AX swap_now_s5_500 (.D(swap_now_s4), .CK(pll_clk), .Q(swap_now_s5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam swap_now_s5_500.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i0 (.D(mic_data_0_c), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i0.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i0 (.D(accepted_sequence_spi[0]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i0.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_505 (.D(frame_toggle_spi), .CK(pll_clk), .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_toggle_meta_505.GSR = "DISABLED";
    FD1S3AX frame_toggle_sync_506 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_toggle_sync_506.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_507 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam stop_toggle_meta_507.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_508 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam stop_toggle_sync_508.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_meta_509 (.D(ws2812_toggle_spi), .CK(pll_clk), 
            .Q(ws2812_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ws2812_toggle_meta_509.GSR = "DISABLED";
    FD1S3AX ws2812_toggle_sync_510 (.D(ws2812_toggle_meta), .CK(pll_clk), 
            .Q(ws2812_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ws2812_toggle_sync_510.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i0 (.D(accepted_sequence_meta[0]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i0.GSR = "DISABLED";
    FD1S3AX update_flags_meta_i1 (.D(expected_next_15__N_1458[3]), .CK(pll_clk), 
            .Q(update_flags_meta[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam update_flags_meta_i1.GSR = "DISABLED";
    FD1S3AX update_flags_sync_i1 (.D(update_flags_meta[1]), .CK(pll_clk), 
            .Q(update_flags_sync[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam update_flags_sync_i1.GSR = "DISABLED";
    FD1P3IX us_tx__i1 (.D(n10777), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i1.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_515 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam invalid_frame_meta_515.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_516 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(status_flags_wire_15__N_1385[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam invalid_frame_sync_516.GSR = "DISABLED";
    FD1P3IX frame_req_519 (.D(n23951), .SP(pll_clk_enable_12), .CD(n13590), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_req_519.GSR = "DISABLED";
    FD1P3AX ev_ch_i0 (.D(ev_ch_6__N_709[0]), .SP(pll_clk_enable_13), .CK(pll_clk), 
            .Q(ev_ch[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_ch_i0.GSR = "DISABLED";
    FD1P3AX build_phase_i0 (.D(staging_q[8]), .SP(pll_clk_enable_233), .CK(pll_clk), 
            .Q(build_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i0.GSR = "DISABLED";
    FD1P3AX build_sum_i0 (.D(build_sum_8__N_2053[0]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i0.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i0.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i0 (.D(staging_rd_addr_6__N_901[0]), .CK(pll_clk), 
            .Q(staging_rd_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam staging_rd_addr_i0.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0 (.D(accepted_sequence_sync[0]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i0.GSR = "DISABLED";
    FD1P3AX ev_clear_done_535 (.D(ev_clear_done_N_2575), .SP(pll_clk_enable_18), 
            .CK(pll_clk), .Q(ev_state_3__N_1940[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_done_535.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i1.GSR = "DISABLED";
    FD1S3AX mic_tick_538 (.D(mic_tick_N_2573), .CK(pll_clk), .Q(mic_tick)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_tick_538.GSR = "DISABLED";
    FD1S3AX mic_clock_reg_540 (.D(mic_clk_N_2522), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_clock_reg_540.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    FD1P3IX ev_bit_i0 (.D(n23951), .SP(pll_clk_enable_347), .CD(n23688), 
            .CK(pll_clk), .Q(ev_bit[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i0.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i5_3_lut (.A(run_addr_s3[4]), .B(ev_rd_slot[4]), 
         .C(n22343), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i5_3_lut.init = 16'hacac;
    LUT4 spi_byte_count_2__bdd_3_lut_14707 (.A(spi_byte_count[2]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[5]), .Z(n23487)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;
    defparam spi_byte_count_2__bdd_3_lut_14707.init = 16'h0101;
    FD1P3AX spi_expected_length_i0 (.D(expected_next[0]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i0.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i1 (.D(expected_next[1]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i1.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i2 (.D(expected_next[2]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i3 (.D(expected_next[3]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i4 (.D(expected_next[4]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i5 (.D(expected_next[5]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i6 (.D(expected_next[6]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i7 (.D(expected_next[7]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i8 (.D(expected_next[8]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i8.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i9 (.D(expected_next[9]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i9.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i10 (.D(expected_next[10]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i10.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i11 (.D(expected_next[11]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i11.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i12 (.D(expected_next[12]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i12.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i13 (.D(expected_next[13]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i13.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i14 (.D(expected_next[14]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i14.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i15 (.D(expected_next[15]), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(spi_expected_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_expected_length_i15.GSR = "ENABLED";
    LUT4 i1_3_lut_rep_85 (.A(spi_command[1]), .B(n22922), .C(spi_command[0]), 
         .Z(n23674)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;
    defparam i1_3_lut_rep_85.init = 16'h4848;
    LUT4 i1_2_lut_rep_75_3_lut (.A(n23668), .B(n22897), .C(spi_byte_count[5]), 
         .Z(n23664)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_rep_75_3_lut.init = 16'h0404;
    LUT4 run_addr_s3_8__I_0_i6_3_lut (.A(run_addr_s3[5]), .B(ev_rd_slot[5]), 
         .C(n22343), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i6_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_61 (.A(n14465), .B(init_shadow[59]), .C(ev_bit[59]), 
         .Z(n13068)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_61.init = 16'hecec;
    VLO i1 (.Z(GND_net));
    LUT4 mux_677_Mux_17_i3_3_lut (.A(rgb_hold[1]), .B(shift_register[16]), 
         .C(state[1]), .Z(shift_register_23__N_2908[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_677_Mux_17_i3_3_lut.init = 16'hcaca;
    LUT4 i1495_2_lut_3_lut_4_lut (.A(ev_ch[2]), .B(n23711), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_6__N_1948[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1495_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i14598_2_lut_2_lut_4_lut (.A(spi_command[1]), .B(n22922), .C(spi_command[0]), 
         .D(spi1_sck_c_enable_199), .Z(spi1_sck_c_enable_238)) /* synthesis lut_function=(A (B (C (D))+!B (D))+!A !(B (C+!(D))+!B !(D))) */ ;
    defparam i14598_2_lut_2_lut_4_lut.init = 16'hb700;
    LUT4 mux_677_Mux_18_i3_3_lut (.A(rgb_hold[2]), .B(shift_register[17]), 
         .C(state[1]), .Z(shift_register_23__N_2908[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_677_Mux_18_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_86_4_lut (.A(spi_byte_count[9]), .B(spi_byte_count[12]), 
         .C(n23710), .D(spi_byte_count[13]), .Z(n23675)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_rep_86_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[22]), 
         .D(ev_bit[22]), .Z(ev_wr_data[22])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut.init = 16'hddd0;
    LUT4 i1_2_lut_4_lut (.A(spi_command[1]), .B(n22922), .C(spi_command[0]), 
         .D(stop_toggle_spi), .Z(stop_toggle_spi_N_2537)) /* synthesis lut_function=(A (B (C (D)+!C !(D))+!B (D))+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;
    defparam i1_2_lut_4_lut.init = 16'hb748;
    LUT4 build_phase_7__I_0_i2_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[1]), 
         .D(build_sum[1]), .Z(ev_rd_slot[1])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i2_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i6_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[5]), 
         .D(build_sum[5]), .Z(ev_rd_slot[5])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i6_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i7_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[6]), 
         .D(build_sum[6]), .Z(ev_rd_slot[6])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i7_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i1_3_lut_adj_62 (.A(n14465), .B(init_shadow[8]), .C(ev_bit[8]), 
         .Z(n12758)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_62.init = 16'hecec;
    LUT4 run_addr_s3_8__I_0_i7_3_lut (.A(run_addr_s3[6]), .B(ev_rd_slot[6]), 
         .C(n22343), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i7_3_lut.init = 16'hacac;
    LUT4 i1_2_lut_rep_121 (.A(spi_byte_count[11]), .B(spi_byte_count[10]), 
         .Z(n23710)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_121.init = 16'heeee;
    LUT4 i2_3_lut_rep_103_4_lut (.A(spi_byte_count[11]), .B(spi_byte_count[10]), 
         .C(spi_byte_count[12]), .D(spi_byte_count[9]), .Z(n23692)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_rep_103_4_lut.init = 16'hfffe;
    LUT4 build_phase_7__I_0_i5_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[4]), 
         .D(build_sum[4]), .Z(ev_rd_slot[4])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i5_3_lut_4_lut.init = 16'hf4b0;
    LUT4 mux_677_Mux_19_i3_3_lut (.A(rgb_hold[3]), .B(shift_register[18]), 
         .C(state[1]), .Z(shift_register_23__N_2908[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_677_Mux_19_i3_3_lut.init = 16'hcaca;
    LUT4 mux_677_Mux_20_i3_3_lut (.A(rgb_hold[4]), .B(shift_register[19]), 
         .C(state[1]), .Z(shift_register_23__N_2908[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_677_Mux_20_i3_3_lut.init = 16'hcaca;
    LUT4 i1476_2_lut_rep_122 (.A(ev_ch[1]), .B(ev_ch[0]), .Z(n23711)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1476_2_lut_rep_122.init = 16'h8888;
    LUT4 build_phase_7__I_0_i4_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[3]), 
         .D(build_sum[3]), .Z(ev_rd_slot[3])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i4_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i8_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[7]), 
         .D(build_sum[7]), .Z(ev_rd_slot[7])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i8_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i1_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[0]), 
         .D(build_sum[0]), .Z(ev_rd_slot[0])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i1_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i1483_2_lut_rep_102_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(n23691)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1483_2_lut_rep_102_3_lut.init = 16'h8080;
    LUT4 build_phase_7__I_0_i3_3_lut_4_lut (.A(n23714), .B(n23713), .C(build_phase[2]), 
         .D(build_sum[2]), .Z(ev_rd_slot[2])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam build_phase_7__I_0_i3_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i2_3_lut_4_lut_adj_63 (.A(frame_settle[0]), .B(n23703), .C(pll_clk_enable_6), 
         .D(pll_clk_enable_27), .Z(pll_clk_enable_484)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[17:37])
    defparam i2_3_lut_4_lut_adj_63.init = 16'hfffe;
    LUT4 i4353_4_lut (.A(ev_ch[4]), .B(staging_rd_addr[4]), .C(n22929), 
         .D(n22914), .Z(staging_rd_addr_6__N_901[4])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i4353_4_lut.init = 16'hac0c;
    LUT4 i1_3_lut_adj_64 (.A(n14465), .B(init_shadow[7]), .C(ev_bit[7]), 
         .Z(n12752)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_64.init = 16'hecec;
    FD1S3AX mem_1378 (.D(staging_rd_addr_6__N_901[0]), .CK(pll_clk), .Q(n10141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1378.GSR = "DISABLED";
    FD1S3AX mem_1386 (.D(staging_rd_addr_6__N_901[4]), .CK(pll_clk), .Q(n10149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1386.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_80_3_lut_4_lut (.A(ev_state[1]), .B(n23719), .C(ev_state[0]), 
         .D(n23707), .Z(pll_clk_enable_347)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_80_3_lut_4_lut.init = 16'h0004;
    LUT4 i1_2_lut_3_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_197), .C(frame_end), 
         .Z(spi1_sck_c_enable_40)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut.init = 16'h4040;
    LUT4 i6704_2_lut_4_lut (.A(expected_next_15__N_1417[7]), .B(n8_adj_3200), 
         .C(n23220), .D(n3), .Z(n15590)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i6704_2_lut_4_lut.init = 16'h0008;
    LUT4 i2_3_lut_4_lut_adj_65 (.A(spi_byte_count[8]), .B(n23722), .C(spi_byte_count[5]), 
         .D(n23712), .Z(n22362)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_65.init = 16'hfffe;
    LUT4 spi_byte_count_2__bdd_4_lut_14706 (.A(spi_byte_count[2]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[5]), .D(spi_byte_count[3]), .Z(n23486)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B (C+(D))+!B ((D)+!C)))) */ ;
    defparam spi_byte_count_2__bdd_4_lut_14706.init = 16'h0016;
    FD1P3IX ws2812_enable_522 (.D(update_flags_sync[1]), .SP(pll_clk_enable_24), 
            .CD(pll_clk_enable_6), .CK(pll_clk), .Q(ws2812_enable)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ws2812_enable_522.GSR = "DISABLED";
    FD1P3AX rgb_values_0__479 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_0__479.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i8_3_lut (.A(run_addr_s3[7]), .B(ev_rd_slot[7]), 
         .C(n22343), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i8_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_4_lut_adj_66 (.A(ev_state[1]), .B(n23684), .C(ev_state[3]), 
         .D(ev_state[2]), .Z(pll_clk_enable_13)) /* synthesis lut_function=(!(A ((D)+!C)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i1_3_lut_4_lut_adj_66.init = 16'h00f4;
    LUT4 mux_677_Mux_21_i3_4_lut (.A(rgb_hold[5]), .B(shift_register[20]), 
         .C(state[1]), .D(n13494), .Z(shift_register_23__N_2908[21])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_677_Mux_21_i3_4_lut.init = 16'h0aca;
    LUT4 i14420_3_lut_4_lut (.A(spi_version[4]), .B(spi_version[3]), .C(spi_version[5]), 
         .D(spi_version[6]), .Z(n23204)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14420_3_lut_4_lut.init = 16'hfffe;
    LUT4 i4612_2_lut (.A(n15), .B(state[0]), .Z(n13494)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam i4612_2_lut.init = 16'hbbbb;
    LUT4 i14615_3_lut_3_lut (.A(n23662), .B(n14025), .C(n23674), .Z(invalid_frame_spi_N_2544)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[30] 358[24])
    defparam i14615_3_lut_3_lut.init = 16'h0404;
    FD1P3IX init_shadow_i50 (.D(n13014), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i50.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_109 (.A(expected_next[0]), .B(expected_next[1]), .Z(n23698)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_109.init = 16'heeee;
    LUT4 i2_2_lut_3_lut (.A(expected_next[0]), .B(expected_next[1]), .C(spi_extension_length[2]), 
         .Z(n6_adj_3176)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_2_lut_3_lut.init = 16'hfefe;
    LUT4 i2_3_lut_4_lut_adj_67 (.A(status_flags_wire_15__N_1401[4]), .B(pll_locked), 
         .C(pll_clk_enable_6), .D(phase_step_s5), .Z(pll_clk_enable_217)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_adj_67.init = 16'hfff7;
    LUT4 i1_2_lut_rep_110 (.A(swap_pending), .B(wrap_s2), .Z(pll_clk_enable_29)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_110.init = 16'h8888;
    FD1P3AX spi_bitmap_i0_i10 (.D(spi_bitmap[2]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i10.GSR = "ENABLED";
    FD1P3AX rgb_values_1__478 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_1__478.GSR = "DISABLED";
    PFUMX i14541 (.BLUT(n23317), .ALUT(n23318), .C0(spi1_miso_N_2513[1]), 
          .Z(n23326));
    LUT4 mux_677_Mux_22_i3_4_lut (.A(rgb_hold[6]), .B(shift_register[21]), 
         .C(state[1]), .D(n13494), .Z(shift_register_23__N_2908[22])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_677_Mux_22_i3_4_lut.init = 16'h0aca;
    LUT4 i3_2_lut_4_lut (.A(spi1_sck_c_enable_197), .B(n14340), .C(n23692), 
         .D(n23042), .Z(n9)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i3_2_lut_4_lut.init = 16'h0002;
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX rgb_values_5__474 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_5__474.GSR = "DISABLED";
    LUT4 i1955_2_lut_3_lut (.A(swap_pending), .B(wrap_s2), .C(phase_step_s2), 
         .Z(pll_clk_enable_119)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1955_2_lut_3_lut.init = 16'hf8f8;
    FD1S3AX mem_1391 (.D(staging_rd_addr_6__N_901[6]), .CK(pll_clk), .Q(n10153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1391.GSR = "DISABLED";
    LUT4 i2_3_lut_rep_79_4_lut (.A(spi_byte_count[13]), .B(n23692), .C(n23696), 
         .D(n18609), .Z(n23668)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i2_3_lut_rep_79_4_lut.init = 16'hfffe;
    LUT4 ws2812_toggle_sync_I_0_2_lut_rep_111 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .Z(n23700)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam ws2812_toggle_sync_I_0_2_lut_rep_111.init = 16'h6666;
    LUT4 mux_677_Mux_23_i3_4_lut (.A(rgb_hold[7]), .B(shift_register[22]), 
         .C(state[1]), .D(n13494), .Z(shift_register_23__N_2908[23])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam mux_677_Mux_23_i3_4_lut.init = 16'h0aca;
    FD1P3AX rgb_values_6__473 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_6__473.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i1_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[0]), .D(pending_sequence[0]), 
         .Z(accepted_sequence_31__N_1122[0])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i1_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_2_lut_3_lut_4_lut_adj_68 (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .C(wrap_s2), .D(swap_pending), .Z(pll_clk_enable_111)) /* synthesis lut_function=(A ((C (D))+!B)+!A (B+(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam i1_2_lut_3_lut_4_lut_adj_68.init = 16'hf666;
    LUT4 accepted_sequence_31__I_0_i2_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[1]), .D(pending_sequence[1]), 
         .Z(accepted_sequence_31__N_1122[1])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i2_3_lut_4_lut.init = 16'hf960;
    FD1S3AX mem_1388 (.D(staging_rd_addr_6__N_901[5]), .CK(pll_clk), .Q(n10151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1388.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_69 (.A(n14465), .B(init_shadow[6]), .C(ev_bit[6]), 
         .Z(n12746)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_69.init = 16'hecec;
    LUT4 accepted_sequence_31__I_0_i3_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[2]), .D(pending_sequence[2]), 
         .Z(accepted_sequence_31__N_1122[2])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i3_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i4_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[3]), .D(pending_sequence[3]), 
         .Z(accepted_sequence_31__N_1122[3])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i4_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i5_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[4]), .D(pending_sequence[4]), 
         .Z(accepted_sequence_31__N_1122[4])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i5_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i6_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[5]), .D(pending_sequence[5]), 
         .Z(accepted_sequence_31__N_1122[5])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i6_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_adj_70 (.A(n14465), .B(init_shadow[5]), .C(ev_bit[5]), 
         .Z(n12740)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_70.init = 16'hecec;
    OBZ spi1_miso_pad (.I(spi1_miso_N_2512), .T(fpga_cs_n_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(280[12:21])
    FD1P3AX frame_toggle_seen_517 (.D(frame_toggle_sync), .SP(pll_clk_enable_27), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_toggle_seen_517.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i7_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[6]), .D(pending_sequence[6]), 
         .Z(accepted_sequence_31__N_1122[6])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i7_3_lut_4_lut.init = 16'hf960;
    LUT4 n23487_bdd_4_lut (.A(n23487), .B(n23486), .C(spi_byte_count[1]), 
         .D(n23668), .Z(n14340)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam n23487_bdd_4_lut.init = 16'h00ca;
    FD1P3AX spi_bitmap_i0_i9 (.D(spi_bitmap[1]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i9.GSR = "ENABLED";
    LUT4 i1474_2_lut (.A(ev_ch[1]), .B(ev_ch[0]), .Z(ev_ch_6__N_1948[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1474_2_lut.init = 16'h6666;
    FD1P3AX spi_bitmap_i0_i8 (.D(spi_bitmap[0]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i8.GSR = "ENABLED";
    LUT4 i3_3_lut_rep_84_4_lut (.A(spi_byte_count[13]), .B(n23692), .C(spi_byte_count[8]), 
         .D(n18609), .Z(n23673)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i3_3_lut_rep_84_4_lut.init = 16'hfffe;
    FD1P3AX spi_bitmap_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i7.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i8_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[7]), .D(pending_sequence[7]), 
         .Z(accepted_sequence_31__N_1122[7])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i8_3_lut_4_lut.init = 16'hf960;
    LUT4 mux_1131_i6_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[5]), 
         .D(ev_clear_addr[5]), .Z(ev_wr_addr[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i6_3_lut_4_lut.init = 16'hf2d0;
    LUT4 accepted_sequence_31__I_0_i9_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[8]), .D(pending_sequence[8]), 
         .Z(accepted_sequence_31__N_1122[8])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i9_3_lut_4_lut.init = 16'hf960;
    LUT4 i4_3_lut_rep_71 (.A(expected_next_15__N_1417[7]), .B(n8_adj_3200), 
         .C(n23220), .Z(spi1_sck_c_enable_237)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i4_3_lut_rep_71.init = 16'h0808;
    FD1P3AX invalid_frame_spi_482 (.D(invalid_frame_spi_N_2544), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam invalid_frame_spi_482.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_71 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[82]), 
         .D(ev_bit[82]), .Z(ev_wr_data[82])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_71.init = 16'hddd0;
    LUT4 mux_1131_i8_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[7]), 
         .D(ev_clear_addr[7]), .Z(ev_wr_addr[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i8_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1131_i7_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[6]), 
         .D(ev_clear_addr[6]), .Z(ev_wr_addr[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i7_3_lut_4_lut.init = 16'hf2d0;
    FD1P3AX rgb_values_7__472 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_7__472.GSR = "DISABLED";
    FD1S3AX mem_1384 (.D(staging_rd_addr_6__N_901[3]), .CK(pll_clk), .Q(n10147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1384.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_72 (.A(n14465), .B(init_shadow[4]), .C(ev_bit[4]), 
         .Z(n12734)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_72.init = 16'hecec;
    LUT4 i1481_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(ev_ch_6__N_1948[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1481_2_lut_3_lut.init = 16'h7878;
    FD1P3AX ev_run_hold_s5_i0_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i0.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i10_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[9]), .D(pending_sequence[9]), 
         .Z(accepted_sequence_31__N_1122[9])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i10_3_lut_4_lut.init = 16'hf960;
    LUT4 frame_toggle_spi_I_0_2_lut (.A(frame_toggle_spi), .B(n23662), .Z(frame_toggle_spi_N_2526)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[30] 358[24])
    defparam frame_toggle_spi_I_0_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_73 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[1]), 
         .D(ev_bit[1]), .Z(ev_wr_data[1])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_73.init = 16'hddd0;
    FD1S3AX mem (.D(spi_phase_pending[7]), .CK(spi1_sck_c), .Q(n10188));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_74 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[48]), 
         .D(ev_bit[48]), .Z(ev_wr_data[48])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_74.init = 16'hddd0;
    LUT4 i1490_2_lut_rep_90_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(n23679)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1490_2_lut_rep_90_3_lut_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_adj_75 (.A(n14465), .B(init_shadow[29]), .C(ev_bit[29]), 
         .Z(n12884)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_75.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_76 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[49]), 
         .D(ev_bit[49]), .Z(ev_wr_data[49])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_76.init = 16'hddd0;
    LUT4 accepted_sequence_31__I_0_i11_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[10]), .D(pending_sequence[10]), 
         .Z(accepted_sequence_31__N_1122[10])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i11_3_lut_4_lut.init = 16'hf960;
    FD1S3AX mem_1408 (.D(spi_phase_pending[6]), .CK(spi1_sck_c), .Q(n10186));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1408.GSR = "DISABLED";
    FD1S3AX mem_1407 (.D(spi_phase_pending[5]), .CK(spi1_sck_c), .Q(n10184));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1407.GSR = "DISABLED";
    FD1S3AX mem_1406 (.D(spi_phase_pending[4]), .CK(spi1_sck_c), .Q(n10182));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1406.GSR = "DISABLED";
    FD1P3IX running_494 (.D(n23951), .SP(pll_clk_enable_29), .CD(pll_clk_enable_6), 
            .CK(pll_clk), .Q(status_flags_wire_15__N_1401[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam running_494.GSR = "DISABLED";
    FD1S3AX mem_1405 (.D(spi_phase_pending[3]), .CK(spi1_sck_c), .Q(n10180));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1405.GSR = "DISABLED";
    FD1S3AX mem_1404 (.D(spi_phase_pending[2]), .CK(spi1_sck_c), .Q(n10178));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1404.GSR = "DISABLED";
    FD1S3AX mem_1403 (.D(spi_phase_pending[1]), .CK(spi1_sck_c), .Q(n10176));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1403.GSR = "DISABLED";
    FD1S3AX mem_1402 (.D(spi_phase_pending[0]), .CK(spi1_sck_c), .Q(n10174));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1402.GSR = "DISABLED";
    FD1S3AX mem_1401 (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n10172));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1401.GSR = "DISABLED";
    FD1S3AX mem_1400 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n10170));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1400.GSR = "DISABLED";
    FD1S3AX mem_1399 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n10168));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1399.GSR = "DISABLED";
    FD1S3AX mem_1398 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n10166));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1398.GSR = "DISABLED";
    FD1S3AX mem_1397 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n10164));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1397.GSR = "DISABLED";
    FD1S3AX mem_1396 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n10162));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1396.GSR = "DISABLED";
    FD1S3AX mem_1395 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n10160));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1395.GSR = "DISABLED";
    FD1P3AX frame_toggle_spi_480 (.D(frame_toggle_spi_N_2526), .SP(spi1_sck_c_enable_40), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam frame_toggle_spi_480.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i12_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[11]), .D(pending_sequence[11]), 
         .Z(accepted_sequence_31__N_1122[11])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i12_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i13_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[12]), .D(pending_sequence[12]), 
         .Z(accepted_sequence_31__N_1122[12])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i13_3_lut_4_lut.init = 16'hf960;
    LUT4 i2_3_lut (.A(ev_state[1]), .B(build_sum[8]), .C(ev_state[0]), 
         .Z(n14465)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i2_3_lut.init = 16'h4040;
    LUT4 accepted_sequence_31__I_0_i14_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[13]), .D(pending_sequence[13]), 
         .Z(accepted_sequence_31__N_1122[13])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i14_3_lut_4_lut.init = 16'hf960;
    LUT4 i1488_2_lut_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(ev_ch_6__N_1948[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1488_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i1_3_lut_4_lut_adj_77 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[50]), 
         .D(ev_bit[50]), .Z(ev_wr_data[50])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_77.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_78 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[51]), 
         .D(ev_bit[51]), .Z(ev_wr_data[51])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_78.init = 16'hddd0;
    LUT4 accepted_sequence_31__I_0_i15_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[14]), .D(pending_sequence[14]), 
         .Z(accepted_sequence_31__N_1122[14])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i15_3_lut_4_lut.init = 16'hf960;
    FD1S3AX mem_1394 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n10158));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1394.GSR = "DISABLED";
    LUT4 i9832_2_lut_rep_123 (.A(spi_byte_count[14]), .B(spi_byte_count[13]), 
         .Z(n23712)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i9832_2_lut_rep_123.init = 16'heeee;
    FD1S3AX mem_1393 (.D(spi_write), .CK(pll_clk), .Q(n10155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1393.GSR = "DISABLED";
    FD1S3AX mem_1380 (.D(staging_rd_addr_6__N_901[1]), .CK(pll_clk), .Q(n10143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1380.GSR = "DISABLED";
    FD1S3AX mem_1382 (.D(staging_rd_addr_6__N_901[2]), .CK(pll_clk), .Q(n10145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mem_1382.GSR = "DISABLED";
    FD1P3AX rgb_values_3__476 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_3__476.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i16_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[15]), .D(pending_sequence[15]), 
         .Z(accepted_sequence_31__N_1122[15])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i16_3_lut_4_lut.init = 16'hf960;
    FD1P3AX spi_bitmap_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i6.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i17_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[16]), .D(pending_sequence[16]), 
         .Z(accepted_sequence_31__N_1122[16])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i17_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i18_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[17]), .D(pending_sequence[17]), 
         .Z(accepted_sequence_31__N_1122[17])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i18_3_lut_4_lut.init = 16'hf960;
    FD1P3AX rgb_values_2__477 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_2__477.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i19_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[18]), .D(pending_sequence[18]), 
         .Z(accepted_sequence_31__N_1122[18])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i19_3_lut_4_lut.init = 16'hf960;
    FD1P3AX rgb_values_4__475 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam rgb_values_4__475.GSR = "DISABLED";
    LUT4 accepted_sequence_31__I_0_i20_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[19]), .D(pending_sequence[19]), 
         .Z(accepted_sequence_31__N_1122[19])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i20_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i21_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[20]), .D(pending_sequence[20]), 
         .Z(accepted_sequence_31__N_1122[20])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i21_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i22_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[21]), .D(pending_sequence[21]), 
         .Z(accepted_sequence_31__N_1122[21])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i22_3_lut_4_lut.init = 16'hf960;
    LUT4 i14617_2_lut_2_lut_4_lut (.A(n22960), .B(spi_byte_count[0]), .C(n23667), 
         .D(spi1_sck_c_enable_197), .Z(spi1_sck_c_enable_29)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i14617_2_lut_2_lut_4_lut.init = 16'h0400;
    LUT4 accepted_sequence_31__I_0_i23_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[22]), .D(pending_sequence[22]), 
         .Z(accepted_sequence_31__N_1122[22])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i23_3_lut_4_lut.init = 16'hf960;
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_79 (.A(n14465), .B(init_shadow[3]), .C(ev_bit[3]), 
         .Z(n12728)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_79.init = 16'hecec;
    FD1P3AX spi_bitmap_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i5.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i24_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[23]), .D(pending_sequence[23]), 
         .Z(accepted_sequence_31__N_1122[23])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i24_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i25_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[24]), .D(pending_sequence[24]), 
         .Z(accepted_sequence_31__N_1122[24])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i25_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_80 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[52]), 
         .D(ev_bit[52]), .Z(ev_wr_data[52])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_80.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_81 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[53]), 
         .D(ev_bit[53]), .Z(ev_wr_data[53])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_81.init = 16'hddd0;
    FD1P3AX spi_bitmap_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i4.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i26_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[25]), .D(pending_sequence[25]), 
         .Z(accepted_sequence_31__N_1122[25])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i26_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i27_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[26]), .D(pending_sequence[26]), 
         .Z(accepted_sequence_31__N_1122[26])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i27_3_lut_4_lut.init = 16'hf960;
    FD1P3AX spi_bitmap_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i7.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i28_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[27]), .D(pending_sequence[27]), 
         .Z(accepted_sequence_31__N_1122[27])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i28_3_lut_4_lut.init = 16'hf960;
    FD1P3AX spi_phase_pending_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i3.GSR = "ENABLED";
    LUT4 accepted_sequence_31__I_0_i29_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[28]), .D(pending_sequence[28]), 
         .Z(accepted_sequence_31__N_1122[28])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i29_3_lut_4_lut.init = 16'hf960;
    LUT4 i2_2_lut_3_lut_4_lut (.A(ws2812_toggle_sync), .B(ws2812_toggle_seen), 
         .C(pll_clk_enable_6), .D(pll_clk_enable_12), .Z(pll_clk_enable_731)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam i2_2_lut_3_lut_4_lut.init = 16'hfff6;
    LUT4 i1_3_lut_4_lut_adj_82 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[54]), 
         .D(ev_bit[54]), .Z(ev_wr_data[54])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_82.init = 16'hddd0;
    LUT4 i14632_3_lut_4_lut (.A(n23668), .B(n22897), .C(spi_byte_count[0]), 
         .D(n22960), .Z(spi1_sck_c_enable_96)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i14632_3_lut_4_lut.init = 16'h0004;
    LUT4 accepted_sequence_31__I_0_i30_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[29]), .D(pending_sequence[29]), 
         .Z(accepted_sequence_31__N_1122[29])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i30_3_lut_4_lut.init = 16'hf960;
    LUT4 accepted_sequence_31__I_0_i31_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[30]), .D(pending_sequence[30]), 
         .Z(accepted_sequence_31__N_1122[30])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i31_3_lut_4_lut.init = 16'hf960;
    LUT4 i1_3_lut_4_lut_adj_83 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[55]), 
         .D(ev_bit[55]), .Z(ev_wr_data[55])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_83.init = 16'hddd0;
    LUT4 n20258_bdd_3_lut_14714 (.A(n20258), .B(ev_state[0]), .C(ev_state[2]), 
         .Z(n23586)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam n20258_bdd_3_lut_14714.init = 16'h0808;
    LUT4 accepted_sequence_31__I_0_i32_3_lut_4_lut (.A(ws2812_toggle_sync), 
         .B(ws2812_toggle_seen), .C(accepted_sequence_sync[31]), .D(pending_sequence[31]), 
         .Z(accepted_sequence_31__N_1122[31])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[13:53])
    defparam accepted_sequence_31__I_0_i32_3_lut_4_lut.init = 16'hf960;
    LUT4 stop_toggle_sync_I_0_2_lut_rep_112 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(pll_clk_enable_6)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam stop_toggle_sync_I_0_2_lut_rep_112.init = 16'h6666;
    LUT4 i1312_2_lut_3_lut_4_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(ws2812_toggle_seen), .D(ws2812_toggle_sync), .Z(pll_clk_enable_7)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1312_2_lut_3_lut_4_lut.init = 16'h6ff6;
    LUT4 i1_3_lut_adj_84 (.A(n23588), .B(ev_state[0]), .C(n20216), .Z(ev_state_3__N_697[1])) /* synthesis lut_function=(A+!(B+!(C))) */ ;
    defparam i1_3_lut_adj_84.init = 16'hbaba;
    LUT4 i1_3_lut_4_lut_adj_85 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[56]), 
         .D(ev_bit[56]), .Z(ev_wr_data[56])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_85.init = 16'hddd0;
    LUT4 i1_4_lut (.A(n5_adj_3198), .B(spi1_sck_c_enable_40), .C(n23662), 
         .D(n6_adj_3197), .Z(spi1_sck_c_enable_230)) /* synthesis lut_function=(A (B (C+(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_4_lut.init = 16'hc8c0;
    LUT4 i9861_4_lut (.A(frame_settle[3]), .B(pll_clk_enable_27), .C(frame_settle[2]), 
         .D(n23716), .Z(frame_settle_3__N_1832[3])) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(440[18] 454[12])
    defparam i9861_4_lut.init = 16'heeed;
    LUT4 i1_2_lut_3_lut_4_lut_adj_86 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1401[4]), .Z(n9817)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1_2_lut_3_lut_4_lut_adj_86.init = 16'h6fff;
    LUT4 i1_2_lut (.A(spi_command[0]), .B(spi_command[1]), .Z(n5_adj_3198)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 i13516_2_lut (.A(staging_q[8]), .B(staging_q[0]), .Z(build_sum_8__N_2053[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13516_2_lut.init = 16'h6666;
    LUT4 i2_2_lut (.A(spi_version[0]), .B(n22845), .Z(n6_adj_3197)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_adj_87 (.A(spi_byte_count[0]), .B(n22939), .Z(spi1_sck_c_enable_65)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_adj_87.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_88 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(wrap_s2), .D(swap_pending), .Z(swap_pending_N_2586)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1_3_lut_4_lut_adj_88.init = 16'h0900;
    LUT4 i2_4_lut (.A(n23661), .B(spi_byte_count[3]), .C(spi_byte_count[1]), 
         .D(spi_byte_count[4]), .Z(n22939)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i2_4_lut.init = 16'h0008;
    FD1P3AX spi_phase_pending_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_phase_pending_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i20.GSR = "ENABLED";
    LUT4 i82_1_lut (.A(spi_channel_field[0]), .Z(n28)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i82_1_lut.init = 16'h5555;
    LUT4 i1_3_lut_4_lut_adj_89 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[57]), 
         .D(ev_bit[57]), .Z(ev_wr_data[57])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_89.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_90 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[58]), 
         .D(ev_bit[58]), .Z(ev_wr_data[58])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_90.init = 16'hddd0;
    FD1P3AX spi_frame_sequence_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i18.GSR = "ENABLED";
    CCU2D add_164_5 (.A0(spi_byte_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22220), .COUT(n22221), .S0(spi_byte_count_15__N_1694[3]), 
          .S1(spi_byte_count_15__N_1694[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_5.INIT0 = 16'h5aaa;
    defparam add_164_5.INIT1 = 16'h5aaa;
    defparam add_164_5.INJECT1_0 = "NO";
    defparam add_164_5.INJECT1_1 = "NO";
    FD1P3AX spi_frame_sequence_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1254__i0 (.D(n40_adj_3169), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254__i0.GSR = "ENABLED";
    FD1S3AX time_divider_1259__i0 (.D(n40_adj_3159), .CK(pll_clk), .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259__i0.GSR = "DISABLED";
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    LUT4 i14611_4_lut (.A(fpga_cs_n_c), .B(spi1_sck_c_enable_197), .C(frame_end), 
         .D(n23662), .Z(spi1_sck_c_enable_199)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i14611_4_lut.init = 16'h0040;
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_1253__i0 (.D(n40), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253__i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    FD1P3AX fpga_time_1258__i0 (.D(n165), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i0.GSR = "DISABLED";
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    LUT4 spi_version_7__bdd_4_lut (.A(spi_version[6]), .B(spi_command[1]), 
         .C(spi_version[4]), .D(spi_version[3]), .Z(n23733)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;
    defparam spi_version_7__bdd_4_lut.init = 16'hfffb;
    LUT4 i1_2_lut_adj_91 (.A(spi_byte_count[0]), .B(n22939), .Z(spi1_sck_c_enable_73)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_adj_91.init = 16'h4444;
    LUT4 i1_3_lut_4_lut_adj_92 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[59]), 
         .D(ev_bit[59]), .Z(ev_wr_data[59])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_92.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_93 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[60]), 
         .D(ev_bit[60]), .Z(ev_wr_data[60])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_93.init = 16'hddd0;
    LUT4 frame_toggle_sync_I_0_2_lut_rep_113 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(pll_clk_enable_27)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[13:51])
    defparam frame_toggle_sync_I_0_2_lut_rep_113.init = 16'h6666;
    FD1P3AX global_phase_s2_1257__i0 (.D(n45), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i0.GSR = "DISABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    LUT4 i2705_2_lut_3_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(stop_toggle_seen), .D(stop_toggle_sync), .Z(n11584)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[13:51])
    defparam i2705_2_lut_3_lut_4_lut.init = 16'h6ff6;
    LUT4 i2591_4_lut (.A(ev_ch[0]), .B(staging_rd_addr[0]), .C(n22929), 
         .D(n22914), .Z(staging_rd_addr_6__N_901[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i2591_4_lut.init = 16'hac0c;
    LUT4 i1_3_lut_4_lut_adj_94 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[61]), 
         .D(ev_bit[61]), .Z(ev_wr_data[61])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_94.init = 16'hddd0;
    LUT4 i1_2_lut_adj_95 (.A(spi_byte_count[3]), .B(spi_byte_count[2]), 
         .Z(n70)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_adj_95.init = 16'h4444;
    LUT4 i6701_1_lut (.A(phase_step_s1), .Z(n15587)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i6701_1_lut.init = 16'h5555;
    LUT4 i1_2_lut_adj_96 (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .Z(n76)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:31])
    defparam i1_2_lut_adj_96.init = 16'h8888;
    LUT4 i7_4_lut (.A(global_phase_s2[0]), .B(n14_adj_3180), .C(n10), 
         .D(global_phase_s2[6]), .Z(wrap_s2_N_2572)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_97 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[62]), 
         .D(ev_bit[62]), .Z(ev_wr_data[62])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_97.init = 16'hddd0;
    LUT4 i14627_4_lut (.A(status_bit_index[4]), .B(status_bit_index[5]), 
         .C(status_bit_index[3]), .D(n6), .Z(spi1_sck_N_416_enable_7)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i14627_4_lut.init = 16'h7fff;
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    FD1S3IX mic_divider_1261__i0 (.D(n40_adj_3182), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261__i0.GSR = "DISABLED";
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1S3AX spi_bit_count_1256__i0 (.D(n20), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[34:54])
    defparam spi_bit_count_1256__i0.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .Q(expected_next[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    FD1P3AX mic_sample_count_1260__i0 (.D(n30), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam mic_sample_count_1260__i0.GSR = "DISABLED";
    CCU2D add_164_3 (.A0(spi_byte_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22219), .COUT(n22220), .S0(spi_byte_count_15__N_1694[1]), 
          .S1(spi_byte_count_15__N_1694[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_3.INIT0 = 16'h5aaa;
    defparam add_164_3.INIT1 = 16'h5aaa;
    defparam add_164_3.INJECT1_0 = "NO";
    defparam add_164_3.INJECT1_1 = "NO";
    LUT4 i6_4_lut (.A(global_phase_s2[3]), .B(global_phase_s2[1]), .C(global_phase_s2[5]), 
         .D(global_phase_s2[7]), .Z(n14_adj_3180)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_98 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[63]), 
         .D(ev_bit[63]), .Z(ev_wr_data[63])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_98.init = 16'hddd0;
    LUT4 i2_2_lut_adj_99 (.A(global_phase_s2[2]), .B(global_phase_s2[4]), 
         .Z(n10)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_adj_99.init = 16'h8888;
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
    FD1P3AX spi_update_flags_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1458[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_update_flags_i0_i1.GSR = "ENABLED";
    LUT4 i6_4_lut_adj_100 (.A(time_divider[2]), .B(n12_adj_3195), .C(time_divider[6]), 
         .D(time_divider[1]), .Z(pll_clk_enable_642)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_100.init = 16'h8000;
    LUT4 i3_3_lut_4_lut (.A(spi_byte_count[14]), .B(spi_byte_count[13]), 
         .C(n15_adj_3181), .D(n23663), .Z(n8_adj_3200)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i3_3_lut_4_lut.init = 16'h1000;
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    LUT4 i1_3_lut_rep_96_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n23703), .D(frame_settle[0]), .Z(pll_clk_enable_12)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(437[13:51])
    defparam i1_3_lut_rep_96_4_lut.init = 16'h0900;
    LUT4 i1_2_lut_adj_101 (.A(ev_state[0]), .B(ev_state[1]), .Z(n22914)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_101.init = 16'h4444;
    LUT4 i5_4_lut (.A(time_divider[0]), .B(time_divider[5]), .C(time_divider[4]), 
         .D(time_divider[3]), .Z(n12_adj_3195)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut.init = 16'h8000;
    LUT4 i2_3_lut_rep_114 (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .Z(n23703)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[17:37])
    defparam i2_3_lut_rep_114.init = 16'hfefe;
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    LUT4 i13465_1_lut (.A(spi_bit_count[0]), .Z(n20)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[34:54])
    defparam i13465_1_lut.init = 16'h5555;
    LUT4 i5730_2_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n14612)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[17:37])
    defparam i5730_2_lut_4_lut.init = 16'h00fe;
    LUT4 i13487_1_lut (.A(mic_sample_count[0]), .Z(n30)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam i13487_1_lut.init = 16'h5555;
    LUT4 i1_2_lut_rep_104_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(pll_clk_enable_45)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(442[17:37])
    defparam i1_2_lut_rep_104_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_rep_124 (.A(ev_state[0]), .B(ev_state[1]), .Z(n23713)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_rep_124.init = 16'h8888;
    LUT4 i2_3_lut_rep_115 (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .C(spi_bit_count[2]), 
         .Z(spi1_sck_c_enable_197)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i2_3_lut_rep_115.init = 16'h8080;
    LUT4 i14600_4_lut (.A(ev_clear_addr[0]), .B(ev_clear_addr[3]), .C(n23218), 
         .D(n23132), .Z(ev_clear_done_N_2575)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(496[34:58])
    defparam i14600_4_lut.init = 16'h4000;
    LUT4 i14434_4_lut (.A(ev_clear_addr[4]), .B(ev_clear_addr[6]), .C(ev_clear_addr[5]), 
         .D(ev_clear_addr[2]), .Z(n23218)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14434_4_lut.init = 16'h8000;
    LUT4 i14348_2_lut (.A(ev_clear_addr[7]), .B(ev_clear_addr[1]), .Z(n23132)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14348_2_lut.init = 16'h8888;
    FD1P3IX ev_bit_i42 (.D(ev_bit[41]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i42.GSR = "DISABLED";
    FD1P3IX ev_bit_i41 (.D(ev_bit[40]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i41.GSR = "DISABLED";
    FD1P3IX ev_bit_i40 (.D(ev_bit[39]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i40.GSR = "DISABLED";
    FD1P3IX ev_bit_i39 (.D(ev_bit[38]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i39.GSR = "DISABLED";
    FD1P3IX ev_bit_i38 (.D(ev_bit[37]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i38.GSR = "DISABLED";
    FD1P3IX ev_bit_i37 (.D(ev_bit[36]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i37.GSR = "DISABLED";
    FD1P3IX ev_bit_i36 (.D(ev_bit[35]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i36.GSR = "DISABLED";
    FD1P3IX ev_bit_i35 (.D(ev_bit[34]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i35.GSR = "DISABLED";
    FD1P3IX ev_bit_i34 (.D(ev_bit[33]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i34.GSR = "DISABLED";
    FD1P3IX ev_bit_i33 (.D(ev_bit[32]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i33.GSR = "DISABLED";
    FD1P3IX ev_bit_i32 (.D(ev_bit[31]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i32.GSR = "DISABLED";
    FD1P3IX ev_bit_i31 (.D(ev_bit[30]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i31.GSR = "DISABLED";
    LUT4 i9816_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_736)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9816_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_adj_102 (.A(frame_settle[1]), .B(frame_settle[0]), .Z(n14169)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_102.init = 16'h9999;
    LUT4 i14595_4_lut (.A(mic_divider[5]), .B(mic_divider[4]), .C(mic_divider[6]), 
         .D(n23210), .Z(mic_tick_N_2573)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(570[21:43])
    defparam i14595_4_lut.init = 16'h0100;
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i17 (.D(spi_bitmap[9]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i17.GSR = "ENABLED";
    LUT4 i14426_4_lut (.A(mic_divider[0]), .B(mic_divider[3]), .C(mic_divider[1]), 
         .D(mic_divider[2]), .Z(n23210)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14426_4_lut.init = 16'h8000;
    LUT4 i3_4_lut (.A(expected_next_15__N_1458[3]), .B(n22925), .C(n22878), 
         .D(n23042), .Z(spi1_sck_c_enable_44)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i3_4_lut.init = 16'h0080;
    LUT4 mic_clk_I_0_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(mic_clk_N_2522)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(590[18] 592[12])
    defparam mic_clk_I_0_2_lut.init = 16'h6666;
    LUT4 i7_4_lut_adj_103 (.A(n13), .B(spi_byte_count[14]), .C(n23148), 
         .D(n23220), .Z(n22878)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7_4_lut_adj_103.init = 16'h0002;
    LUT4 i1_3_lut_adj_104 (.A(n14465), .B(init_shadow[2]), .C(ev_bit[2]), 
         .Z(n12722)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_104.init = 16'hecec;
    PFUMX i14540 (.BLUT(n23315), .ALUT(n23316), .C0(spi1_miso_N_2513[1]), 
          .Z(n23325));
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:29])
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
    FD1P3AX spi_bitmap_i0_i18 (.D(spi_bitmap[10]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i19 (.D(spi_bitmap[11]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i20 (.D(spi_bitmap[12]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i21 (.D(spi_bitmap[13]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i22 (.D(spi_bitmap[14]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i23 (.D(spi_bitmap[15]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i24 (.D(spi_bitmap[16]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i25 (.D(spi_bitmap[17]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i26 (.D(spi_bitmap[18]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i27 (.D(spi_bitmap[19]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i28 (.D(spi_bitmap[20]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i29 (.D(spi_bitmap[21]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i30 (.D(spi_bitmap[22]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i31 (.D(spi_bitmap[23]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i32 (.D(spi_bitmap[24]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i32.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i33 (.D(spi_bitmap[25]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i33.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i34 (.D(spi_bitmap[26]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i34.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i35 (.D(spi_bitmap[27]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i35.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i36 (.D(spi_bitmap[28]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i36.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i37 (.D(spi_bitmap[29]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i37.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i38 (.D(spi_bitmap[30]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i38.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i39 (.D(spi_bitmap[31]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i39.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i40 (.D(spi_bitmap[32]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i40.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i41 (.D(spi_bitmap[33]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i41.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i42 (.D(spi_bitmap[34]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i42.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i43 (.D(spi_bitmap[35]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i43.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i44 (.D(spi_bitmap[36]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i44.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i45 (.D(spi_bitmap[37]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i45.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i46 (.D(spi_bitmap[38]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i46.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i47 (.D(spi_bitmap[39]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i47.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i48 (.D(spi_bitmap[40]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i48.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i49 (.D(spi_bitmap[41]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i49.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i50 (.D(spi_bitmap[42]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i50.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i51 (.D(spi_bitmap[43]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i51.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i52 (.D(spi_bitmap[44]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i52.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i53 (.D(spi_bitmap[45]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i53.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i54 (.D(spi_bitmap[46]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i54.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i55 (.D(spi_bitmap[47]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i55.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i56 (.D(spi_bitmap[48]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i56.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i57 (.D(spi_bitmap[49]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i57.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i58 (.D(spi_bitmap[50]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i58.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i59 (.D(spi_bitmap[51]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i59.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i60 (.D(spi_bitmap[52]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i60.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i61 (.D(spi_bitmap[53]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i61.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i62 (.D(spi_bitmap[54]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i62.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i63 (.D(spi_bitmap[55]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i63.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i64 (.D(spi_bitmap[56]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i64.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i65 (.D(spi_bitmap[57]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i65.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i66 (.D(spi_bitmap[58]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i66.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i67 (.D(spi_bitmap[59]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i67.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i68 (.D(spi_bitmap[60]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i68.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i69 (.D(spi_bitmap[61]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i69.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i70 (.D(spi_bitmap[62]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i70.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i71 (.D(spi_bitmap[63]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i71.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i72 (.D(spi_bitmap[64]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i72.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i73 (.D(spi_bitmap[65]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i73.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i74 (.D(spi_bitmap[66]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i74.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i75 (.D(spi_bitmap[67]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i75.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i76 (.D(spi_bitmap[68]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i76.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i77 (.D(spi_bitmap[69]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i77.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i78 (.D(spi_bitmap[70]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i78.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i79 (.D(spi_bitmap[71]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i79.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i80 (.D(spi_bitmap[72]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i80.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i81 (.D(spi_bitmap[73]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i81.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i82 (.D(spi_bitmap[74]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i82.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i83 (.D(spi_bitmap[75]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i83.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i84 (.D(spi_bitmap[76]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i84.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i85 (.D(spi_bitmap[77]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i85.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i86 (.D(spi_bitmap[78]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i86.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i87 (.D(spi_bitmap[79]), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .Q(spi_bitmap[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i87.GSR = "ENABLED";
    FD1P3IX frame_settle__i1 (.D(n14169), .SP(pll_clk_enable_45), .CD(n11584), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    FD1P3IX frame_settle__i2 (.D(n14163), .SP(pll_clk_enable_45), .CD(n11584), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i1 (.D(spi_byte_count_15__N_1694[1]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i1.GSR = "ENABLED";
    FD1P3IX init_shadow_i28 (.D(n12878), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i28.GSR = "DISABLED";
    FD1P3IX init_shadow_i27 (.D(n12872), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i27.GSR = "DISABLED";
    FD1P3IX init_shadow_i26 (.D(n12866), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i26.GSR = "DISABLED";
    FD1P3IX init_shadow_i25 (.D(n12860), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i25.GSR = "DISABLED";
    FD1P3IX init_shadow_i24 (.D(n12854), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i24.GSR = "DISABLED";
    FD1P3IX init_shadow_i23 (.D(n12848), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i23.GSR = "DISABLED";
    FD1P3IX init_shadow_i22 (.D(n12842), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i22.GSR = "DISABLED";
    FD1P3IX init_shadow_i21 (.D(n12836), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i21.GSR = "DISABLED";
    FD1P3IX init_shadow_i20 (.D(n12830), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i20.GSR = "DISABLED";
    FD1P3IX init_shadow_i19 (.D(n12824), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i19.GSR = "DISABLED";
    FD1P3IX init_shadow_i18 (.D(n12818), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i18.GSR = "DISABLED";
    FD1P3IX init_shadow_i17 (.D(n12812), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i17.GSR = "DISABLED";
    FD1P3IX init_shadow_i16 (.D(n12806), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i16.GSR = "DISABLED";
    FD1P3IX init_shadow_i15 (.D(n12800), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i15.GSR = "DISABLED";
    FD1P3IX init_shadow_i14 (.D(n12794), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i14.GSR = "DISABLED";
    FD1P3IX init_shadow_i13 (.D(n12788), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i13.GSR = "DISABLED";
    FD1P3IX init_shadow_i12 (.D(n12782), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i12.GSR = "DISABLED";
    FD1P3IX init_shadow_i11 (.D(n12776), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i11.GSR = "DISABLED";
    FD1P3IX init_shadow_i10 (.D(n12770), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i10.GSR = "DISABLED";
    FD1P3IX init_shadow_i9 (.D(n12764), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i8 (.D(n12758), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i8.GSR = "DISABLED";
    FD1P3IX init_shadow_i7 (.D(n12752), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i7.GSR = "DISABLED";
    FD1P3IX init_shadow_i6 (.D(n12746), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i6.GSR = "DISABLED";
    FD1P3IX init_shadow_i5 (.D(n12740), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i5.GSR = "DISABLED";
    FD1P3IX init_shadow_i4 (.D(n12734), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i4.GSR = "DISABLED";
    FD1P3IX init_shadow_i3 (.D(n12728), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i3.GSR = "DISABLED";
    FD1P3IX init_shadow_i2 (.D(n12722), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i2.GSR = "DISABLED";
    FD1P3IX init_shadow_i1 (.D(n12716), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i1.GSR = "DISABLED";
    FD1P3IX ev_ch_i6 (.D(ev_ch_6__N_1948[6]), .SP(pll_clk_enable_464), .CD(n16748), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_ch_i6.GSR = "DISABLED";
    FD1P3IX ev_ch_i5 (.D(ev_ch_6__N_1948[5]), .SP(pll_clk_enable_464), .CD(n16748), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_ch_i5.GSR = "DISABLED";
    FD1P3IX ev_ch_i4 (.D(ev_ch_6__N_1948[4]), .SP(pll_clk_enable_464), .CD(n16748), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_ch_i4.GSR = "DISABLED";
    FD1P3IX ev_ch_i3 (.D(ev_ch_6__N_1948[3]), .SP(pll_clk_enable_464), .CD(n16748), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_ch_i3.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i2 (.D(spi_byte_count_15__N_1694[2]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i3 (.D(spi_byte_count_15__N_1694[3]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i4 (.D(spi_byte_count_15__N_1694[4]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i5 (.D(spi_byte_count_15__N_1694[5]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i6 (.D(spi_byte_count_15__N_1694[6]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i7 (.D(spi_byte_count_15__N_1694[7]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i8 (.D(spi_byte_count_15__N_1694[8]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i9 (.D(spi_byte_count_15__N_1694[9]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i10 (.D(spi_byte_count_15__N_1694[10]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i11 (.D(spi_byte_count_15__N_1694[11]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i12 (.D(spi_byte_count_15__N_1694[12]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i13 (.D(spi_byte_count_15__N_1694[13]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i14 (.D(spi_byte_count_15__N_1694[14]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i15 (.D(spi_byte_count_15__N_1694[15]), .SP(spi1_sck_c_enable_197), 
            .CK(spi1_sck_c), .Q(spi_byte_count[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_byte_count_i0_i15.GSR = "ENABLED";
    LUT4 i6_4_lut_adj_105 (.A(mic_sample_count[3]), .B(n12), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_453)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_adj_105.init = 16'h0800;
    LUT4 i5_4_lut_adj_106 (.A(n14340), .B(spi1_sck_c_enable_197), .C(n23710), 
         .D(spi_byte_count[9]), .Z(n13)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i5_4_lut_adj_106.init = 16'h0004;
    FD1P3AX status_hold__i2 (.D(accepted_sequence[25]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i2.GSR = "DISABLED";
    FD1P3IX ev_ch_i2 (.D(ev_ch_6__N_1948[2]), .SP(pll_clk_enable_464), .CD(n16711), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_ch_i2.GSR = "DISABLED";
    LUT4 i10080_2_lut_rep_106_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .D(fpga_cs_n_c), .Z(n23695)) /* synthesis lut_function=((((D)+!C)+!B)+!A) */ ;
    defparam i10080_2_lut_rep_106_4_lut.init = 16'hff7f;
    LUT4 i14485_3_lut_3_lut (.A(status_bit_index[4]), .B(n23718), .C(status_hold[104]), 
         .Z(n23270)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14485_3_lut_3_lut.init = 16'hc4c4;
    LUT4 i14548_3_lut_3_lut (.A(status_bit_index[4]), .B(n23332), .C(n23331), 
         .Z(n23333)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14548_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_adj_107 (.A(n14465), .B(init_shadow[28]), .C(ev_bit[28]), 
         .Z(n12878)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_107.init = 16'hecec;
    LUT4 i14584_3_lut_4_lut_4_lut (.A(status_bit_index[4]), .B(n23725), 
         .C(status_hold[88]), .D(n23718), .Z(n23269)) /* synthesis lut_function=(A (B)+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14584_3_lut_4_lut_4_lut.init = 16'hd888;
    LUT4 i1_3_lut_adj_108 (.A(n14465), .B(init_shadow[27]), .C(ev_bit[27]), 
         .Z(n12872)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_108.init = 16'hecec;
    LUT4 i1_3_lut_adj_109 (.A(n14465), .B(init_shadow[26]), .C(ev_bit[26]), 
         .Z(n12866)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_109.init = 16'hecec;
    LUT4 i1_3_lut_adj_110 (.A(n14465), .B(init_shadow[25]), .C(ev_bit[25]), 
         .Z(n12860)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_110.init = 16'hecec;
    LUT4 i14517_3_lut_3_lut (.A(status_bit_index[4]), .B(n23301), .C(n23300), 
         .Z(n23302)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14517_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_adj_111 (.A(n14465), .B(init_shadow[24]), .C(ev_bit[24]), 
         .Z(n12854)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_111.init = 16'hecec;
    LUT4 i1_3_lut_adj_112 (.A(n14465), .B(init_shadow[23]), .C(ev_bit[23]), 
         .Z(n12848)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_112.init = 16'hecec;
    LUT4 mux_1408_i1_3_lut (.A(n10157), .B(n10158), .C(n10156), .Z(rd_data_15__N_2647[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i1_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_adj_113 (.A(n14465), .B(init_shadow[22]), .C(ev_bit[22]), 
         .Z(n12842)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_113.init = 16'hecec;
    LUT4 i1_3_lut_adj_114 (.A(n14465), .B(init_shadow[21]), .C(ev_bit[21]), 
         .Z(n12836)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_114.init = 16'hecec;
    LUT4 i7_4_lut_adj_115 (.A(n10155), .B(n23206), .C(n23092), .D(n6_adj_3190), 
         .Z(n10156)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7_4_lut_adj_115.init = 16'h0002;
    LUT4 i1_3_lut_adj_116 (.A(n14465), .B(init_shadow[20]), .C(ev_bit[20]), 
         .Z(n12830)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_116.init = 16'hecec;
    LUT4 i14422_4_lut (.A(n10144), .B(n23088), .C(n5), .D(n10145), .Z(n23206)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+(C+(D)))) */ ;
    defparam i14422_4_lut.init = 16'hfdfe;
    LUT4 i1_3_lut_adj_117 (.A(n14465), .B(init_shadow[1]), .C(ev_bit[1]), 
         .Z(n12716)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_117.init = 16'hecec;
    LUT4 i14308_4_lut (.A(n10146), .B(n10140), .C(n10147), .D(n10141), 
         .Z(n23092)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14308_4_lut.init = 16'h7bde;
    LUT4 i1_3_lut_adj_118 (.A(n14465), .B(init_shadow[19]), .C(ev_bit[19]), 
         .Z(n12824)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_118.init = 16'hecec;
    FD1P3IX ev_bit_i8 (.D(ev_bit[7]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i8.GSR = "DISABLED";
    FD1P3IX ev_bit_i30 (.D(ev_bit[29]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i30.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_116 (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .Z(n23705)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_rep_116.init = 16'heeee;
    LUT4 i1_3_lut_adj_119 (.A(n14465), .B(init_shadow[18]), .C(ev_bit[18]), 
         .Z(n12818)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_119.init = 16'hecec;
    LUT4 equal_1389_i6_2_lut (.A(n10150), .B(n10151), .Z(n6_adj_3190)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam equal_1389_i6_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_adj_120 (.A(n14465), .B(init_shadow[17]), .C(ev_bit[17]), 
         .Z(n12812)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_120.init = 16'hecec;
    CCU2D fpga_time_1258_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22281), .COUT(n22282), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_19.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_121 (.A(n14465), .B(init_shadow[16]), .C(ev_bit[16]), 
         .Z(n12806)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_121.init = 16'hecec;
    LUT4 i1_2_lut_rep_72_3_lut_4_lut (.A(n23668), .B(n22897), .C(spi_byte_count[2]), 
         .D(spi_byte_count[5]), .Z(n23661)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_rep_72_3_lut_4_lut.init = 16'h0004;
    LUT4 i1_3_lut_4_lut_adj_122 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[2]), 
         .D(ev_bit[2]), .Z(ev_wr_data[2])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_122.init = 16'hddd0;
    LUT4 i22_3_lut_4_lut (.A(ev_state[0]), .B(ev_state[1]), .C(ev_state[3]), 
         .D(n23046), .Z(n22771)) /* synthesis lut_function=(!(A (B (C (D))+!B ((D)+!C))+!A ((D)+!C))) */ ;
    defparam i22_3_lut_4_lut.init = 16'h08f8;
    LUT4 i14304_4_lut (.A(n10152), .B(n10142), .C(n10153), .D(n10143), 
         .Z(n23088)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14304_4_lut.init = 16'h7bde;
    LUT4 equal_1389_i5_2_lut (.A(n10148), .B(n10149), .Z(n5)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam equal_1389_i5_2_lut.init = 16'h6666;
    CCU2D fpga_time_1258_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22280), .COUT(n22281), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_17.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_647_11 (.A0(spi_rx_shift[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22258), .COUT(n22259), .S0(expected_next[11]), 
          .S1(expected_next[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_11.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_647_11.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_647_11.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_11.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_123 (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[4]), .D(spi_byte_count[5]), .Z(n16216)) /* synthesis lut_function=(A (D)+!A (B (D)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_3_lut_4_lut_adj_123.init = 16'hfe00;
    LUT4 i1_2_lut_rep_117 (.A(ev_state[1]), .B(ev_state[2]), .Z(n23706)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_117.init = 16'heeee;
    LUT4 i1_2_lut_rep_92_3_lut_4_lut (.A(ev_state[1]), .B(ev_state[2]), 
         .C(ev_state[0]), .D(ev_state[3]), .Z(pll_clk_enable_18)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i1_2_lut_rep_92_3_lut_4_lut.init = 16'h0010;
    LUT4 i1_3_lut_adj_124 (.A(n14465), .B(init_shadow[15]), .C(ev_bit[15]), 
         .Z(n12800)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_124.init = 16'hecec;
    LUT4 ev_state_3__I_0_593_i6_2_lut_rep_125 (.A(ev_state[2]), .B(ev_state[3]), 
         .Z(n23714)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[55:75])
    defparam ev_state_3__I_0_593_i6_2_lut_rep_125.init = 16'hdddd;
    CCU2D fpga_time_1258_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22279), .COUT(n22280), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_15.INJECT1_1 = "NO";
    CCU2D add_650_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22241), .COUT(n22242), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_23.INIT0 = 16'h5aaa;
    defparam add_650_23.INIT1 = 16'h5aaa;
    defparam add_650_23.INJECT1_0 = "NO";
    defparam add_650_23.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_125 (.A(ev_state[3]), .B(pll_clk_enable_347), .C(n23708), 
         .D(n23717), .Z(pll_clk_enable_464)) /* synthesis lut_function=(A (B+(C))+!A (B+(C (D)))) */ ;
    defparam i1_4_lut_adj_125.init = 16'hfcec;
    LUT4 i1_3_lut_4_lut_adj_126 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[3]), 
         .D(ev_bit[3]), .Z(ev_wr_data[3])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_126.init = 16'hddd0;
    FD1P3AX status_hold__i3 (.D(accepted_sequence[26]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[27]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[28]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[29]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[30]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[31]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[16]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[17]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[18]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[19]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[20]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[21]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[22]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[23]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(accepted_sequence[8]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(accepted_sequence[9]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(accepted_sequence[10]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(accepted_sequence[11]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(accepted_sequence[12]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(accepted_sequence[13]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(accepted_sequence[14]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(accepted_sequence[15]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(accepted_sequence[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(accepted_sequence[1]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(accepted_sequence[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(accepted_sequence[3]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(accepted_sequence[4]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(accepted_sequence[5]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(accepted_sequence[6]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(accepted_sequence[7]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[24]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[25]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[26]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[27]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[28]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[29]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[30]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[31]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[16]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[17]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[18]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[19]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[20]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[21]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[22]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[23]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(fpga_time[8]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(fpga_time[9]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(fpga_time[10]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(fpga_time[11]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(fpga_time[12]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(fpga_time[13]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(fpga_time[14]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(fpga_time[15]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(fpga_time[0]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(fpga_time[1]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(fpga_time[2]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(fpga_time[3]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(fpga_time[4]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(fpga_time[5]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX status_hold__i63 (.D(fpga_time[6]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i63.GSR = "DISABLED";
    FD1P3AX status_hold__i64 (.D(fpga_time[7]), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i64.GSR = "DISABLED";
    FD1P3AX status_hold__i65 (.D(status_flags_wire_15__N_1385[2]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i65.GSR = "DISABLED";
    FD1P3AX status_hold__i66 (.D(status_flags_wire_15__N_1401[4]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i66.GSR = "DISABLED";
    FD1P3AX status_hold__i67 (.D(n23678), .SP(fpga_cs_n_c), .CK(pll_clk), 
            .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i67.GSR = "DISABLED";
    FD1P3AX status_hold__i68 (.D(fifo_credit_wire[0]), .SP(fpga_cs_n_c), 
            .CK(pll_clk), .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam status_hold__i68.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i1 (.D(accepted_sequence_31__N_1122[1]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i1.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_98_3_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[3]), 
         .Z(n23687)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_98_3_lut.init = 16'hfefe;
    LUT4 i1_3_lut_4_lut_adj_127 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[64]), 
         .D(ev_bit[64]), .Z(ev_wr_data[64])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_127.init = 16'hddd0;
    LUT4 i6816_2_lut_3_lut_4_lut_4_lut (.A(ev_state[0]), .B(pll_clk_enable_18), 
         .C(n23707), .D(n23694), .Z(n15697)) /* synthesis lut_function=(!(A+!(B+!(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i6816_2_lut_3_lut_4_lut_4_lut.init = 16'h4544;
    LUT4 i1_3_lut_4_lut_adj_128 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[65]), 
         .D(ev_bit[65]), .Z(ev_wr_data[65])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_128.init = 16'hddd0;
    LUT4 i14340_2_lut_rep_118 (.A(ev_state[2]), .B(ev_state[3]), .Z(n23707)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14340_2_lut_rep_118.init = 16'heeee;
    LUT4 i1_2_lut_adj_129 (.A(spi_byte_count[0]), .B(n22840), .Z(spi1_sck_c_enable_111)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_129.init = 16'h4444;
    LUT4 i4_4_lut (.A(spi_byte_count[15]), .B(n22897), .C(n23154), .D(n22976), 
         .Z(n22840)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i4_4_lut.init = 16'h0004;
    LUT4 i1_2_lut_3_lut_4_lut_adj_130 (.A(ev_state[2]), .B(ev_state[3]), 
         .C(n23684), .D(ev_state[1]), .Z(n16711)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_130.init = 16'h0010;
    LUT4 i1_2_lut_rep_119 (.A(ev_state[0]), .B(ev_state[2]), .Z(n23708)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_119.init = 16'h2222;
    CCU2D fpga_time_1258_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22278), .COUT(n22279), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_13.INJECT1_1 = "NO";
    LUT4 i14370_4_lut (.A(n23692), .B(spi_byte_count[1]), .C(n22362), 
         .D(spi_byte_count[2]), .Z(n23154)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14370_4_lut.init = 16'hfffe;
    LUT4 i14193_2_lut (.A(spi_byte_count[4]), .B(spi_byte_count[3]), .Z(n22976)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14193_2_lut.init = 16'heeee;
    CCU2D expected_next_15__I_0_647_9 (.A0(spi_rx_shift[0]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22257), .COUT(n22258), .S0(expected_next[9]), 
          .S1(expected_next[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_9.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_647_9.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_647_9.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_9.INJECT1_1 = "NO";
    FD1P3AX accepted_sequence_i2 (.D(accepted_sequence_31__N_1122[2]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i3 (.D(accepted_sequence_31__N_1122[3]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i4 (.D(accepted_sequence_31__N_1122[4]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i5 (.D(accepted_sequence_31__N_1122[5]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i6 (.D(accepted_sequence_31__N_1122[6]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i7 (.D(accepted_sequence_31__N_1122[7]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i8 (.D(accepted_sequence_31__N_1122[8]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i9 (.D(accepted_sequence_31__N_1122[9]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i10 (.D(accepted_sequence_31__N_1122[10]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i11 (.D(accepted_sequence_31__N_1122[11]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i12 (.D(accepted_sequence_31__N_1122[12]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i13 (.D(accepted_sequence_31__N_1122[13]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i14 (.D(accepted_sequence_31__N_1122[14]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i15 (.D(accepted_sequence_31__N_1122[15]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i16 (.D(accepted_sequence_31__N_1122[16]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i17 (.D(accepted_sequence_31__N_1122[17]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i18 (.D(accepted_sequence_31__N_1122[18]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i19 (.D(accepted_sequence_31__N_1122[19]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i20 (.D(accepted_sequence_31__N_1122[20]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i21 (.D(accepted_sequence_31__N_1122[21]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i22 (.D(accepted_sequence_31__N_1122[22]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i23 (.D(accepted_sequence_31__N_1122[23]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i24 (.D(accepted_sequence_31__N_1122[24]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i25 (.D(accepted_sequence_31__N_1122[25]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i26 (.D(accepted_sequence_31__N_1122[26]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i27 (.D(accepted_sequence_31__N_1122[27]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i28 (.D(accepted_sequence_31__N_1122[28]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i29 (.D(accepted_sequence_31__N_1122[29]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i30 (.D(accepted_sequence_31__N_1122[30]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i31 (.D(accepted_sequence_31__N_1122[31]), .SP(pll_clk_enable_111), 
            .CK(pll_clk), .Q(accepted_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_i31.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i1 (.D(global_phase_s2[1]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i1.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i2 (.D(global_phase_s2[2]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i2.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i3 (.D(global_phase_s2[3]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i3.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i4 (.D(global_phase_s2[4]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i4.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i5 (.D(global_phase_s2[5]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i5.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i6 (.D(global_phase_s2[6]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i6.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i7 (.D(global_phase_s2[7]), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i7.GSR = "DISABLED";
    FD1P3AX run_addr_s3_i8 (.D(active_bank_N_910), .SP(pll_clk_enable_119), 
            .CK(pll_clk), .Q(run_addr_s3[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam run_addr_s3_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i1 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i1.GSR = "DISABLED";
    LUT4 i14364_3_lut (.A(spi_byte_count[12]), .B(spi_byte_count[4]), .C(spi_byte_count[3]), 
         .Z(n23148)) /* synthesis lut_function=(A+(B (C))) */ ;
    defparam i14364_3_lut.init = 16'heaea;
    LUT4 i1_2_lut_rep_97_3_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(n23686)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[55:75])
    defparam i1_2_lut_rep_97_3_lut.init = 16'hfdfd;
    LUT4 i2_3_lut_adj_131 (.A(n44_adj_3194), .B(spi_byte_count[7]), .C(spi_byte_count[6]), 
         .Z(n22925)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i2_3_lut_adj_131.init = 16'h8080;
    LUT4 i9802_3_lut_4_lut_4_lut_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n18593)) /* synthesis lut_function=((B+!(C (D)+!C !(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[55:75])
    defparam i9802_3_lut_4_lut_4_lut_3_lut_4_lut.init = 16'hdffd;
    FD1P3AX mic_shift_0_l_i0_i2 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i3 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i4 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i5 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i6 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i7 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i8 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i9 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i10 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i11 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i12 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i13 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i14 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i15 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_0_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_l_i0_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i1 (.D(accepted_sequence_spi[1]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i2 (.D(accepted_sequence_spi[2]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i3 (.D(accepted_sequence_spi[3]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i4 (.D(accepted_sequence_spi[4]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i5 (.D(accepted_sequence_spi[5]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i6 (.D(accepted_sequence_spi[6]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i7 (.D(accepted_sequence_spi[7]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i8 (.D(accepted_sequence_spi[8]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i9 (.D(accepted_sequence_spi[9]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i10 (.D(accepted_sequence_spi[10]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i11 (.D(accepted_sequence_spi[11]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i12 (.D(accepted_sequence_spi[12]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i13 (.D(accepted_sequence_spi[13]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i14 (.D(accepted_sequence_spi[14]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i15 (.D(accepted_sequence_spi[15]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i16 (.D(accepted_sequence_spi[16]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i17 (.D(accepted_sequence_spi[17]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i18 (.D(accepted_sequence_spi[18]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i19 (.D(accepted_sequence_spi[19]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i20 (.D(accepted_sequence_spi[20]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i21 (.D(accepted_sequence_spi[21]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i22 (.D(accepted_sequence_spi[22]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i23 (.D(accepted_sequence_spi[23]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i24 (.D(accepted_sequence_spi[24]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i25 (.D(accepted_sequence_spi[25]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i26 (.D(accepted_sequence_spi[26]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i27 (.D(accepted_sequence_spi[27]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i28 (.D(accepted_sequence_spi[28]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i29 (.D(accepted_sequence_spi[29]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i30 (.D(accepted_sequence_spi[30]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i31 (.D(accepted_sequence_spi[31]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_meta_i31.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i1 (.D(accepted_sequence_meta[1]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i2 (.D(accepted_sequence_meta[2]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i3 (.D(accepted_sequence_meta[3]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i4 (.D(accepted_sequence_meta[4]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i5 (.D(accepted_sequence_meta[5]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i6 (.D(accepted_sequence_meta[6]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i7 (.D(accepted_sequence_meta[7]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i8 (.D(accepted_sequence_meta[8]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i9 (.D(accepted_sequence_meta[9]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i10 (.D(accepted_sequence_meta[10]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i11 (.D(accepted_sequence_meta[11]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i12 (.D(accepted_sequence_meta[12]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i13 (.D(accepted_sequence_meta[13]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i14 (.D(accepted_sequence_meta[14]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i15 (.D(accepted_sequence_meta[15]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i16 (.D(accepted_sequence_meta[16]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i17 (.D(accepted_sequence_meta[17]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i18 (.D(accepted_sequence_meta[18]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i19 (.D(accepted_sequence_meta[19]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i20 (.D(accepted_sequence_meta[20]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i21 (.D(accepted_sequence_meta[21]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i22 (.D(accepted_sequence_meta[22]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i23 (.D(accepted_sequence_meta[23]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i24 (.D(accepted_sequence_meta[24]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i25 (.D(accepted_sequence_meta[25]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i26 (.D(accepted_sequence_meta[26]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i27 (.D(accepted_sequence_meta[27]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i28 (.D(accepted_sequence_meta[28]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i29 (.D(accepted_sequence_meta[29]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i30 (.D(accepted_sequence_meta[30]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i31 (.D(accepted_sequence_meta[31]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam accepted_sequence_sync_i31.GSR = "DISABLED";
    FD1P3IX us_tx__i2 (.D(n11408), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i2.GSR = "DISABLED";
    FD1P3IX us_tx__i3 (.D(n11410), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i3.GSR = "DISABLED";
    FD1P3IX us_tx__i4 (.D(n11412), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i4.GSR = "DISABLED";
    FD1P3IX us_tx__i5 (.D(n11414), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i5.GSR = "DISABLED";
    FD1P3IX us_tx__i6 (.D(n11416), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i6.GSR = "DISABLED";
    FD1P3IX us_tx__i7 (.D(n11418), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i7.GSR = "DISABLED";
    FD1P3IX us_tx__i8 (.D(n11420), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i8.GSR = "DISABLED";
    FD1P3IX us_tx__i9 (.D(n11422), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i9.GSR = "DISABLED";
    FD1P3IX us_tx__i10 (.D(n11424), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i10.GSR = "DISABLED";
    FD1P3IX us_tx__i11 (.D(n11426), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i11.GSR = "DISABLED";
    FD1P3IX us_tx__i12 (.D(n11428), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i12.GSR = "DISABLED";
    FD1P3IX us_tx__i13 (.D(n11430), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i13.GSR = "DISABLED";
    FD1P3IX us_tx__i14 (.D(n11432), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i14.GSR = "DISABLED";
    FD1P3IX us_tx__i15 (.D(n11434), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i15.GSR = "DISABLED";
    FD1P3IX us_tx__i16 (.D(n11436), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i16.GSR = "DISABLED";
    FD1P3IX us_tx__i17 (.D(n11438), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i17.GSR = "DISABLED";
    FD1P3IX us_tx__i18 (.D(n11440), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i18.GSR = "DISABLED";
    FD1P3IX us_tx__i19 (.D(n11442), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i19.GSR = "DISABLED";
    FD1P3IX us_tx__i20 (.D(n11444), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i20.GSR = "DISABLED";
    FD1P3IX us_tx__i21 (.D(n11446), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i21.GSR = "DISABLED";
    FD1P3IX us_tx__i22 (.D(n11448), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i22.GSR = "DISABLED";
    FD1P3IX us_tx__i23 (.D(n11450), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i23.GSR = "DISABLED";
    FD1P3IX us_tx__i24 (.D(n11452), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i24.GSR = "DISABLED";
    FD1P3IX us_tx__i25 (.D(n11454), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i25.GSR = "DISABLED";
    FD1P3IX us_tx__i26 (.D(n11456), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i26.GSR = "DISABLED";
    FD1P3IX us_tx__i27 (.D(n11458), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i27.GSR = "DISABLED";
    FD1P3IX us_tx__i28 (.D(n11460), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i28.GSR = "DISABLED";
    FD1P3IX us_tx__i29 (.D(n11462), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i29.GSR = "DISABLED";
    FD1P3IX us_tx__i30 (.D(n11464), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i30.GSR = "DISABLED";
    FD1P3IX us_tx__i31 (.D(n11466), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i31.GSR = "DISABLED";
    FD1P3IX us_tx__i32 (.D(n11468), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i32.GSR = "DISABLED";
    FD1P3IX us_tx__i33 (.D(n11470), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i33.GSR = "DISABLED";
    FD1P3IX us_tx__i34 (.D(n11472), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i34.GSR = "DISABLED";
    FD1P3IX us_tx__i35 (.D(n11474), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i35.GSR = "DISABLED";
    FD1P3IX us_tx__i36 (.D(n11476), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i36.GSR = "DISABLED";
    FD1P3IX us_tx__i37 (.D(n11478), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i37.GSR = "DISABLED";
    FD1P3IX us_tx__i38 (.D(n11480), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i38.GSR = "DISABLED";
    FD1P3IX us_tx__i39 (.D(n11482), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i39.GSR = "DISABLED";
    FD1P3IX us_tx__i40 (.D(n11484), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i40.GSR = "DISABLED";
    FD1P3IX us_tx__i41 (.D(n11486), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i41.GSR = "DISABLED";
    FD1P3IX us_tx__i42 (.D(n11488), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i42.GSR = "DISABLED";
    FD1P3IX us_tx__i43 (.D(n11490), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i43.GSR = "DISABLED";
    FD1P3IX us_tx__i44 (.D(n11492), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i44.GSR = "DISABLED";
    FD1P3IX us_tx__i45 (.D(n11494), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i45.GSR = "DISABLED";
    FD1P3IX us_tx__i46 (.D(n11496), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i46.GSR = "DISABLED";
    FD1P3IX us_tx__i47 (.D(n11498), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i47.GSR = "DISABLED";
    FD1P3IX us_tx__i48 (.D(n11500), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i48.GSR = "DISABLED";
    FD1P3IX us_tx__i49 (.D(n11502), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i49.GSR = "DISABLED";
    FD1P3IX us_tx__i50 (.D(n11504), .SP(pll_clk_enable_183), .CD(n23976), 
            .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i50.GSR = "DISABLED";
    FD1P3IX us_tx__i51 (.D(n11506), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i51.GSR = "DISABLED";
    FD1P3IX us_tx__i52 (.D(n11508), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i52.GSR = "DISABLED";
    FD1P3IX us_tx__i53 (.D(n11510), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i53.GSR = "DISABLED";
    FD1P3IX us_tx__i54 (.D(n11512), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i54.GSR = "DISABLED";
    FD1P3IX us_tx__i55 (.D(n11514), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i55.GSR = "DISABLED";
    FD1P3IX us_tx__i56 (.D(n11516), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i56.GSR = "DISABLED";
    FD1P3IX us_tx__i57 (.D(n11518), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i57.GSR = "DISABLED";
    FD1P3IX us_tx__i58 (.D(n11520), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i58.GSR = "DISABLED";
    FD1P3IX us_tx__i59 (.D(n11522), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i59.GSR = "DISABLED";
    FD1P3IX us_tx__i60 (.D(n11524), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i60.GSR = "DISABLED";
    FD1P3IX us_tx__i61 (.D(n11526), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i61.GSR = "DISABLED";
    FD1P3IX us_tx__i62 (.D(n11528), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i62.GSR = "DISABLED";
    FD1P3IX us_tx__i63 (.D(n11530), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i63.GSR = "DISABLED";
    FD1P3IX us_tx__i64 (.D(n11532), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i64.GSR = "DISABLED";
    FD1P3IX us_tx__i65 (.D(n11534), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i65.GSR = "DISABLED";
    FD1P3IX us_tx__i66 (.D(n11536), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i66.GSR = "DISABLED";
    FD1P3IX us_tx__i67 (.D(n11538), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i67.GSR = "DISABLED";
    FD1P3IX us_tx__i68 (.D(n11540), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i68.GSR = "DISABLED";
    FD1P3IX us_tx__i69 (.D(n11542), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i69.GSR = "DISABLED";
    FD1P3IX us_tx__i70 (.D(n11544), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i70.GSR = "DISABLED";
    FD1P3IX us_tx__i71 (.D(n11546), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i71.GSR = "DISABLED";
    FD1P3IX us_tx__i72 (.D(n11548), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i72.GSR = "DISABLED";
    FD1P3IX us_tx__i73 (.D(n11550), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i73.GSR = "DISABLED";
    FD1P3IX us_tx__i74 (.D(n11552), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i74.GSR = "DISABLED";
    FD1P3IX us_tx__i75 (.D(n11554), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i75.GSR = "DISABLED";
    FD1P3IX us_tx__i76 (.D(n11556), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i76.GSR = "DISABLED";
    FD1P3IX us_tx__i77 (.D(n11558), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i77.GSR = "DISABLED";
    FD1P3IX us_tx__i78 (.D(n11560), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i78.GSR = "DISABLED";
    FD1P3IX us_tx__i79 (.D(n11562), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i79.GSR = "DISABLED";
    FD1P3IX us_tx__i80 (.D(n11564), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i80.GSR = "DISABLED";
    FD1P3IX us_tx__i81 (.D(n11566), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i81.GSR = "DISABLED";
    FD1P3IX us_tx__i82 (.D(n11568), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i82.GSR = "DISABLED";
    FD1P3IX us_tx__i83 (.D(n11570), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i83.GSR = "DISABLED";
    FD1P3IX us_tx__i84 (.D(n11572), .SP(pll_clk_enable_217), .CD(n9817), 
            .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam us_tx__i84.GSR = "DISABLED";
    LUT4 i14258_2_lut (.A(spi_byte_count[5]), .B(spi_byte_count[13]), .Z(n23042)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14258_2_lut.init = 16'heeee;
    FD1P3AX ev_state_i2 (.D(ev_state_3__N_697[2]), .SP(pll_clk_enable_729), 
            .CK(pll_clk), .Q(ev_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_state_i2.GSR = "DISABLED";
    FD1S3AX ev_state_i3 (.D(n22771), .CK(pll_clk), .Q(ev_state[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_state_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i1 (.D(staging_q[9]), .SP(pll_clk_enable_233), .CK(pll_clk), 
            .Q(build_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i1.GSR = "DISABLED";
    FD1P3AX build_phase_i2 (.D(staging_q[10]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i2.GSR = "DISABLED";
    FD1P3AX build_phase_i3 (.D(staging_q[11]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i4 (.D(staging_q[12]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i4.GSR = "DISABLED";
    FD1P3AX build_phase_i5 (.D(staging_q[13]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i5.GSR = "DISABLED";
    FD1P3AX build_phase_i6 (.D(staging_q[14]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i6.GSR = "DISABLED";
    FD1P3AX build_phase_i7 (.D(staging_q[15]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_phase_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i1 (.D(build_sum_8__N_2053[1]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i1.GSR = "DISABLED";
    FD1P3AX build_sum_i2 (.D(build_sum_8__N_2053[2]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i2.GSR = "DISABLED";
    FD1P3AX build_sum_i3 (.D(build_sum_8__N_2053[3]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i3.GSR = "DISABLED";
    FD1P3AX build_sum_i4 (.D(build_sum_8__N_2053[4]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i4.GSR = "DISABLED";
    FD1P3AX build_sum_i5 (.D(build_sum_8__N_2053[5]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i5.GSR = "DISABLED";
    FD1P3AX build_sum_i6 (.D(build_sum_8__N_2053[6]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i6.GSR = "DISABLED";
    FD1P3AX build_sum_i7 (.D(build_sum_8__N_2053[7]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i8 (.D(build_sum_8__N_2053[8]), .SP(pll_clk_enable_233), 
            .CK(pll_clk), .Q(build_sum[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam build_sum_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i1.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i2.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i3.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i4.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i5.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i6.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i7.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_282), .CK(pll_clk), 
            .Q(ev_rd_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i9.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i10.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i11.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i12.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i13.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i14.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i15.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i16.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i17.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i18.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i19.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i20.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i21.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i22.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i23.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i24.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i25.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i26.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i27.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i28.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i29.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i30.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i31.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i32.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i33.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i34.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i35.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i36.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i37.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i38.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i39.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i40.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i41.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i42.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i43.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i44.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i45.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i46.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i47.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i48.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_282), 
            .CK(pll_clk), .Q(ev_rd_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i49.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i50.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i51.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i52.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i53.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i54.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i55.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i56.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i57.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i58.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i59.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i60.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i61.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i62.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i63.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i64.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i65.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i66.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i67.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i68.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i69.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i70.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i71.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i72.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i73.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i74.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i75.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i76.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i77.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i78.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i79.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i80.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i81.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i82.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_316), 
            .CK(pll_clk), .Q(ev_rd_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_rd_hold_i83.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut_4_lut_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[0]), .D(ev_state[1]), .Z(ev_we)) /* synthesis lut_function=(!(A (B+(C+!(D)))+!A ((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(221[55:75])
    defparam i2_3_lut_4_lut_4_lut_4_lut_4_lut.init = 16'h0250;
    LUT4 i1_3_lut_adj_132 (.A(n14465), .B(init_shadow[14]), .C(ev_bit[14]), 
         .Z(n12794)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_132.init = 16'hecec;
    LUT4 i1_3_lut_adj_133 (.A(n14465), .B(init_shadow[13]), .C(ev_bit[13]), 
         .Z(n12788)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_133.init = 16'hecec;
    LUT4 i1_2_lut_rep_99_3_lut (.A(ev_state[0]), .B(ev_state[2]), .C(ev_state[3]), 
         .Z(n23688)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_99_3_lut.init = 16'h2020;
    LUT4 i1_3_lut_adj_134 (.A(n14465), .B(init_shadow[50]), .C(ev_bit[50]), 
         .Z(n13014)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_134.init = 16'hecec;
    LUT4 i1_2_lut_3_lut_4_lut_adj_135 (.A(ev_state[0]), .B(ev_state[2]), 
         .C(n20258), .D(ev_state[3]), .Z(n22891)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_3_lut_4_lut_adj_135.init = 16'h0200;
    LUT4 i1_3_lut_adj_136 (.A(n14465), .B(init_shadow[12]), .C(ev_bit[12]), 
         .Z(n12782)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_136.init = 16'hecec;
    LUT4 i1_3_lut_adj_137 (.A(spi_bit_count[0]), .B(spi_bit_count[2]), .C(spi_bit_count[1]), 
         .Z(n22897)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_3_lut_adj_137.init = 16'h8080;
    LUT4 i14532_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[61]), 
         .C(status_hold[60]), .Z(n23317)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14532_3_lut_3_lut.init = 16'he4e4;
    LUT4 i12007_1_lut (.A(status_bit_index[1]), .Z(spi1_miso_N_2513[1])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam i12007_1_lut.init = 16'h5555;
    LUT4 i13452_2_lut (.A(spi_channel_field[1]), .B(spi_channel_field[0]), 
         .Z(n14)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(333[78:102])
    defparam i13452_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_138 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[0]), 
         .D(ev_bit[0]), .Z(ev_wr_data[0])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_138.init = 16'hddd0;
    LUT4 i14533_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[63]), 
         .C(status_hold[62]), .Z(n23318)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14533_3_lut_3_lut.init = 16'he4e4;
    FD1S3AX staging_rd_addr_i1 (.D(staging_rd_addr_6__N_901[1]), .CK(pll_clk), 
            .Q(staging_rd_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam staging_rd_addr_i1.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i2 (.D(staging_rd_addr_6__N_901[2]), .CK(pll_clk), 
            .Q(staging_rd_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam staging_rd_addr_i2.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i3 (.D(staging_rd_addr_6__N_901[3]), .CK(pll_clk), 
            .Q(staging_rd_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam staging_rd_addr_i3.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i4 (.D(staging_rd_addr_6__N_901[4]), .CK(pll_clk), 
            .Q(staging_rd_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam staging_rd_addr_i4.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i5 (.D(staging_rd_addr_6__N_901[5]), .CK(pll_clk), 
            .Q(staging_rd_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam staging_rd_addr_i5.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i6 (.D(staging_rd_addr_6__N_901[6]), .CK(pll_clk), 
            .Q(staging_rd_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam staging_rd_addr_i6.GSR = "DISABLED";
    LUT4 i14530_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[57]), 
         .C(status_hold[56]), .Z(n23315)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14530_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14528_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[53]), 
         .C(status_hold[52]), .Z(n23313)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14528_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX pending_sequence_i1 (.D(accepted_sequence_sync[1]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i2 (.D(accepted_sequence_sync[2]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i3 (.D(accepted_sequence_sync[3]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i4 (.D(accepted_sequence_sync[4]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i5 (.D(accepted_sequence_sync[5]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i6 (.D(accepted_sequence_sync[6]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i7 (.D(accepted_sequence_sync[7]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i8 (.D(accepted_sequence_sync[8]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i9 (.D(accepted_sequence_sync[9]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i10 (.D(accepted_sequence_sync[10]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i11 (.D(accepted_sequence_sync[11]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i12 (.D(accepted_sequence_sync[12]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i13 (.D(accepted_sequence_sync[13]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i14 (.D(accepted_sequence_sync[14]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i15 (.D(accepted_sequence_sync[15]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i15.GSR = "DISABLED";
    FD1P3AX pending_sequence_i16 (.D(accepted_sequence_sync[16]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i16.GSR = "DISABLED";
    FD1P3AX pending_sequence_i17 (.D(accepted_sequence_sync[17]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i17.GSR = "DISABLED";
    FD1P3AX pending_sequence_i18 (.D(accepted_sequence_sync[18]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i18.GSR = "DISABLED";
    FD1P3AX pending_sequence_i19 (.D(accepted_sequence_sync[19]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i19.GSR = "DISABLED";
    FD1P3AX pending_sequence_i20 (.D(accepted_sequence_sync[20]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i20.GSR = "DISABLED";
    FD1P3AX pending_sequence_i21 (.D(accepted_sequence_sync[21]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i21.GSR = "DISABLED";
    FD1P3AX pending_sequence_i22 (.D(accepted_sequence_sync[22]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i22.GSR = "DISABLED";
    FD1P3AX pending_sequence_i23 (.D(accepted_sequence_sync[23]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i23.GSR = "DISABLED";
    FD1P3AX pending_sequence_i24 (.D(accepted_sequence_sync[24]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i24.GSR = "DISABLED";
    FD1P3AX pending_sequence_i25 (.D(accepted_sequence_sync[25]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i25.GSR = "DISABLED";
    FD1P3AX pending_sequence_i26 (.D(accepted_sequence_sync[26]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i26.GSR = "DISABLED";
    FD1P3AX pending_sequence_i27 (.D(accepted_sequence_sync[27]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i27.GSR = "DISABLED";
    FD1P3AX pending_sequence_i28 (.D(accepted_sequence_sync[28]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i28.GSR = "DISABLED";
    FD1P3AX pending_sequence_i29 (.D(accepted_sequence_sync[29]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i29.GSR = "DISABLED";
    FD1P3AX pending_sequence_i30 (.D(accepted_sequence_sync[30]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i30.GSR = "DISABLED";
    FD1P3AX pending_sequence_i31 (.D(accepted_sequence_sync[31]), .SP(pll_clk_enable_347), 
            .CK(pll_clk), .Q(pending_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam pending_sequence_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i1 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i2 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i3 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i4 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i5 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i6 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i7 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i8 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i9 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i10 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i11 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i12 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i13 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i14 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i15 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_362), 
            .CK(pll_clk), .Q(mic_shift_1_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i2 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i3 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i4 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i5 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i6 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i7 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i8 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i9 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i10 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i11 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i12 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i13 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i14 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i15 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_0_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_0_r__i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i2 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i3 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i4 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i5 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i6 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i7 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i8 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i9 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i10 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i11 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i12 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i13 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i14 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i15 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_shift_1_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_shift_1_r__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1_r[14]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_1_l[15]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i32 (.D(mic_data_0_c), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i33 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i34 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i35 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i36 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i37 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i38 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i39 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i40 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i41 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i42 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i43 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i44 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i45 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i46 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i46.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i47 (.D(mic_shift_0_r[14]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i47.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i48 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i48.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i49 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_439), 
            .CK(pll_clk), .Q(mic_latest[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i49.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i50 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i50.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i51 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i51.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i52 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i52.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i53 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i53.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i54 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i54.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i55 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i55.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i56 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i56.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i57 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i57.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i58 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i58.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i59 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i59.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i60 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i60.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i61 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i61.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i62 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i62.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i63 (.D(mic_shift_0_l[15]), .SP(pll_clk_enable_453), 
            .CK(pll_clk), .Q(mic_latest[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mic_latest_i0_i63.GSR = "DISABLED";
    LUT4 i14529_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[55]), 
         .C(status_hold[54]), .Z(n23314)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14529_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX init_shadow_i59 (.D(n13068), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i59.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_139 (.A(n14465), .B(init_shadow[11]), .C(ev_bit[11]), 
         .Z(n12776)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_139.init = 16'hecec;
    CCU2D add_650_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22240), .COUT(n22241), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_21.INIT0 = 16'h5aaa;
    defparam add_650_21.INIT1 = 16'h5555;
    defparam add_650_21.INJECT1_0 = "NO";
    defparam add_650_21.INJECT1_1 = "NO";
    LUT4 i14527_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[51]), 
         .C(status_hold[50]), .Z(n23312)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14527_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_adj_140 (.A(n14465), .B(init_shadow[10]), .C(ev_bit[10]), 
         .Z(n12770)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_140.init = 16'hecec;
    LUT4 i14526_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[49]), 
         .C(status_hold[48]), .Z(n23311)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14526_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14531_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[59]), 
         .C(status_hold[58]), .Z(n23316)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14531_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX ev_bit_i7 (.D(ev_bit[6]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i7.GSR = "DISABLED";
    FD1P3IX ev_ch_i1 (.D(ev_ch_6__N_1948[1]), .SP(pll_clk_enable_464), .CD(n16711), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_ch_i1.GSR = "DISABLED";
    LUT4 i14523_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[43]), 
         .C(status_hold[42]), .Z(n23308)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14523_3_lut_3_lut.init = 16'he4e4;
    CCU2D fpga_time_1258_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22277), .COUT(n22278), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_11.INJECT1_1 = "NO";
    CCU2D fpga_time_1258_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22276), .COUT(n22277), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_9.INJECT1_1 = "NO";
    LUT4 i14522_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[41]), 
         .C(status_hold[40]), .Z(n23307)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14522_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14521_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[39]), 
         .C(status_hold[38]), .Z(n23306)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14521_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14520_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[37]), 
         .C(status_hold[36]), .Z(n23305)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14520_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14525_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[47]), 
         .C(status_hold[46]), .Z(n23310)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14525_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14519_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[35]), 
         .C(status_hold[34]), .Z(n23304)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14519_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14518_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[33]), 
         .C(status_hold[32]), .Z(n23303)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14518_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14502_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[31]), 
         .C(status_hold[30]), .Z(n23287)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14502_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14501_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[29]), 
         .C(status_hold[28]), .Z(n23286)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14501_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX ev_state_i1 (.D(ev_state_3__N_697[1]), .SP(pll_clk_enable_729), 
            .CK(pll_clk), .Q(ev_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_state_i1.GSR = "DISABLED";
    FD1P3IX frame_settle__i3 (.D(frame_settle_3__N_1832[3]), .SP(pll_clk_enable_484), 
            .CD(pll_clk_enable_6), .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    FD1P3IX rgb_hold__i8 (.D(rgb_values[7]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i8.GSR = "DISABLED";
    FD1P3IX rgb_hold__i7 (.D(rgb_values[6]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i7.GSR = "DISABLED";
    FD1P3IX rgb_hold__i6 (.D(rgb_values[5]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i6.GSR = "DISABLED";
    FD1P3IX rgb_hold__i5 (.D(rgb_values[4]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i5.GSR = "DISABLED";
    FD1P3IX rgb_hold__i4 (.D(rgb_values[3]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i4.GSR = "DISABLED";
    FD1P3IX rgb_hold__i3 (.D(rgb_values[2]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i3.GSR = "DISABLED";
    FD1P3IX rgb_hold__i2 (.D(rgb_values[1]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i2.GSR = "DISABLED";
    FD1P3IX spi_channel_field_1255__i0 (.D(n28), .SP(spi1_sck_c_enable_237), 
            .CD(n15590), .CK(spi1_sck_c), .Q(spi_channel_field[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(333[78:102])
    defparam spi_channel_field_1255__i0.GSR = "ENABLED";
    FD1P3AX stop_toggle_spi_483 (.D(stop_toggle_spi_N_2537), .SP(spi1_sck_c_enable_199), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam stop_toggle_spi_483.GSR = "DISABLED";
    FD1S3IX wrap_s2_489 (.D(wrap_s2_N_2572), .CK(pll_clk), .CD(n15587), 
            .Q(wrap_s2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam wrap_s2_489.GSR = "DISABLED";
    FD1P3IX ev_bit_i83 (.D(ev_bit[82]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i83.GSR = "DISABLED";
    FD1P3IX ev_bit_i82 (.D(ev_bit[81]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i82.GSR = "DISABLED";
    FD1P3IX ev_bit_i81 (.D(ev_bit[80]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i81.GSR = "DISABLED";
    FD1P3IX ev_bit_i80 (.D(ev_bit[79]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i80.GSR = "DISABLED";
    FD1P3IX ev_bit_i79 (.D(ev_bit[78]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i79.GSR = "DISABLED";
    FD1P3IX ev_bit_i78 (.D(ev_bit[77]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i78.GSR = "DISABLED";
    FD1P3IX ev_bit_i77 (.D(ev_bit[76]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i77.GSR = "DISABLED";
    FD1P3IX ev_bit_i76 (.D(ev_bit[75]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i76.GSR = "DISABLED";
    FD1P3IX ev_bit_i75 (.D(ev_bit[74]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i75.GSR = "DISABLED";
    FD1P3IX ev_bit_i74 (.D(ev_bit[73]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i74.GSR = "DISABLED";
    FD1P3IX ev_bit_i73 (.D(ev_bit[72]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i73.GSR = "DISABLED";
    FD1P3IX ev_bit_i72 (.D(ev_bit[71]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i72.GSR = "DISABLED";
    FD1P3IX ev_bit_i71 (.D(ev_bit[70]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i71.GSR = "DISABLED";
    FD1P3IX ev_bit_i70 (.D(ev_bit[69]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i70.GSR = "DISABLED";
    FD1P3IX ev_bit_i69 (.D(ev_bit[68]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i69.GSR = "DISABLED";
    FD1P3IX ev_bit_i68 (.D(ev_bit[67]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i68.GSR = "DISABLED";
    FD1P3IX ev_bit_i67 (.D(ev_bit[66]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i67.GSR = "DISABLED";
    FD1P3IX ev_bit_i66 (.D(ev_bit[65]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i66.GSR = "DISABLED";
    FD1P3IX ev_bit_i65 (.D(ev_bit[64]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i65.GSR = "DISABLED";
    FD1P3IX ev_bit_i64 (.D(ev_bit[63]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i64.GSR = "DISABLED";
    FD1P3IX ev_bit_i63 (.D(ev_bit[62]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i63.GSR = "DISABLED";
    FD1P3IX ev_bit_i62 (.D(ev_bit[61]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i62.GSR = "DISABLED";
    FD1P3IX ev_bit_i61 (.D(ev_bit[60]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i61.GSR = "DISABLED";
    FD1P3IX ev_bit_i60 (.D(ev_bit[59]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i60.GSR = "DISABLED";
    FD1P3IX ev_bit_i59 (.D(ev_bit[58]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i59.GSR = "DISABLED";
    FD1P3IX ev_bit_i58 (.D(ev_bit[57]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i58.GSR = "DISABLED";
    FD1P3IX ev_bit_i57 (.D(ev_bit[56]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i57.GSR = "DISABLED";
    FD1P3IX ev_bit_i56 (.D(ev_bit[55]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i56.GSR = "DISABLED";
    FD1P3IX ev_bit_i55 (.D(ev_bit[54]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i55.GSR = "DISABLED";
    FD1P3IX ev_bit_i54 (.D(ev_bit[53]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i54.GSR = "DISABLED";
    FD1P3IX ev_bit_i51 (.D(ev_bit[50]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i51.GSR = "DISABLED";
    FD1P3IX ev_bit_i50 (.D(ev_bit[49]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i50.GSR = "DISABLED";
    FD1P3IX ev_bit_i49 (.D(ev_bit[48]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i49.GSR = "DISABLED";
    FD1P3IX ev_bit_i48 (.D(ev_bit[47]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i48.GSR = "DISABLED";
    FD1P3IX ev_bit_i47 (.D(ev_bit[46]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i47.GSR = "DISABLED";
    LUT4 i14500_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[27]), 
         .C(status_hold[26]), .Z(n23285)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14500_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14499_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[25]), 
         .C(status_hold[24]), .Z(n23284)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14499_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14498_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[23]), 
         .C(status_hold[22]), .Z(n23283)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14498_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14524_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[45]), 
         .C(status_hold[44]), .Z(n23309)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14524_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14497_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[21]), 
         .C(status_hold[20]), .Z(n23282)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14497_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14496_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[19]), 
         .C(status_hold[18]), .Z(n23281)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14496_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14495_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[17]), 
         .C(status_hold[16]), .Z(n23280)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14495_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14494_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[15]), 
         .C(status_hold[14]), .Z(n23279)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14494_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14493_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[13]), 
         .C(status_hold[12]), .Z(n23278)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14493_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14492_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[11]), 
         .C(status_hold[10]), .Z(n23277)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14492_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX ev_run_hold_s5_i0_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i1.GSR = "DISABLED";
    LUT4 m1_lut (.Z(n23951)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    LUT4 i1_4_lut_adj_141 (.A(n17), .B(spi_command[0]), .C(n23735), .D(spi_command[4]), 
         .Z(n14025)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(351[34:54])
    defparam i1_4_lut_adj_141.init = 16'hfbff;
    LUT4 i14491_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[9]), 
         .C(status_hold[8]), .Z(n23276)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14491_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14490_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[7]), 
         .C(status_hold[6]), .Z(n23275)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14490_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_142 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[4]), 
         .D(ev_bit[4]), .Z(ev_wr_data[4])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_142.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_143 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[5]), 
         .D(ev_bit[5]), .Z(ev_wr_data[5])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_143.init = 16'hddd0;
    LUT4 i14489_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[5]), 
         .C(status_hold[4]), .Z(n23274)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14489_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14488_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[3]), 
         .C(status_hold[2]), .Z(n23273)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14488_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14487_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[1]), 
         .C(status_hold[0]), .Z(n23272)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i14487_3_lut_3_lut.init = 16'he4e4;
    LUT4 i13492_2_lut_rep_126 (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n23715)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam i13492_2_lut_rep_126.init = 16'h8888;
    LUT4 i13503_2_lut_3_lut_4_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_sample_count[3]), .D(mic_sample_count[2]), .Z(n27)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam i13503_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i5_3_lut_4_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_tick), .D(mic_sample_count[2]), .Z(n12)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam i5_3_lut_4_lut.init = 16'h8000;
    LUT4 i13496_2_lut_3_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .C(mic_sample_count[2]), .Z(n28_adj_3177)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam i13496_2_lut_3_lut.init = 16'h7878;
    LUT4 i1_3_lut_4_lut_adj_144 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[6]), 
         .D(ev_bit[6]), .Z(ev_wr_data[6])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_144.init = 16'hddd0;
    LUT4 i1432_2_lut_rep_127 (.A(frame_settle[1]), .B(frame_settle[0]), 
         .Z(n23716)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(441[29:48])
    defparam i1432_2_lut_rep_127.init = 16'heeee;
    LUT4 i4357_4_lut (.A(ev_ch[6]), .B(staging_rd_addr[6]), .C(n22929), 
         .D(n22914), .Z(staging_rd_addr_6__N_901[6])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i4357_4_lut.init = 16'hac0c;
    LUT4 i1_2_lut_3_lut_adj_145 (.A(frame_settle[1]), .B(frame_settle[0]), 
         .C(frame_settle[2]), .Z(n14163)) /* synthesis lut_function=(A (C)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(441[29:48])
    defparam i1_2_lut_3_lut_adj_145.init = 16'he1e1;
    LUT4 i1_2_lut_rep_128 (.A(ev_state[1]), .B(ev_state_3__N_1940[1]), .Z(n23717)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_128.init = 16'h4444;
    LUT4 i1_4_lut_4_lut (.A(ev_state[1]), .B(ev_state_3__N_1940[1]), .C(ev_state[0]), 
         .D(n23707), .Z(n22929)) /* synthesis lut_function=(!(A (C+(D))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_4_lut_4_lut.init = 16'h004a;
    LUT4 i2_3_lut_rep_129 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_bit_index[1]), .Z(n23718)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam i2_3_lut_rep_129.init = 16'h8080;
    LUT4 i1_3_lut_4_lut_adj_146 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[7]), 
         .D(ev_bit[7]), .Z(ev_wr_data[7])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_146.init = 16'hddd0;
    LUT4 i1_2_lut_4_lut_adj_147 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_bit_index[1]), .D(status_bit_index[6]), .Z(n6)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam i1_2_lut_4_lut_adj_147.init = 16'h8000;
    FD1P3AX ev_run_hold_s5_i0_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i2.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i3.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i4.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i5.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i6.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i7.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i8.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i9.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i10.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i11.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i12.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i13.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i14.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i15.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i16.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i17.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i18.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i19.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i20.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i21.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i22.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i23.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i24.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i25.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i26.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i27.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i28.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i29.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i30.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i31.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i32.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i33.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i34.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i35.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i36.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i37.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i38.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i39.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i40.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i41.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i42.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i43.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i44.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i45.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i46.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i47.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i48.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_576), 
            .CK(pll_clk), .Q(ev_run_hold_s5[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i49.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i50.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i51.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i52.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i53.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i54.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i55.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i56.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i57.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i58.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i59.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i60.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i61.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i62.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i63.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i64.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i65.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i66.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i67.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i68.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i69.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i70.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i71.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i72.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i73.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i74.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i75.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i76.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i77.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i78.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i79.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i80.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i81.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i82.GSR = "DISABLED";
    FD1P3AX ev_run_hold_s5_i0_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_610), 
            .CK(pll_clk), .Q(ev_run_hold_s5[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_run_hold_s5_i0_i83.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    LUT4 i1_4_lut_then_4_lut (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n23724)) /* synthesis lut_function=(!((B (D)+!B !(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(127[17:33])
    defparam i1_4_lut_then_4_lut.init = 16'h2088;
    LUT4 i1_3_lut_4_lut_adj_148 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[8]), 
         .D(ev_bit[8]), .Z(ev_wr_data[8])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_148.init = 16'hddd0;
    LUT4 i1_2_lut_3_lut_4_lut_adj_149 (.A(ev_state[1]), .B(ev_state[2]), 
         .C(ev_state[3]), .D(ev_state[0]), .Z(pll_clk_enable_233)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_3_lut_4_lut_adj_149.init = 16'h0020;
    LUT4 i11427_4_lut_4_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[3]), 
         .D(n7), .Z(n20216)) /* synthesis lut_function=(!((B (C)+!B (C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i11427_4_lut_4_lut.init = 16'h0a2a;
    LUT4 i1_3_lut_4_lut_adj_150 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[9]), 
         .D(ev_bit[9]), .Z(ev_wr_data[9])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_150.init = 16'hddd0;
    LUT4 i1_2_lut_rep_130 (.A(swap_pending), .B(frame_req), .Z(n23719)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_130.init = 16'h4444;
    LUT4 i11508_3_lut_rep_95_4_lut (.A(swap_pending), .B(frame_req), .C(ev_state[0]), 
         .D(ev_state_3__N_1940[1]), .Z(n23684)) /* synthesis lut_function=(A (C (D))+!A (B ((D)+!C)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i11508_3_lut_rep_95_4_lut.init = 16'hf404;
    LUT4 i1_3_lut_4_lut_adj_151 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[10]), 
         .D(ev_bit[10]), .Z(ev_wr_data[10])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_151.init = 16'hddd0;
    LUT4 i1_2_lut_rep_105_3_lut (.A(swap_pending), .B(frame_req), .C(ev_state[1]), 
         .Z(n23694)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_105_3_lut.init = 16'h0404;
    LUT4 active_bank_I_0_1_lut_rep_131 (.A(active_bank), .Z(n23720)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(217[34:46])
    defparam active_bank_I_0_1_lut_rep_131.init = 16'h5555;
    LUT4 i4355_4_lut (.A(ev_ch[5]), .B(staging_rd_addr[5]), .C(n22929), 
         .D(n22914), .Z(staging_rd_addr_6__N_901[5])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i4355_4_lut.init = 16'hac0c;
    LUT4 i1_3_lut_adj_152 (.A(n14465), .B(init_shadow[83]), .C(ev_bit[83]), 
         .Z(n13225)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_152.init = 16'hecec;
    LUT4 i1_3_lut_adj_153 (.A(n14465), .B(init_shadow[82]), .C(ev_bit[82]), 
         .Z(n13219)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_153.init = 16'hecec;
    LUT4 i1_3_lut_adj_154 (.A(n14465), .B(init_shadow[81]), .C(ev_bit[81]), 
         .Z(n13213)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_154.init = 16'hecec;
    LUT4 i1_3_lut_adj_155 (.A(n14465), .B(init_shadow[80]), .C(ev_bit[80]), 
         .Z(n13207)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_155.init = 16'hecec;
    LUT4 i1_3_lut_adj_156 (.A(n14465), .B(init_shadow[79]), .C(ev_bit[79]), 
         .Z(n13199)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_156.init = 16'hecec;
    LUT4 i1_3_lut_adj_157 (.A(n14465), .B(init_shadow[78]), .C(ev_bit[78]), 
         .Z(n13193)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_157.init = 16'hecec;
    LUT4 i1_3_lut_adj_158 (.A(n14465), .B(init_shadow[77]), .C(ev_bit[77]), 
         .Z(n13187)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_158.init = 16'hecec;
    LUT4 i1_3_lut_adj_159 (.A(n14465), .B(init_shadow[76]), .C(ev_bit[76]), 
         .Z(n13181)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_159.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_160 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[11]), 
         .D(ev_bit[11]), .Z(ev_wr_data[11])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_160.init = 16'hddd0;
    LUT4 i1_3_lut_adj_161 (.A(n14465), .B(init_shadow[75]), .C(ev_bit[75]), 
         .Z(n13175)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_161.init = 16'hecec;
    LUT4 i1_3_lut_adj_162 (.A(n14465), .B(init_shadow[74]), .C(ev_bit[74]), 
         .Z(n13163)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_162.init = 16'hecec;
    LUT4 i1_3_lut_adj_163 (.A(n14465), .B(init_shadow[73]), .C(ev_bit[73]), 
         .Z(n13157)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_163.init = 16'hecec;
    LUT4 i1_3_lut_adj_164 (.A(n14465), .B(init_shadow[72]), .C(ev_bit[72]), 
         .Z(n13151)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_164.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_165 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[30]), 
         .D(ev_bit[30]), .Z(ev_wr_data[30])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_165.init = 16'hddd0;
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i16 (.D(spi_frame_sequence[16]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i17 (.D(spi_frame_sequence[17]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i18 (.D(spi_frame_sequence[18]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i19 (.D(spi_frame_sequence[19]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i20 (.D(spi_frame_sequence[20]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i21 (.D(spi_frame_sequence[21]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i22 (.D(spi_frame_sequence[22]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i23 (.D(spi_frame_sequence[23]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i24 (.D(spi_frame_sequence[24]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i25 (.D(spi_frame_sequence[25]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i26 (.D(spi_frame_sequence[26]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i27 (.D(spi_frame_sequence[27]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i28 (.D(spi_frame_sequence[28]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i29 (.D(spi_frame_sequence[29]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i30 (.D(spi_frame_sequence[30]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i31 (.D(spi_frame_sequence[31]), .SP(spi1_sck_c_enable_230), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam accepted_sequence_spi_i0_i31.GSR = "DISABLED";
    FD1P3IX init_shadow_i37 (.D(n12932), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i37.GSR = "DISABLED";
    FD1P3AX spi_channel_index_1254__i1 (.D(n39_adj_3170), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254__i1.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1254__i2 (.D(n38_adj_3171), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254__i2.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1254__i3 (.D(n37_adj_3172), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254__i3.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1254__i4 (.D(n36_adj_3173), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254__i4.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1254__i5 (.D(n35_adj_3174), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254__i5.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1254__i6 (.D(n34_adj_3175), .SP(spi1_sck_c_enable_236), 
            .CK(spi1_sck_c), .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254__i6.GSR = "ENABLED";
    FD1S3AX time_divider_1259__i1 (.D(n39_adj_3160), .CK(pll_clk), .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259__i1.GSR = "DISABLED";
    FD1S3AX time_divider_1259__i2 (.D(n38_adj_3161), .CK(pll_clk), .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259__i2.GSR = "DISABLED";
    FD1S3AX time_divider_1259__i3 (.D(n37_adj_3162), .CK(pll_clk), .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259__i3.GSR = "DISABLED";
    FD1S3AX time_divider_1259__i4 (.D(n36_adj_3163), .CK(pll_clk), .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1259__i5 (.D(n35_adj_3164), .CK(pll_clk), .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259__i5.GSR = "DISABLED";
    FD1S3AX time_divider_1259__i6 (.D(n34_adj_3165), .CK(pll_clk), .Q(time_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259__i6.GSR = "DISABLED";
    FD1P3AX status_bit_index_1253__i1 (.D(n39), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253__i1.GSR = "ENABLED";
    FD1P3AX status_bit_index_1253__i2 (.D(n38), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253__i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_1253__i3 (.D(n37), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253__i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_1253__i4 (.D(n36), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253__i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_1253__i5 (.D(n35), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253__i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_1253__i6 (.D(n34), .SP(spi1_sck_N_416_enable_7), 
            .CK(spi1_sck_N_416), .Q(status_bit_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253__i6.GSR = "ENABLED";
    FD1P3AX fpga_time_1258__i1 (.D(n164), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i1.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i9_3_lut_3_lut (.A(active_bank), .B(n22343), 
         .C(run_addr_s3[8]), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (B (C))+!A ((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(217[34:46])
    defparam run_addr_s3_8__I_0_i9_3_lut_3_lut.init = 16'hd1d1;
    PFUMX i103645_i1 (.BLUT(n23302), .ALUT(n23333), .C0(spi1_miso_N_2513[5]), 
          .Z(n63));
    FD1P3AX fpga_time_1258__i2 (.D(n163), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i2.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i3 (.D(n162), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i3.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i4 (.D(n161), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i4.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i5 (.D(n160), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i5.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i6 (.D(n159), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i6.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i7 (.D(n158), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i7.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i8 (.D(n157), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i8.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i9 (.D(n156), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i9.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i10 (.D(n155), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i10.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i11 (.D(n154), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i11.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i12 (.D(n153), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i12.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i13 (.D(n152), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i13.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i14 (.D(n151), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i14.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i15 (.D(n150), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i15.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i16 (.D(n149), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i16.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i17 (.D(n148), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i17.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i18 (.D(n147), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i18.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i19 (.D(n146), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i19.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i20 (.D(n145), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i20.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i21 (.D(n144), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i21.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i22 (.D(n143), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i22.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i23 (.D(n142), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i23.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i24 (.D(n141), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i24.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i25 (.D(n140), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i25.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i26 (.D(n139), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i26.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i27 (.D(n138), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i27.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i28 (.D(n137), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i28.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i29 (.D(n136), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i29.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i30 (.D(n135), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i30.GSR = "DISABLED";
    FD1P3AX fpga_time_1258__i31 (.D(n134), .SP(pll_clk_enable_642), .CK(pll_clk), 
            .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258__i31.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1257__i1 (.D(n44), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i1.GSR = "DISABLED";
    FD1P3IX init_shadow_i49 (.D(n13008), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i49.GSR = "DISABLED";
    LUT4 spi1_miso_I_15_i127_4_lut (.A(n23271), .B(n63), .C(status_bit_index[6]), 
         .D(status_bit_index[3]), .Z(spi1_miso_N_2512)) /* synthesis lut_function=(A (B (C+!(D))+!B !(C+(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(280[55:80])
    defparam spi1_miso_I_15_i127_4_lut.init = 16'hc0ca;
    FD1P3IX ev_bit_i29 (.D(ev_bit[28]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i29.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_166 (.A(spi_command[1]), .B(spi1_sck_c_enable_40), 
         .C(n8_adj_3193), .D(spi_command[4]), .Z(spi1_sck_c_enable_38)) /* synthesis lut_function=(A (B)+!A (B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_4_lut_adj_166.init = 16'hccc8;
    FD1P3IX ev_bit_i28 (.D(ev_bit[27]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i28.GSR = "DISABLED";
    LUT4 i3_3_lut (.A(n22966), .B(spi_command[0]), .C(invalid_frame_spi_N_2544), 
         .Z(n8_adj_3193)) /* synthesis lut_function=(A+!(B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[58:78])
    defparam i3_3_lut.init = 16'hbfbf;
    FD1P3IX ev_bit_i27 (.D(ev_bit[26]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i27.GSR = "DISABLED";
    FD1P3IX ev_bit_i26 (.D(ev_bit[25]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i26.GSR = "DISABLED";
    FD1P3IX ev_bit_i25 (.D(ev_bit[24]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i25.GSR = "DISABLED";
    FD1P3IX ev_bit_i24 (.D(ev_bit[23]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i24.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_167 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[31]), 
         .D(ev_bit[31]), .Z(ev_wr_data[31])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_167.init = 16'hddd0;
    PFUMX i14486 (.BLUT(n23269), .ALUT(n23270), .C0(spi1_miso_N_2513[5]), 
          .Z(n23271));
    LUT4 i1_3_lut_adj_168 (.A(n14465), .B(init_shadow[71]), .C(ev_bit[71]), 
         .Z(n13145)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_168.init = 16'hecec;
    LUT4 i1_3_lut_adj_169 (.A(n14465), .B(init_shadow[70]), .C(ev_bit[70]), 
         .Z(n13139)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_169.init = 16'hecec;
    CCU2D fpga_time_1258_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22275), .COUT(n22276), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_7.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i58 (.D(n13062), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i58.GSR = "DISABLED";
    FD1P3IX init_shadow_i36 (.D(n12926), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i36.GSR = "DISABLED";
    TSALL TSALL_INST (.TSALL(GND_net));
    LUT4 i1_2_lut_rep_133 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .Z(n23722)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_rep_133.init = 16'heeee;
    L6MUX21 i14515 (.D0(n23296), .D1(n23297), .SD(spi1_miso_N_2513[3]), 
            .Z(n23300));
    L6MUX21 i14516 (.D0(n23298), .D1(n23299), .SD(spi1_miso_N_2513[3]), 
            .Z(n23301));
    L6MUX21 i14546 (.D0(n23327), .D1(n23328), .SD(spi1_miso_N_2513[3]), 
            .Z(n23331));
    L6MUX21 i14547 (.D0(n23329), .D1(n23330), .SD(spi1_miso_N_2513[3]), 
            .Z(n23332));
    LUT4 i1_2_lut_rep_107_3_lut (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(spi_byte_count[8]), .Z(n23696)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_rep_107_3_lut.init = 16'hfefe;
    LUT4 i1_3_lut_4_lut_adj_170 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[32]), 
         .D(ev_bit[32]), .Z(ev_wr_data[32])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_170.init = 16'hddd0;
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    LUT4 i1_3_lut_adj_171 (.A(n14465), .B(init_shadow[69]), .C(ev_bit[69]), 
         .Z(n13133)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_171.init = 16'hecec;
    L6MUX21 i14511 (.D0(n23288), .D1(n23289), .SD(spi1_miso_N_2513[2]), 
            .Z(n23296));
    L6MUX21 i14512 (.D0(n23290), .D1(n23291), .SD(spi1_miso_N_2513[2]), 
            .Z(n23297));
    LUT4 i1_3_lut_adj_172 (.A(n14465), .B(init_shadow[68]), .C(ev_bit[68]), 
         .Z(n13127)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_172.init = 16'hecec;
    LUT4 i1_3_lut_adj_173 (.A(n14465), .B(init_shadow[67]), .C(ev_bit[67]), 
         .Z(n13121)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_173.init = 16'hecec;
    LUT4 i1_3_lut_adj_174 (.A(n14465), .B(init_shadow[66]), .C(ev_bit[66]), 
         .Z(n13115)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_174.init = 16'hecec;
    L6MUX21 i14513 (.D0(n23292), .D1(n23293), .SD(spi1_miso_N_2513[2]), 
            .Z(n23298));
    L6MUX21 i14514 (.D0(n23294), .D1(n23295), .SD(spi1_miso_N_2513[2]), 
            .Z(n23299));
    FD1P3IX ev_bit_i23 (.D(ev_bit[22]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i23.GSR = "DISABLED";
    L6MUX21 i14542 (.D0(n23319), .D1(n23320), .SD(spi1_miso_N_2513[2]), 
            .Z(n23327));
    FD1P3IX init_shadow_i57 (.D(n13056), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i57.GSR = "DISABLED";
    L6MUX21 i14543 (.D0(n23321), .D1(n23322), .SD(spi1_miso_N_2513[2]), 
            .Z(n23328));
    LUT4 i1_3_lut_4_lut_adj_175 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[33]), 
         .D(ev_bit[33]), .Z(ev_wr_data[33])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_175.init = 16'hddd0;
    FD1P3IX ev_bit_i22 (.D(ev_bit[21]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i22.GSR = "DISABLED";
    FD1P3IX spi_channel_field_1255__i1 (.D(n14), .SP(spi1_sck_c_enable_237), 
            .CD(n15590), .CK(spi1_sck_c), .Q(spi_channel_field[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(333[78:102])
    defparam spi_channel_field_1255__i1.GSR = "ENABLED";
    L6MUX21 i14544 (.D0(n23323), .D1(n23324), .SD(spi1_miso_N_2513[2]), 
            .Z(n23329));
    L6MUX21 i14545 (.D0(n23325), .D1(n23326), .SD(spi1_miso_N_2513[2]), 
            .Z(n23330));
    PFUMX i14503 (.BLUT(n23272), .ALUT(n23273), .C0(spi1_miso_N_2513[1]), 
          .Z(n23288));
    LUT4 i1_3_lut_4_lut_adj_176 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[34]), 
         .D(ev_bit[34]), .Z(ev_wr_data[34])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_176.init = 16'hddd0;
    LUT4 i7_4_lut_adj_177 (.A(ev_run_hold_s5[1]), .B(us_tx_c_1), .C(init_shadow[1]), 
         .D(swap_now_s5), .Z(n11408)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_177.init = 16'h5a66;
    LUT4 i7_4_lut_adj_178 (.A(ev_run_hold_s5[2]), .B(us_tx_c_2), .C(init_shadow[2]), 
         .D(swap_now_s5), .Z(n11410)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_178.init = 16'h5a66;
    LUT4 i7_4_lut_adj_179 (.A(ev_run_hold_s5[3]), .B(us_tx_c_3), .C(init_shadow[3]), 
         .D(swap_now_s5), .Z(n11412)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_179.init = 16'h5a66;
    CCU2D fpga_time_1258_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22274), .COUT(n22275), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_5.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_180 (.A(ev_run_hold_s5[4]), .B(us_tx_c_4), .C(init_shadow[4]), 
         .D(swap_now_s5), .Z(n11414)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_180.init = 16'h5a66;
    LUT4 i7_4_lut_adj_181 (.A(ev_run_hold_s5[5]), .B(us_tx_c_5), .C(init_shadow[5]), 
         .D(swap_now_s5), .Z(n11416)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_181.init = 16'h5a66;
    LUT4 i7_4_lut_adj_182 (.A(ev_run_hold_s5[6]), .B(us_tx_c_6), .C(init_shadow[6]), 
         .D(swap_now_s5), .Z(n11418)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_182.init = 16'h5a66;
    LUT4 i1_2_lut_adj_183 (.A(spi_command[1]), .B(spi_version[0]), .Z(n17_adj_3196)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_183.init = 16'h4444;
    LUT4 i7_4_lut_adj_184 (.A(ev_run_hold_s5[7]), .B(us_tx_c_7), .C(init_shadow[7]), 
         .D(swap_now_s5), .Z(n11420)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_184.init = 16'h5a66;
    LUT4 i7_4_lut_adj_185 (.A(ev_run_hold_s5[8]), .B(us_tx_c_8), .C(init_shadow[8]), 
         .D(swap_now_s5), .Z(n11422)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_185.init = 16'h5a66;
    LUT4 i7_4_lut_adj_186 (.A(ev_run_hold_s5[9]), .B(us_tx_c_9), .C(init_shadow[9]), 
         .D(swap_now_s5), .Z(n11424)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_186.init = 16'h5a66;
    LUT4 i7_4_lut_adj_187 (.A(ev_run_hold_s5[10]), .B(us_tx_c_10), .C(init_shadow[10]), 
         .D(swap_now_s5), .Z(n11426)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_187.init = 16'h5a66;
    LUT4 i7_4_lut_adj_188 (.A(ev_run_hold_s5[11]), .B(us_tx_c_11), .C(init_shadow[11]), 
         .D(swap_now_s5), .Z(n11428)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_188.init = 16'h5a66;
    LUT4 i7_4_lut_adj_189 (.A(ev_run_hold_s5[12]), .B(us_tx_c_12), .C(init_shadow[12]), 
         .D(swap_now_s5), .Z(n11430)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_189.init = 16'h5a66;
    LUT4 i7_4_lut_adj_190 (.A(ev_run_hold_s5[13]), .B(us_tx_c_13), .C(init_shadow[13]), 
         .D(swap_now_s5), .Z(n11432)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_190.init = 16'h5a66;
    PFUMX i14504 (.BLUT(n23274), .ALUT(n23275), .C0(spi1_miso_N_2513[1]), 
          .Z(n23289));
    LUT4 i7_4_lut_adj_191 (.A(ev_run_hold_s5[14]), .B(us_tx_c_14), .C(init_shadow[14]), 
         .D(swap_now_s5), .Z(n11434)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_191.init = 16'h5a66;
    LUT4 i14_4_lut (.A(n23214), .B(n23118), .C(spi_rx_shift[0]), .D(n22845), 
         .Z(n30_adj_3192)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i14_4_lut.init = 16'h0100;
    LUT4 i1_3_lut_4_lut_adj_192 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[23]), 
         .D(ev_bit[23]), .Z(ev_wr_data[23])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_192.init = 16'hddd0;
    LUT4 i7_4_lut_adj_193 (.A(ev_run_hold_s5[15]), .B(us_tx_c_15), .C(init_shadow[15]), 
         .D(swap_now_s5), .Z(n11436)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_193.init = 16'h5a66;
    LUT4 i14432_4_lut (.A(spi_rx_shift[5]), .B(n34_adj_3189), .C(spi1_mosi_c_0), 
         .D(spi_extension_length[6]), .Z(n23216)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14432_4_lut.init = 16'hfffe;
    PFUMX i14505 (.BLUT(n23276), .ALUT(n23277), .C0(spi1_miso_N_2513[1]), 
          .Z(n23290));
    LUT4 i7_4_lut_adj_194 (.A(ev_run_hold_s5[16]), .B(us_tx_c_16), .C(init_shadow[16]), 
         .D(swap_now_s5), .Z(n11438)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_194.init = 16'h5a66;
    LUT4 i7_4_lut_adj_195 (.A(ev_run_hold_s5[17]), .B(us_tx_c_17), .C(init_shadow[17]), 
         .D(swap_now_s5), .Z(n11440)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_195.init = 16'h5a66;
    LUT4 i7_4_lut_adj_196 (.A(ev_run_hold_s5[18]), .B(us_tx_c_18), .C(init_shadow[18]), 
         .D(swap_now_s5), .Z(n11442)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_196.init = 16'h5a66;
    PFUMX i14506 (.BLUT(n23278), .ALUT(n23279), .C0(spi1_miso_N_2513[1]), 
          .Z(n23291));
    LUT4 i1_3_lut_4_lut_adj_197 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[35]), 
         .D(ev_bit[35]), .Z(ev_wr_data[35])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_197.init = 16'hddd0;
    LUT4 i7_4_lut_adj_198 (.A(ev_run_hold_s5[19]), .B(us_tx_c_19), .C(init_shadow[19]), 
         .D(swap_now_s5), .Z(n11444)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_198.init = 16'h5a66;
    LUT4 i7_4_lut_adj_199 (.A(ev_run_hold_s5[20]), .B(us_tx_c_20), .C(init_shadow[20]), 
         .D(swap_now_s5), .Z(n11446)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_199.init = 16'h5a66;
    LUT4 i7_4_lut_adj_200 (.A(ev_run_hold_s5[21]), .B(us_tx_c_21), .C(init_shadow[21]), 
         .D(swap_now_s5), .Z(n11448)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_200.init = 16'h5a66;
    LUT4 i7_4_lut_adj_201 (.A(ev_run_hold_s5[22]), .B(us_tx_c_22), .C(init_shadow[22]), 
         .D(swap_now_s5), .Z(n11450)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_201.init = 16'h5a66;
    PFUMX i14507 (.BLUT(n23280), .ALUT(n23281), .C0(spi1_miso_N_2513[1]), 
          .Z(n23292));
    LUT4 i2_3_lut_4_lut_adj_202 (.A(spi_byte_count[4]), .B(n23664), .C(spi_byte_count[2]), 
         .D(n22948), .Z(spi1_sck_c_enable_89)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i2_3_lut_4_lut_adj_202.init = 16'h4000;
    LUT4 i14346_4_lut (.A(spi_command[0]), .B(expected_next_15__N_1417[7]), 
         .C(n23258), .D(n174), .Z(n23130)) /* synthesis lut_function=(A+(B ((D)+!C))) */ ;
    defparam i14346_4_lut.init = 16'heeae;
    PFUMX i14508 (.BLUT(n23282), .ALUT(n23283), .C0(spi1_miso_N_2513[1]), 
          .Z(n23293));
    LUT4 i14430_4_lut (.A(spi_extension_length[7]), .B(spi_rx_shift[3]), 
         .C(spi_rx_shift[4]), .D(spi_rx_shift[6]), .Z(n23214)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14430_4_lut.init = 16'hfffe;
    LUT4 i7_4_lut_adj_203 (.A(ev_run_hold_s5[23]), .B(us_tx_c_23), .C(init_shadow[23]), 
         .D(swap_now_s5), .Z(n11452)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_203.init = 16'h5a66;
    LUT4 i7_4_lut_adj_204 (.A(ev_run_hold_s5[24]), .B(us_tx_c_24), .C(init_shadow[24]), 
         .D(swap_now_s5), .Z(n11454)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_204.init = 16'h5a66;
    LUT4 i7_4_lut_adj_205 (.A(ev_run_hold_s5[25]), .B(us_tx_c_25), .C(init_shadow[25]), 
         .D(swap_now_s5), .Z(n11456)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_205.init = 16'h5a66;
    PFUMX i14509 (.BLUT(n23284), .ALUT(n23285), .C0(spi1_miso_N_2513[1]), 
          .Z(n23294));
    LUT4 i7_4_lut_adj_206 (.A(ev_run_hold_s5[26]), .B(us_tx_c_26), .C(init_shadow[26]), 
         .D(swap_now_s5), .Z(n11458)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_206.init = 16'h5a66;
    LUT4 i7_4_lut_adj_207 (.A(ev_run_hold_s5[27]), .B(us_tx_c_27), .C(init_shadow[27]), 
         .D(swap_now_s5), .Z(n11460)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_207.init = 16'h5a66;
    LUT4 i7_4_lut_adj_208 (.A(ev_run_hold_s5[28]), .B(us_tx_c_28), .C(init_shadow[28]), 
         .D(swap_now_s5), .Z(n11462)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_208.init = 16'h5a66;
    LUT4 i7_4_lut_adj_209 (.A(ev_run_hold_s5[29]), .B(us_tx_c_29), .C(init_shadow[29]), 
         .D(swap_now_s5), .Z(n11464)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_209.init = 16'h5a66;
    LUT4 i7_4_lut_adj_210 (.A(ev_run_hold_s5[30]), .B(us_tx_c_30), .C(init_shadow[30]), 
         .D(swap_now_s5), .Z(n11466)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_210.init = 16'h5a66;
    LUT4 i7_4_lut_adj_211 (.A(ev_run_hold_s5[31]), .B(us_tx_c_31), .C(init_shadow[31]), 
         .D(swap_now_s5), .Z(n11468)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_211.init = 16'h5a66;
    LUT4 i7_4_lut_adj_212 (.A(ev_run_hold_s5[32]), .B(us_tx_c_32), .C(init_shadow[32]), 
         .D(swap_now_s5), .Z(n11470)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_212.init = 16'h5a66;
    LUT4 i7_4_lut_adj_213 (.A(ev_run_hold_s5[33]), .B(us_tx_c_33), .C(init_shadow[33]), 
         .D(swap_now_s5), .Z(n11472)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_213.init = 16'h5a66;
    PFUMX i14510 (.BLUT(n23286), .ALUT(n23287), .C0(spi1_miso_N_2513[1]), 
          .Z(n23295));
    LUT4 i7_4_lut_adj_214 (.A(ev_run_hold_s5[34]), .B(us_tx_c_34), .C(init_shadow[34]), 
         .D(swap_now_s5), .Z(n11474)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_214.init = 16'h5a66;
    LUT4 i7_4_lut_adj_215 (.A(ev_run_hold_s5[35]), .B(us_tx_c_35), .C(init_shadow[35]), 
         .D(swap_now_s5), .Z(n11476)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_215.init = 16'h5a66;
    PFUMX i14534 (.BLUT(n23303), .ALUT(n23304), .C0(spi1_miso_N_2513[1]), 
          .Z(n23319));
    LUT4 i7_4_lut_adj_216 (.A(ev_run_hold_s5[36]), .B(us_tx_c_36), .C(init_shadow[36]), 
         .D(swap_now_s5), .Z(n11478)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_216.init = 16'h5a66;
    LUT4 i7_4_lut_adj_217 (.A(ev_run_hold_s5[37]), .B(us_tx_c_37), .C(init_shadow[37]), 
         .D(swap_now_s5), .Z(n11480)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_217.init = 16'h5a66;
    LUT4 i7_4_lut_adj_218 (.A(ev_run_hold_s5[38]), .B(us_tx_c_38), .C(init_shadow[38]), 
         .D(swap_now_s5), .Z(n11482)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_218.init = 16'h5a66;
    LUT4 i7_4_lut_adj_219 (.A(ev_run_hold_s5[39]), .B(us_tx_c_39), .C(init_shadow[39]), 
         .D(swap_now_s5), .Z(n11484)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_219.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i7 (.D(ev_clear_addr_7__N_2237[7]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i7.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i6 (.D(ev_clear_addr_7__N_2237[6]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i6.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_220 (.A(ev_run_hold_s5[40]), .B(us_tx_c_40), .C(init_shadow[40]), 
         .D(swap_now_s5), .Z(n11486)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_220.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i5 (.D(ev_clear_addr_7__N_2237[5]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i5.GSR = "DISABLED";
    PFUMX i14535 (.BLUT(n23305), .ALUT(n23306), .C0(spi1_miso_N_2513[1]), 
          .Z(n23320));
    LUT4 i7_4_lut_adj_221 (.A(ev_run_hold_s5[41]), .B(us_tx_c_41), .C(init_shadow[41]), 
         .D(swap_now_s5), .Z(n11488)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_221.init = 16'h5a66;
    LUT4 i7_4_lut_adj_222 (.A(ev_run_hold_s5[42]), .B(us_tx_c_42), .C(init_shadow[42]), 
         .D(swap_now_s5), .Z(n11490)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_222.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i4 (.D(ev_clear_addr_7__N_2237[4]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i4.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_223 (.A(ev_run_hold_s5[43]), .B(us_tx_c_43), .C(init_shadow[43]), 
         .D(swap_now_s5), .Z(n11492)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_223.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i3 (.D(ev_clear_addr_7__N_2237[3]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i3.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_224 (.A(ev_run_hold_s5[44]), .B(us_tx_c_44), .C(init_shadow[44]), 
         .D(swap_now_s5), .Z(n11494)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_224.init = 16'h5a66;
    LUT4 i2_3_lut_4_lut_adj_225 (.A(spi_byte_count[4]), .B(n23664), .C(n76), 
         .D(n70), .Z(spi1_sck_c_enable_82)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i2_3_lut_4_lut_adj_225.init = 16'h4000;
    FD1P3IX ev_clear_addr_i2 (.D(ev_clear_addr_7__N_2237[2]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i2.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_226 (.A(ev_run_hold_s5[45]), .B(us_tx_c_45), .C(init_shadow[45]), 
         .D(swap_now_s5), .Z(n11496)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_226.init = 16'h5a66;
    FD1P3IX ev_clear_addr_i1 (.D(ev_clear_addr_7__N_2237[1]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i1.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_227 (.A(n14465), .B(init_shadow[65]), .C(ev_bit[65]), 
         .Z(n13109)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_227.init = 16'hecec;
    FD1P3IX init_shadow_i83 (.D(n13225), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i83.GSR = "DISABLED";
    FD1P3IX init_shadow_i82 (.D(n13219), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i82.GSR = "DISABLED";
    PFUMX i14536 (.BLUT(n23307), .ALUT(n23308), .C0(spi1_miso_N_2513[1]), 
          .Z(n23321));
    LUT4 i1_3_lut_adj_228 (.A(n14465), .B(init_shadow[64]), .C(ev_bit[64]), 
         .Z(n13098)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_228.init = 16'hecec;
    FD1P3IX init_shadow_i81 (.D(n13213), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i81.GSR = "DISABLED";
    FD1P3IX init_shadow_i80 (.D(n13207), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i80.GSR = "DISABLED";
    FD1P3IX init_shadow_i79 (.D(n13199), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i79.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_229 (.A(n14465), .B(init_shadow[63]), .C(ev_bit[63]), 
         .Z(n13092)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_229.init = 16'hecec;
    LUT4 i1_3_lut_adj_230 (.A(n14465), .B(init_shadow[62]), .C(ev_bit[62]), 
         .Z(n13086)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_230.init = 16'hecec;
    FD1P3IX init_shadow_i78 (.D(n13193), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i78.GSR = "DISABLED";
    FD1P3IX init_shadow_i77 (.D(n13187), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i77.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_231 (.A(n14465), .B(init_shadow[61]), .C(ev_bit[61]), 
         .Z(n13080)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_231.init = 16'hecec;
    FD1P3IX init_shadow_i76 (.D(n13181), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i76.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_232 (.A(ev_run_hold_s5[46]), .B(us_tx_c_46), .C(init_shadow[46]), 
         .D(swap_now_s5), .Z(n11498)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_232.init = 16'h5a66;
    LUT4 i14334_2_lut (.A(spi_rx_shift[1]), .B(spi_rx_shift[2]), .Z(n23118)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14334_2_lut.init = 16'heeee;
    FD1P3IX init_shadow_i75 (.D(n13175), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i75.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_233 (.A(spi_extension_length[5]), .B(spi_extension_length[3]), 
         .C(n6_adj_3176), .D(spi_extension_length[4]), .Z(n34_adj_3189)) /* synthesis lut_function=(A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[51:71])
    defparam i1_4_lut_adj_233.init = 16'haaa8;
    LUT4 i1_3_lut_adj_234 (.A(n14465), .B(init_shadow[35]), .C(ev_bit[35]), 
         .Z(n12920)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_234.init = 16'hecec;
    PFUMX i14537 (.BLUT(n23309), .ALUT(n23310), .C0(spi1_miso_N_2513[1]), 
          .Z(n23322));
    FD1P3IX init_shadow_i74 (.D(n13163), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i74.GSR = "DISABLED";
    FD1P3IX init_shadow_i73 (.D(n13157), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i73.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_235 (.A(n14465), .B(init_shadow[37]), .C(ev_bit[37]), 
         .Z(n12932)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_235.init = 16'hecec;
    FD1P3IX init_shadow_i72 (.D(n13151), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i72.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_236 (.A(n14465), .B(init_shadow[56]), .C(ev_bit[56]), 
         .Z(n13050)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_236.init = 16'hecec;
    FD1P3IX init_shadow_i71 (.D(n13145), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i71.GSR = "DISABLED";
    FD1P3IX init_shadow_i70 (.D(n13139), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i70.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_237 (.A(n14465), .B(init_shadow[48]), .C(ev_bit[48]), 
         .Z(n13002)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_237.init = 16'hecec;
    LUT4 i1_3_lut_adj_238 (.A(n14465), .B(init_shadow[47]), .C(ev_bit[47]), 
         .Z(n12994)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_238.init = 16'hecec;
    FD1P3IX init_shadow_i69 (.D(n13133), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i69.GSR = "DISABLED";
    FD1P3IX init_shadow_i68 (.D(n13127), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i68.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_239 (.A(n14465), .B(init_shadow[46]), .C(ev_bit[46]), 
         .Z(n12986)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_239.init = 16'hecec;
    FD1P3IX init_shadow_i67 (.D(n13121), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i67.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_240 (.A(n14465), .B(init_shadow[45]), .C(ev_bit[45]), 
         .Z(n12980)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_240.init = 16'hecec;
    LUT4 i1_3_lut_adj_241 (.A(n14465), .B(init_shadow[44]), .C(ev_bit[44]), 
         .Z(n12974)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_241.init = 16'hecec;
    FD1P3IX init_shadow_i66 (.D(n13115), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i66.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_242 (.A(ev_run_hold_s5[47]), .B(us_tx_c_47), .C(init_shadow[47]), 
         .D(swap_now_s5), .Z(n11500)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_242.init = 16'h5a66;
    LUT4 i1_3_lut_adj_243 (.A(n14465), .B(init_shadow[43]), .C(ev_bit[43]), 
         .Z(n12968)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_243.init = 16'hecec;
    LUT4 i1_3_lut_adj_244 (.A(n14465), .B(init_shadow[55]), .C(ev_bit[55]), 
         .Z(n13044)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_244.init = 16'hecec;
    FD1P3IX init_shadow_i65 (.D(n13109), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i65.GSR = "DISABLED";
    FD1P3IX init_shadow_i64 (.D(n13098), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i64.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_245 (.A(n14465), .B(init_shadow[54]), .C(ev_bit[54]), 
         .Z(n13038)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_245.init = 16'hecec;
    FD1P3IX init_shadow_i63 (.D(n13092), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i63.GSR = "DISABLED";
    FD1P3IX init_shadow_i62 (.D(n13086), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i62.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_246 (.A(n14465), .B(init_shadow[34]), .C(ev_bit[34]), 
         .Z(n12914)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_246.init = 16'hecec;
    LUT4 i1_3_lut_adj_247 (.A(n14465), .B(init_shadow[33]), .C(ev_bit[33]), 
         .Z(n12908)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_247.init = 16'hecec;
    FD1P3IX init_shadow_i61 (.D(n13080), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i61.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_248 (.A(n14465), .B(init_shadow[32]), .C(ev_bit[32]), 
         .Z(n12902)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_248.init = 16'hecec;
    LUT4 i1_3_lut_adj_249 (.A(n14465), .B(init_shadow[31]), .C(ev_bit[31]), 
         .Z(n12896)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_249.init = 16'hecec;
    LUT4 i1_3_lut_adj_250 (.A(n14465), .B(init_shadow[42]), .C(ev_bit[42]), 
         .Z(n12962)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_250.init = 16'hecec;
    PFUMX i14538 (.BLUT(n23311), .ALUT(n23312), .C0(spi1_miso_N_2513[1]), 
          .Z(n23323));
    FD1P3IX init_shadow_i35 (.D(n12920), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i35.GSR = "DISABLED";
    PFUMX i14697 (.BLUT(n23587), .ALUT(n23586), .C0(ev_state[3]), .Z(n23588));
    LUT4 i1_3_lut_adj_251 (.A(n14465), .B(init_shadow[41]), .C(ev_bit[41]), 
         .Z(n12956)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_251.init = 16'hecec;
    LUT4 i1_3_lut_adj_252 (.A(n14465), .B(init_shadow[30]), .C(ev_bit[30]), 
         .Z(n12890)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_252.init = 16'hecec;
    LUT4 i1_3_lut_adj_253 (.A(n14465), .B(init_shadow[53]), .C(ev_bit[53]), 
         .Z(n13032)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_253.init = 16'hecec;
    LUT4 i1_3_lut_adj_254 (.A(n14465), .B(init_shadow[52]), .C(ev_bit[52]), 
         .Z(n13026)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_254.init = 16'hecec;
    LUT4 i1_3_lut_adj_255 (.A(n14465), .B(init_shadow[51]), .C(ev_bit[51]), 
         .Z(n13020)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_255.init = 16'hecec;
    LUT4 i1_3_lut_adj_256 (.A(n14465), .B(init_shadow[0]), .C(ev_bit[0]), 
         .Z(n11385)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_256.init = 16'hecec;
    CCU2D fpga_time_1258_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22273), .COUT(n22274), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_3.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_257 (.A(ws2812_toggle_spi), .B(n14025), .Z(ws2812_toggle_spi_N_2556)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_257.init = 16'h9999;
    LUT4 i1_2_lut_adj_258 (.A(spi_byte_count[0]), .B(n22840), .Z(spi1_sck_c_enable_104)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_258.init = 16'h8888;
    PFUMX i14539 (.BLUT(n23313), .ALUT(n23314), .C0(spi1_miso_N_2513[1]), 
          .Z(n23324));
    LUT4 i1_3_lut_adj_259 (.A(spi_byte_count[1]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[0]), .Z(n22948)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_3_lut_adj_259.init = 16'h0202;
    CCU2D fpga_time_1258_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22273), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_1258_add_4_1.INIT1 = 16'h0555;
    defparam fpga_time_1258_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_260 (.A(ev_run_hold_s5[48]), .B(us_tx_c_48), .C(init_shadow[48]), 
         .D(swap_now_s5), .Z(n11502)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_260.init = 16'h5a66;
    CCU2D expected_next_15__I_0_647_7 (.A0(expected_next_15__N_1417[7]), .B0(spi_extension_length[7]), 
          .C0(GND_net), .D0(GND_net), .A1(spi1_mosi_c_0), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22256), .COUT(n22257), .S0(expected_next[7]), 
          .S1(expected_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_7.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_647_7.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_647_7.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_7.INJECT1_1 = "NO";
    LUT4 i13467_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[34:54])
    defparam i13467_2_lut.init = 16'h6666;
    LUT4 i13474_3_lut (.A(spi_bit_count[2]), .B(spi_bit_count[1]), .C(spi_bit_count[0]), 
         .Z(n18_adj_3179)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[34:54])
    defparam i13474_3_lut.init = 16'h6a6a;
    LUT4 i13489_2_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n29)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam i13489_2_lut.init = 16'h6666;
    LUT4 i14474_4_lut (.A(n23182), .B(n23248), .C(n23228), .D(n23180), 
         .Z(n23258)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14474_4_lut.init = 16'h8000;
    LUT4 i35_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2521)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i35_1_lut.init = 16'h5555;
    LUT4 i1_2_lut_adj_261 (.A(spi_channel_index[0]), .B(spi_channel_index[3]), 
         .Z(n8)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(328[43:69])
    defparam i1_2_lut_adj_261.init = 16'hdddd;
    LUT4 i2_4_lut_adj_262 (.A(spi_channel_index[1]), .B(spi_channel_index[5]), 
         .C(spi_channel_index[4]), .D(spi_channel_index[2]), .Z(n22867)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(328[43:69])
    defparam i2_4_lut_adj_262.init = 16'hffdf;
    LUT4 i1_3_lut_4_lut_adj_263 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[36]), 
         .D(ev_bit[36]), .Z(ev_wr_data[36])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_263.init = 16'hddd0;
    LUT4 i7_4_lut_adj_264 (.A(ev_run_hold_s5[49]), .B(us_tx_c_49), .C(init_shadow[49]), 
         .D(swap_now_s5), .Z(n11504)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_264.init = 16'h5a66;
    LUT4 i1_3_lut_adj_265 (.A(n14465), .B(init_shadow[60]), .C(ev_bit[60]), 
         .Z(n13074)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_265.init = 16'hecec;
    LUT4 i86_4_lut (.A(n23166), .B(n23260), .C(n166), .D(n23164), .Z(n174)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;
    defparam i86_4_lut.init = 16'hf7ff;
    LUT4 i1_3_lut_adj_266 (.A(n14465), .B(init_shadow[40]), .C(ev_bit[40]), 
         .Z(n12950)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_266.init = 16'hecec;
    LUT4 i1_3_lut_adj_267 (.A(n14465), .B(init_shadow[39]), .C(ev_bit[39]), 
         .Z(n12944)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_267.init = 16'hecec;
    LUT4 i14382_4_lut (.A(spi_bitmap[64]), .B(spi_bitmap[25]), .C(spi_bitmap[19]), 
         .D(spi_bitmap[30]), .Z(n23166)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14382_4_lut.init = 16'h8000;
    LUT4 i14476_4_lut (.A(n23198), .B(n23252), .C(n23236), .D(n23196), 
         .Z(n23260)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14476_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_adj_268 (.A(n14465), .B(init_shadow[38]), .C(ev_bit[38]), 
         .Z(n12938)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_268.init = 16'hecec;
    LUT4 i7_4_lut_adj_269 (.A(ev_run_hold_s5[50]), .B(us_tx_c_50), .C(init_shadow[50]), 
         .D(swap_now_s5), .Z(n11506)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_269.init = 16'h5a66;
    LUT4 i78_4_lut (.A(n133), .B(n23240), .C(n91), .D(n23084), .Z(n166)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;
    defparam i78_4_lut.init = 16'hfbff;
    LUT4 i1_3_lut_4_lut_adj_270 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[37]), 
         .D(ev_bit[37]), .Z(ev_wr_data[37])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_270.init = 16'hddd0;
    CCU2D mic_divider_1261_add_4_7 (.A0(mic_divider[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22295), .S0(n35_adj_3187), .S1(n34_adj_3188));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_divider_1261_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_divider_1261_add_4_7.INJECT1_0 = "NO";
    defparam mic_divider_1261_add_4_7.INJECT1_1 = "NO";
    CCU2D status_bit_index_1253_add_4_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22271), .S0(n35), .S1(n34));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253_add_4_7.INIT0 = 16'hfaaa;
    defparam status_bit_index_1253_add_4_7.INIT1 = 16'hfaaa;
    defparam status_bit_index_1253_add_4_7.INJECT1_0 = "NO";
    defparam status_bit_index_1253_add_4_7.INJECT1_1 = "NO";
    LUT4 i14380_4_lut (.A(spi_bitmap[47]), .B(spi_bitmap[66]), .C(spi_bitmap[71]), 
         .D(spi_bitmap[17]), .Z(n23164)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14380_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_271 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[38]), 
         .D(ev_bit[38]), .Z(ev_wr_data[38])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_271.init = 16'hddd0;
    CCU2D mic_divider_1261_add_4_5 (.A0(mic_divider[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22294), .COUT(n22295), .S0(n37_adj_3185), 
          .S1(n36_adj_3186));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_divider_1261_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_divider_1261_add_4_5.INJECT1_0 = "NO";
    defparam mic_divider_1261_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_272 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[39]), 
         .D(ev_bit[39]), .Z(ev_wr_data[39])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_272.init = 16'hddd0;
    LUT4 i9818_2_lut (.A(spi_byte_count[15]), .B(spi_byte_count[14]), .Z(n18609)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i9818_2_lut.init = 16'heeee;
    LUT4 i3_4_lut_adj_273 (.A(n18593), .B(n23956), .C(phase_step_s3), 
         .D(phase_step_s2), .Z(n22343)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_273.init = 16'hfffe;
    LUT4 i7_4_lut_adj_274 (.A(ev_run_hold_s5[51]), .B(us_tx_c_51), .C(init_shadow[51]), 
         .D(swap_now_s5), .Z(n11508)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_274.init = 16'h5a66;
    LUT4 i14414_4_lut (.A(spi_bitmap[15]), .B(spi_bitmap[36]), .C(spi_bitmap[29]), 
         .D(spi_bitmap[39]), .Z(n23198)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14414_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_275 (.A(ev_run_hold_s5[52]), .B(us_tx_c_52), .C(init_shadow[52]), 
         .D(swap_now_s5), .Z(n11510)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_275.init = 16'h5a66;
    FD1S3AX phase_step_s4_497_rep_134 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_610)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s4_497_rep_134.GSR = "DISABLED";
    LUT4 run_addr_s3_8__I_0_i1_3_lut (.A(run_addr_s3[0]), .B(ev_rd_slot[0]), 
         .C(n22343), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i1_3_lut.init = 16'hacac;
    LUT4 i7_4_lut_adj_276 (.A(ev_run_hold_s5[53]), .B(us_tx_c_53), .C(init_shadow[53]), 
         .D(swap_now_s5), .Z(n11512)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_276.init = 16'h5a66;
    LUT4 i14468_4_lut (.A(n23058), .B(n23232), .C(n23188), .D(n23056), 
         .Z(n23252)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14468_4_lut.init = 16'h8000;
    LUT4 i14452_4_lut (.A(spi_bitmap[79]), .B(n23192), .C(n23064), .D(spi_bitmap[4]), 
         .Z(n23236)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14452_4_lut.init = 16'h8000;
    LUT4 i3_4_lut_adj_277 (.A(ev_ch[3]), .B(ev_ch[5]), .C(ev_ch[2]), .D(n23208), 
         .Z(n20258)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i3_4_lut_adj_277.init = 16'hfeff;
    LUT4 i14412_4_lut (.A(spi_bitmap[58]), .B(spi_bitmap[69]), .C(spi_bitmap[63]), 
         .D(spi_bitmap[78]), .Z(n23196)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14412_4_lut.init = 16'h8000;
    LUT4 i45_4_lut (.A(spi_bitmap[14]), .B(spi_bitmap[87]), .C(spi_bitmap[85]), 
         .D(spi_bitmap[84]), .Z(n133)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;
    defparam i45_4_lut.init = 16'hfffd;
    LUT4 i1_3_lut_4_lut_adj_278 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[40]), 
         .D(ev_bit[40]), .Z(ev_wr_data[40])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_278.init = 16'hddd0;
    LUT4 i7_4_lut_adj_279 (.A(ev_run_hold_s5[54]), .B(us_tx_c_54), .C(init_shadow[54]), 
         .D(swap_now_s5), .Z(n11514)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_279.init = 16'h5a66;
    LUT4 i7_4_lut_adj_280 (.A(ev_run_hold_s5[55]), .B(us_tx_c_55), .C(init_shadow[55]), 
         .D(swap_now_s5), .Z(n11516)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_280.init = 16'h5a66;
    LUT4 i7_4_lut_adj_281 (.A(ev_run_hold_s5[56]), .B(us_tx_c_56), .C(init_shadow[56]), 
         .D(swap_now_s5), .Z(n11518)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_281.init = 16'h5a66;
    LUT4 i7_4_lut_adj_282 (.A(ev_run_hold_s5[57]), .B(us_tx_c_57), .C(init_shadow[57]), 
         .D(swap_now_s5), .Z(n11520)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_282.init = 16'h5a66;
    LUT4 i7_4_lut_adj_283 (.A(ev_run_hold_s5[58]), .B(us_tx_c_58), .C(init_shadow[58]), 
         .D(swap_now_s5), .Z(n11522)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_283.init = 16'h5a66;
    CCU2D mic_divider_1261_add_4_3 (.A0(mic_divider[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22293), .COUT(n22294), .S0(n39_adj_3183), 
          .S1(n38_adj_3184));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_divider_1261_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_divider_1261_add_4_3.INJECT1_0 = "NO";
    defparam mic_divider_1261_add_4_3.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_284 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[41]), 
         .D(ev_bit[41]), .Z(ev_wr_data[41])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_284.init = 16'hddd0;
    LUT4 i7_4_lut_adj_285 (.A(ev_run_hold_s5[59]), .B(us_tx_c_59), .C(init_shadow[59]), 
         .D(swap_now_s5), .Z(n11524)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_285.init = 16'h5a66;
    LUT4 i14456_4_lut (.A(spi_bitmap[82]), .B(n23200), .C(n23080), .D(spi_bitmap[7]), 
         .Z(n23240)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14456_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_286 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[42]), 
         .D(ev_bit[42]), .Z(ev_wr_data[42])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_286.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_287 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[43]), 
         .D(ev_bit[43]), .Z(ev_wr_data[43])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_287.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_288 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[44]), 
         .D(ev_bit[44]), .Z(ev_wr_data[44])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_288.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_289 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[45]), 
         .D(ev_bit[45]), .Z(ev_wr_data[45])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_289.init = 16'hddd0;
    LUT4 i7_4_lut_adj_290 (.A(ev_run_hold_s5[60]), .B(us_tx_c_60), .C(init_shadow[60]), 
         .D(swap_now_s5), .Z(n11526)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_290.init = 16'h5a66;
    LUT4 i14424_4_lut (.A(ev_ch[0]), .B(ev_ch[1]), .C(ev_ch[4]), .D(ev_ch[6]), 
         .Z(n23208)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14424_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_291 (.A(ev_run_hold_s5[61]), .B(us_tx_c_61), .C(init_shadow[61]), 
         .D(swap_now_s5), .Z(n11528)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_291.init = 16'h5a66;
    LUT4 i7_4_lut_adj_292 (.A(ev_run_hold_s5[62]), .B(us_tx_c_62), .C(init_shadow[62]), 
         .D(swap_now_s5), .Z(n11530)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_292.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_293 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[46]), 
         .D(ev_bit[46]), .Z(ev_wr_data[46])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_293.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_294 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[47]), 
         .D(ev_bit[47]), .Z(ev_wr_data[47])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_294.init = 16'hddd0;
    LUT4 i1_4_lut_4_lut_adj_295 (.A(n23673), .B(n23722), .C(n22925), .D(n16216), 
         .Z(n22876)) /* synthesis lut_function=(!(A+(B (C)+!B (C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_4_lut_4_lut_adj_295.init = 16'h0504;
    LUT4 i7_4_lut_adj_296 (.A(ev_run_hold_s5[63]), .B(us_tx_c_63), .C(init_shadow[63]), 
         .D(swap_now_s5), .Z(n11532)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_296.init = 16'h5a66;
    LUT4 i2_3_lut_rep_76_4_lut (.A(n23673), .B(n23722), .C(spi_byte_count[0]), 
         .D(n22960), .Z(n23665)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i2_3_lut_rep_76_4_lut.init = 16'hffef;
    CCU2D mic_divider_1261_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22293), .S1(n40_adj_3182));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261_add_4_1.INIT0 = 16'hF000;
    defparam mic_divider_1261_add_4_1.INIT1 = 16'h0555;
    defparam mic_divider_1261_add_4_1.INJECT1_0 = "NO";
    defparam mic_divider_1261_add_4_1.INJECT1_1 = "NO";
    CCU2D status_bit_index_1253_add_4_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22270), .COUT(n22271), .S0(n37), 
          .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253_add_4_5.INIT0 = 16'hfaaa;
    defparam status_bit_index_1253_add_4_5.INIT1 = 16'hfaaa;
    defparam status_bit_index_1253_add_4_5.INJECT1_0 = "NO";
    defparam status_bit_index_1253_add_4_5.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_647_5 (.A0(expected_next_15__N_1417[7]), .B0(spi_extension_length[5]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1417[7]), 
          .B1(spi_extension_length[6]), .C1(GND_net), .D1(GND_net), .CIN(n22255), 
          .COUT(n22256), .S0(expected_next[5]), .S1(expected_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_5.INIT0 = 16'ha999;
    defparam expected_next_15__I_0_647_5.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_647_5.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_5.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_647_3 (.A0(expected_next_15__N_1417[7]), .B0(spi_extension_length[3]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1458[3]), 
          .B1(spi_extension_length[4]), .C1(GND_net), .D1(GND_net), .CIN(n22254), 
          .COUT(n22255), .S0(expected_next[3]), .S1(expected_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_3.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_647_3.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_647_3.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_3.INJECT1_1 = "NO";
    CCU2D status_bit_index_1253_add_4_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22269), .COUT(n22270), .S0(n39), 
          .S1(n38));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253_add_4_3.INIT0 = 16'hfaaa;
    defparam status_bit_index_1253_add_4_3.INIT1 = 16'hfaaa;
    defparam status_bit_index_1253_add_4_3.INJECT1_0 = "NO";
    defparam status_bit_index_1253_add_4_3.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_647_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(expected_next_15__N_1458[3]), .B1(spi_extension_length[2]), 
          .C1(GND_net), .D1(GND_net), .COUT(n22254), .S1(expected_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_1.INIT0 = 16'hF000;
    defparam expected_next_15__I_0_647_1.INIT1 = 16'ha999;
    defparam expected_next_15__I_0_647_1.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_1.INJECT1_1 = "NO";
    CCU2D global_phase_s2_1257_add_4_9 (.A0(global_phase_s2[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22292), .S0(n38_adj_3166));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257_add_4_9.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1257_add_4_9.INIT1 = 16'h0000;
    defparam global_phase_s2_1257_add_4_9.INJECT1_0 = "NO";
    defparam global_phase_s2_1257_add_4_9.INJECT1_1 = "NO";
    LUT4 i23_4_lut (.A(spi_byte_count[7]), .B(n44_adj_3194), .C(spi_byte_count[6]), 
         .D(n16216), .Z(n15_adj_3181)) /* synthesis lut_function=(!(A (B (C))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:31])
    defparam i23_4_lut.init = 16'h7f7a;
    CCU2D global_phase_s2_1257_add_4_7 (.A0(global_phase_s2[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22291), .COUT(n22292), .S0(n40_adj_3168), 
          .S1(n39_adj_3167));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257_add_4_7.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1257_add_4_7.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1257_add_4_7.INJECT1_0 = "NO";
    defparam global_phase_s2_1257_add_4_7.INJECT1_1 = "NO";
    CCU2D status_bit_index_1253_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22269), .S1(n40));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[66:89])
    defparam status_bit_index_1253_add_4_1.INIT0 = 16'hF000;
    defparam status_bit_index_1253_add_4_1.INIT1 = 16'h0555;
    defparam status_bit_index_1253_add_4_1.INJECT1_0 = "NO";
    defparam status_bit_index_1253_add_4_1.INJECT1_1 = "NO";
    CCU2D add_650_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22239), .COUT(n22240), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_19.INIT0 = 16'h5aaa;
    defparam add_650_19.INIT1 = 16'h5555;
    defparam add_650_19.INJECT1_0 = "NO";
    defparam add_650_19.INJECT1_1 = "NO";
    CCU2D add_650_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22238), .COUT(n22239), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_17.INIT0 = 16'h5aaa;
    defparam add_650_17.INIT1 = 16'h5aaa;
    defparam add_650_17.INJECT1_0 = "NO";
    defparam add_650_17.INJECT1_1 = "NO";
    CCU2D add_650_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22237), .COUT(n22238), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_15.INIT0 = 16'h5555;
    defparam add_650_15.INIT1 = 16'h5555;
    defparam add_650_15.INJECT1_0 = "NO";
    defparam add_650_15.INJECT1_1 = "NO";
    CCU2D add_650_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22236), .COUT(n22237), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_13.INIT0 = 16'h5555;
    defparam add_650_13.INIT1 = 16'h5555;
    defparam add_650_13.INJECT1_0 = "NO";
    defparam add_650_13.INJECT1_1 = "NO";
    CCU2D time_divider_1259_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22267), .S0(n35_adj_3164), 
          .S1(n34_adj_3165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_1259_add_4_7.INIT1 = 16'hfaaa;
    defparam time_divider_1259_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_1259_add_4_7.INJECT1_1 = "NO";
    CCU2D add_650_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22235), .COUT(n22236), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_11.INIT0 = 16'h5555;
    defparam add_650_11.INIT1 = 16'h5aaa;
    defparam add_650_11.INJECT1_0 = "NO";
    defparam add_650_11.INJECT1_1 = "NO";
    CCU2D add_650_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22234), .COUT(n22235), .S0(phase_frac_sum[7]), .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_9.INIT0 = 16'h5555;
    defparam add_650_9.INIT1 = 16'h5aaa;
    defparam add_650_9.INJECT1_0 = "NO";
    defparam add_650_9.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i21 (.D(ev_bit[20]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i21.GSR = "DISABLED";
    CCU2D add_650_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22233), .COUT(n22234), .S0(phase_frac_sum[5]), .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_7.INIT0 = 16'h5555;
    defparam add_650_7.INIT1 = 16'h5555;
    defparam add_650_7.INJECT1_0 = "NO";
    defparam add_650_7.INJECT1_1 = "NO";
    CCU2D add_650_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22232), .COUT(n22233), .S0(phase_frac_sum[3]), .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_5.INIT0 = 16'h5aaa;
    defparam add_650_5.INIT1 = 16'h5aaa;
    defparam add_650_5.INJECT1_0 = "NO";
    defparam add_650_5.INJECT1_1 = "NO";
    CCU2D global_phase_s2_1257_add_4_5 (.A0(global_phase_s2[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22290), .COUT(n22291), .S0(n42), 
          .S1(n41));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257_add_4_5.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1257_add_4_5.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1257_add_4_5.INJECT1_0 = "NO";
    defparam global_phase_s2_1257_add_4_5.INJECT1_1 = "NO";
    LUT4 i2_4_lut_adj_297 (.A(spi_byte_count[4]), .B(spi_byte_count[2]), 
         .C(spi_byte_count[5]), .D(spi_byte_count[3]), .Z(n44_adj_3194)) /* synthesis lut_function=(A+(B (C+(D))+!B (C))) */ ;
    defparam i2_4_lut_adj_297.init = 16'hfefa;
    LUT4 i3_2_lut (.A(spi_bitmap[86]), .B(spi_bitmap[62]), .Z(n91)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam i3_2_lut.init = 16'hbbbb;
    LUT4 i7_4_lut_adj_298 (.A(ev_run_hold_s5[64]), .B(us_tx_c_64), .C(init_shadow[64]), 
         .D(swap_now_s5), .Z(n11534)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_298.init = 16'h5a66;
    LUT4 run_addr_s3_8__I_0_i2_3_lut (.A(run_addr_s3[1]), .B(ev_rd_slot[1]), 
         .C(n22343), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[33:74])
    defparam run_addr_s3_8__I_0_i2_3_lut.init = 16'hacac;
    LUT4 i14300_2_lut (.A(spi_bitmap[41]), .B(spi_bitmap[72]), .Z(n23084)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14300_2_lut.init = 16'h8888;
    LUT4 i14416_4_lut (.A(spi_bitmap[81]), .B(spi_bitmap[54]), .C(spi_bitmap[40]), 
         .D(spi_bitmap[59]), .Z(n23200)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14416_4_lut.init = 16'h8000;
    LUT4 i14296_2_lut (.A(spi_bitmap[9]), .B(spi_bitmap[55]), .Z(n23080)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14296_2_lut.init = 16'h8888;
    LUT4 i14436_3_lut (.A(spi_byte_count[8]), .B(spi_byte_count[15]), .C(fpga_cs_n_c), 
         .Z(n23220)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i14436_3_lut.init = 16'hfefe;
    FD1P3IX init_shadow_i56 (.D(n13050), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i56.GSR = "DISABLED";
    LUT4 i14398_4_lut (.A(spi_bitmap[56]), .B(spi_bitmap[67]), .C(spi_bitmap[60]), 
         .D(spi_bitmap[6]), .Z(n23182)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14398_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_299 (.A(ev_run_hold_s5[65]), .B(us_tx_c_65), .C(init_shadow[65]), 
         .D(swap_now_s5), .Z(n11536)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_299.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_300 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[12]), 
         .D(ev_bit[12]), .Z(ev_wr_data[12])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_300.init = 16'hddd0;
    CCU2D global_phase_s2_1257_add_4_3 (.A0(global_phase_s2[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase_s2[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22289), .COUT(n22290), .S0(n44), 
          .S1(n43));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257_add_4_3.INIT0 = 16'hfaaa;
    defparam global_phase_s2_1257_add_4_3.INIT1 = 16'hfaaa;
    defparam global_phase_s2_1257_add_4_3.INJECT1_0 = "NO";
    defparam global_phase_s2_1257_add_4_3.INJECT1_1 = "NO";
    CCU2D time_divider_1259_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22266), .COUT(n22267), .S0(n37_adj_3162), 
          .S1(n36_adj_3163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_1259_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_1259_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_1259_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_301 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[13]), 
         .D(ev_bit[13]), .Z(ev_wr_data[13])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_301.init = 16'hddd0;
    CCU2D time_divider_1259_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22265), .COUT(n22266), .S0(n39_adj_3160), 
          .S1(n38_adj_3161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_1259_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_1259_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_1259_add_4_3.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_302 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[14]), 
         .D(ev_bit[14]), .Z(ev_wr_data[14])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_302.init = 16'hddd0;
    CCU2D global_phase_s2_1257_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(global_phase_s2[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22289), .S1(n45));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257_add_4_1.INIT0 = 16'hF000;
    defparam global_phase_s2_1257_add_4_1.INIT1 = 16'h0555;
    defparam global_phase_s2_1257_add_4_1.INJECT1_0 = "NO";
    defparam global_phase_s2_1257_add_4_1.INJECT1_1 = "NO";
    CCU2D fpga_time_1258_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22288), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_1258_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_33.INJECT1_1 = "NO";
    CCU2D time_divider_1259_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22265), .S1(n40_adj_3159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(416[29:48])
    defparam time_divider_1259_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_1259_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_1259_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_1259_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_303 (.A(ev_run_hold_s5[66]), .B(us_tx_c_66), .C(init_shadow[66]), 
         .D(swap_now_s5), .Z(n11538)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_303.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_304 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[15]), 
         .D(ev_bit[15]), .Z(ev_wr_data[15])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_304.init = 16'hddd0;
    CCU2D add_650_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22231), .COUT(n22232), .S0(phase_frac_sum[1]), .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_3.INIT0 = 16'h5aaa;
    defparam add_650_3.INIT1 = 16'h5aaa;
    defparam add_650_3.INJECT1_0 = "NO";
    defparam add_650_3.INJECT1_1 = "NO";
    CCU2D add_164_15 (.A0(spi_byte_count[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22225), .COUT(n22226), .S0(spi_byte_count_15__N_1694[13]), 
          .S1(spi_byte_count_15__N_1694[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_15.INIT0 = 16'h5aaa;
    defparam add_164_15.INIT1 = 16'h5aaa;
    defparam add_164_15.INJECT1_0 = "NO";
    defparam add_164_15.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_305 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[16]), 
         .D(ev_bit[16]), .Z(ev_wr_data[16])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_305.init = 16'hddd0;
    CCU2D add_164_13 (.A0(spi_byte_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22224), .COUT(n22225), .S0(spi_byte_count_15__N_1694[11]), 
          .S1(spi_byte_count_15__N_1694[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_13.INIT0 = 16'h5aaa;
    defparam add_164_13.INIT1 = 16'h5aaa;
    defparam add_164_13.INJECT1_0 = "NO";
    defparam add_164_13.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_306 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[17]), 
         .D(ev_bit[17]), .Z(ev_wr_data[17])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_306.init = 16'hddd0;
    CCU2D add_650_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22231), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_1.INIT0 = 16'hF000;
    defparam add_650_1.INIT1 = 16'h5555;
    defparam add_650_1.INJECT1_0 = "NO";
    defparam add_650_1.INJECT1_1 = "NO";
    LUT4 i1633_1_lut (.A(status_bit_index[5]), .Z(spi1_miso_N_2513[5])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i1633_1_lut.init = 16'h5555;
    CCU2D add_328_9 (.A0(ev_clear_addr[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22230), .S0(ev_clear_addr_7__N_2237[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(495[34:54])
    defparam add_328_9.INIT0 = 16'h5aaa;
    defparam add_328_9.INIT1 = 16'h0000;
    defparam add_328_9.INJECT1_0 = "NO";
    defparam add_328_9.INJECT1_1 = "NO";
    CCU2D add_164_11 (.A0(spi_byte_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22223), .COUT(n22224), .S0(spi_byte_count_15__N_1694[9]), 
          .S1(spi_byte_count_15__N_1694[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_11.INIT0 = 16'h5aaa;
    defparam add_164_11.INIT1 = 16'h5aaa;
    defparam add_164_11.INJECT1_0 = "NO";
    defparam add_164_11.INJECT1_1 = "NO";
    CCU2D add_164_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_byte_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22219), .S1(spi_byte_count_15__N_1694[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_1.INIT0 = 16'hF000;
    defparam add_164_1.INIT1 = 16'h5555;
    defparam add_164_1.INJECT1_0 = "NO";
    defparam add_164_1.INJECT1_1 = "NO";
    CCU2D add_164_9 (.A0(spi_byte_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22222), .COUT(n22223), .S0(spi_byte_count_15__N_1694[7]), 
          .S1(spi_byte_count_15__N_1694[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_9.INIT0 = 16'h5aaa;
    defparam add_164_9.INIT1 = 16'h5aaa;
    defparam add_164_9.INJECT1_0 = "NO";
    defparam add_164_9.INJECT1_1 = "NO";
    CCU2D add_328_7 (.A0(ev_clear_addr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22229), .COUT(n22230), .S0(ev_clear_addr_7__N_2237[5]), 
          .S1(ev_clear_addr_7__N_2237[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(495[34:54])
    defparam add_328_7.INIT0 = 16'h5aaa;
    defparam add_328_7.INIT1 = 16'h5aaa;
    defparam add_328_7.INJECT1_0 = "NO";
    defparam add_328_7.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i6 (.D(ev_bit[5]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i6.GSR = "DISABLED";
    FD1P3IX ev_bit_i20 (.D(ev_bit[19]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i20.GSR = "DISABLED";
    FD1P3IX ev_bit_i5 (.D(ev_bit[4]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i5.GSR = "DISABLED";
    FD1P3IX ev_bit_i19 (.D(ev_bit[18]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i19.GSR = "DISABLED";
    FD1P3IX ev_bit_i4 (.D(ev_bit[3]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i4.GSR = "DISABLED";
    FD1P3IX ev_bit_i18 (.D(ev_bit[17]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i18.GSR = "DISABLED";
    FD1P3IX ev_bit_i3 (.D(ev_bit[2]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i3.GSR = "DISABLED";
    FD1P3IX ev_bit_i17 (.D(ev_bit[16]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i17.GSR = "DISABLED";
    FD1P3IX ev_bit_i2 (.D(ev_bit[1]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i2.GSR = "DISABLED";
    FD1P3IX ev_bit_i16 (.D(ev_bit[15]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i16.GSR = "DISABLED";
    FD1P3IX ev_bit_i1 (.D(ev_bit[0]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i1.GSR = "DISABLED";
    FD1P3IX ev_bit_i15 (.D(ev_bit[14]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i15.GSR = "DISABLED";
    FD1P3IX ev_bit_i53 (.D(ev_bit[52]), .SP(pll_clk_enable_705), .CD(n15506), 
            .CK(pll_clk), .Q(ev_bit[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i53.GSR = "DISABLED";
    FD1P3IX ev_bit_i14 (.D(ev_bit[13]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i14.GSR = "DISABLED";
    CCU2D spi_channel_index_1254_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22263), .S0(n35_adj_3174), 
          .S1(n34_adj_3175));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1254_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1254_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_1254_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_307 (.A(n14465), .B(init_shadow[49]), .C(ev_bit[49]), 
         .Z(n13008)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_307.init = 16'hecec;
    CCU2D add_328_5 (.A0(ev_clear_addr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22228), .COUT(n22229), .S0(ev_clear_addr_7__N_2237[3]), 
          .S1(ev_clear_addr_7__N_2237[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(495[34:54])
    defparam add_328_5.INIT0 = 16'h5aaa;
    defparam add_328_5.INIT1 = 16'h5aaa;
    defparam add_328_5.INJECT1_0 = "NO";
    defparam add_328_5.INJECT1_1 = "NO";
    CCU2D add_164_7 (.A0(spi_byte_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22221), .COUT(n22222), .S0(spi_byte_count_15__N_1694[5]), 
          .S1(spi_byte_count_15__N_1694[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_7.INIT0 = 16'h5aaa;
    defparam add_164_7.INIT1 = 16'h5aaa;
    defparam add_164_7.INJECT1_0 = "NO";
    defparam add_164_7.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_308 (.A(ev_run_hold_s5[67]), .B(us_tx_c_67), .C(init_shadow[67]), 
         .D(swap_now_s5), .Z(n11540)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_308.init = 16'h5a66;
    LUT4 i14464_4_lut (.A(n23020), .B(n23224), .C(n23172), .D(n23018), 
         .Z(n23248)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14464_4_lut.init = 16'h8000;
    LUT4 i14444_4_lut (.A(spi_bitmap[45]), .B(n23176), .C(n23026), .D(spi_bitmap[5]), 
         .Z(n23228)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14444_4_lut.init = 16'h8000;
    LUT4 i2_3_lut_4_lut_adj_309 (.A(spi_byte_count[2]), .B(n23664), .C(spi_byte_count[4]), 
         .D(n22948), .Z(spi1_sck_c_enable_97)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i2_3_lut_4_lut_adj_309.init = 16'h4000;
    LUT4 i14396_4_lut (.A(spi_bitmap[12]), .B(spi_bitmap[24]), .C(spi_bitmap[13]), 
         .D(spi_bitmap[27]), .Z(n23180)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14396_4_lut.init = 16'h8000;
    LUT4 i7845_3_lut (.A(active_bank), .B(wrap_s2), .C(swap_pending), 
         .Z(active_bank_N_910)) /* synthesis lut_function=(!(A (B (C))+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(202[28:40])
    defparam i7845_3_lut.init = 16'h6a6a;
    LUT4 i1_3_lut_4_lut_adj_310 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[24]), 
         .D(ev_bit[24]), .Z(ev_wr_data[24])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_310.init = 16'hddd0;
    LUT4 i14392_4_lut (.A(spi_bitmap[43]), .B(spi_bitmap[70]), .C(spi_bitmap[21]), 
         .D(spi_bitmap[73]), .Z(n23176)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14392_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_311 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[25]), 
         .D(ev_bit[25]), .Z(ev_wr_data[25])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_311.init = 16'hddd0;
    CCU2D add_652_6 (.A0(staging_q[12]), .B0(staging_q[4]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[13]), .B1(staging_q[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n22216), .COUT(n22217), .S0(build_sum_8__N_2053[4]), 
          .S1(build_sum_8__N_2053[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[32:80])
    defparam add_652_6.INIT0 = 16'h5666;
    defparam add_652_6.INIT1 = 16'h5666;
    defparam add_652_6.INJECT1_0 = "NO";
    defparam add_652_6.INJECT1_1 = "NO";
    LUT4 i5_4_lut_adj_312 (.A(n9), .B(n23653), .C(spi_byte_count[4]), 
         .D(n23696), .Z(spi1_sck_c_enable_182)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i5_4_lut_adj_312.init = 16'h0080;
    CCU2D add_652_8 (.A0(staging_q[14]), .B0(staging_q[6]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[15]), .B1(staging_q[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n22217), .COUT(n22218), .S0(build_sum_8__N_2053[6]), 
          .S1(build_sum_8__N_2053[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[32:80])
    defparam add_652_8.INIT0 = 16'h5666;
    defparam add_652_8.INIT1 = 16'h5666;
    defparam add_652_8.INJECT1_0 = "NO";
    defparam add_652_8.INJECT1_1 = "NO";
    CCU2D add_328_3 (.A0(ev_clear_addr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22227), .COUT(n22228), .S0(ev_clear_addr_7__N_2237[1]), 
          .S1(ev_clear_addr_7__N_2237[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(495[34:54])
    defparam add_328_3.INIT0 = 16'h5aaa;
    defparam add_328_3.INIT1 = 16'h5aaa;
    defparam add_328_3.INJECT1_0 = "NO";
    defparam add_328_3.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_313 (.A(ev_run_hold_s5[81]), .B(us_tx_c_81), .C(init_shadow[81]), 
         .D(swap_now_s5), .Z(n11568)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_313.init = 16'h5a66;
    CCU2D add_650_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22242), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[34:66])
    defparam add_650_25.INIT0 = 16'h5aaa;
    defparam add_650_25.INIT1 = 16'h0000;
    defparam add_650_25.INJECT1_0 = "NO";
    defparam add_650_25.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_314 (.A(ev_run_hold_s5[68]), .B(us_tx_c_68), .C(init_shadow[68]), 
         .D(swap_now_s5), .Z(n11542)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_314.init = 16'h5a66;
    CCU2D add_328_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(ev_clear_addr[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22227), .S1(ev_clear_addr_7__N_2237[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(495[34:54])
    defparam add_328_1.INIT0 = 16'hF000;
    defparam add_328_1.INIT1 = 16'h5555;
    defparam add_328_1.INJECT1_0 = "NO";
    defparam add_328_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_315 (.A(ev_run_hold_s5[82]), .B(us_tx_c_82), .C(init_shadow[82]), 
         .D(swap_now_s5), .Z(n11570)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_315.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_316 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[26]), 
         .D(ev_bit[26]), .Z(ev_wr_data[26])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_316.init = 16'hddd0;
    CCU2D add_652_4 (.A0(staging_q[10]), .B0(staging_q[2]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[11]), .B1(staging_q[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n22215), .COUT(n22216), .S0(build_sum_8__N_2053[2]), 
          .S1(build_sum_8__N_2053[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[32:80])
    defparam add_652_4.INIT0 = 16'h5666;
    defparam add_652_4.INIT1 = 16'h5666;
    defparam add_652_4.INJECT1_0 = "NO";
    defparam add_652_4.INJECT1_1 = "NO";
    CCU2D equal_1861_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n22137), 
          .S0(frame_end_N_2611));
    defparam equal_1861_17.INIT0 = 16'hFFFF;
    defparam equal_1861_17.INIT1 = 16'h0000;
    defparam equal_1861_17.INJECT1_0 = "NO";
    defparam equal_1861_17.INJECT1_1 = "NO";
    LUT4 n23734_bdd_2_lut_3_lut (.A(n23733), .B(spi_version[7]), .C(n22966), 
         .Z(n23735)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam n23734_bdd_2_lut_3_lut.init = 16'hfefe;
    CCU2D equal_1861_17_13440 (.A0(spi_expected_length[3]), .B0(spi_byte_count_15__N_1694[3]), 
          .C0(spi_expected_length[2]), .D0(spi_byte_count_15__N_1694[2]), 
          .A1(spi_expected_length[1]), .B1(spi_byte_count_15__N_1694[1]), 
          .C1(spi_expected_length[0]), .D1(spi_byte_count_15__N_1694[0]), 
          .CIN(n22136), .COUT(n22137));
    defparam equal_1861_17_13440.INIT0 = 16'h9009;
    defparam equal_1861_17_13440.INIT1 = 16'h9009;
    defparam equal_1861_17_13440.INJECT1_0 = "YES";
    defparam equal_1861_17_13440.INJECT1_1 = "YES";
    LUT4 i1_3_lut_4_lut_adj_317 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[18]), 
         .D(ev_bit[18]), .Z(ev_wr_data[18])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_317.init = 16'hddd0;
    CCU2D add_652_2 (.A0(staging_q[8]), .B0(staging_q[0]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[9]), .B1(staging_q[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n22215), .S1(build_sum_8__N_2053[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[32:80])
    defparam add_652_2.INIT0 = 16'h7000;
    defparam add_652_2.INIT1 = 16'h5666;
    defparam add_652_2.INJECT1_0 = "NO";
    defparam add_652_2.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_318 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[19]), 
         .D(ev_bit[19]), .Z(ev_wr_data[19])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_318.init = 16'hddd0;
    LUT4 i1_4_lut_4_lut_4_lut (.A(ev_state[0]), .B(ev_state[3]), .C(ev_state[2]), 
         .D(ev_state[1]), .Z(pll_clk_enable_316)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A ((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_4_lut_4_lut_4_lut.init = 16'h0024;
    CCU2D add_652_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n22218), 
          .S0(build_sum_8__N_2053[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(512[32:80])
    defparam add_652_cout.INIT0 = 16'h0000;
    defparam add_652_cout.INIT1 = 16'h0000;
    defparam add_652_cout.INJECT1_0 = "NO";
    defparam add_652_cout.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_319 (.A(n14465), .B(init_shadow[58]), .C(ev_bit[58]), 
         .Z(n13062)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_319.init = 16'hecec;
    LUT4 i14242_2_lut (.A(spi_bitmap[22]), .B(spi_bitmap[44]), .Z(n23026)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14242_2_lut.init = 16'h8888;
    LUT4 spi_byte_count_3__bdd_4_lut_14788 (.A(spi_byte_count[3]), .B(n18609), 
         .C(spi_byte_count[2]), .D(n76), .Z(n23653)) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+!(C)))) */ ;
    defparam spi_byte_count_3__bdd_4_lut_14788.init = 16'h1232;
    LUT4 i763_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_362)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(590[18] 592[12])
    defparam i763_2_lut.init = 16'h4444;
    CCU2D fpga_time_1258_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22287), .COUT(n22288), .S0(n136), .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_31.INJECT1_1 = "NO";
    CCU2D fpga_time_1258_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22286), .COUT(n22287), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_29.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_320 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[27]), 
         .D(ev_bit[27]), .Z(ev_wr_data[27])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_320.init = 16'hddd0;
    FD1P3IX init_shadow_i48 (.D(n13002), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i48.GSR = "DISABLED";
    CCU2D spi_channel_index_1254_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22262), .COUT(n22263), .S0(n37_adj_3172), 
          .S1(n36_adj_3173));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1254_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1254_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_1254_add_4_5.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i47 (.D(n12994), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i47.GSR = "DISABLED";
    FD1P3IX init_shadow_i46 (.D(n12986), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i46.GSR = "DISABLED";
    LUT4 i1_4_lut_else_4_lut (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n23723)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(127[17:33])
    defparam i1_4_lut_else_4_lut.init = 16'h2000;
    FD1P3IX init_shadow_i45 (.D(n12980), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i45.GSR = "DISABLED";
    FD1P3IX init_shadow_i44 (.D(n12974), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i44.GSR = "DISABLED";
    FD1P3IX init_shadow_i43 (.D(n12968), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i43.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_321 (.A(n14465), .B(init_shadow[36]), .C(ev_bit[36]), 
         .Z(n12926)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_321.init = 16'hecec;
    LUT4 i14236_2_lut (.A(spi_bitmap[80]), .B(spi_bitmap[18]), .Z(n23020)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14236_2_lut.init = 16'h8888;
    LUT4 i7_4_lut_adj_322 (.A(ev_run_hold_s5[69]), .B(us_tx_c_69), .C(init_shadow[69]), 
         .D(swap_now_s5), .Z(n11544)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_322.init = 16'h5a66;
    CCU2D equal_1861_15 (.A0(spi_expected_length[7]), .B0(spi_byte_count_15__N_1694[7]), 
          .C0(spi_expected_length[6]), .D0(spi_byte_count_15__N_1694[6]), 
          .A1(spi_expected_length[5]), .B1(spi_byte_count_15__N_1694[5]), 
          .C1(spi_expected_length[4]), .D1(spi_byte_count_15__N_1694[4]), 
          .CIN(n22135), .COUT(n22136));
    defparam equal_1861_15.INIT0 = 16'h9009;
    defparam equal_1861_15.INIT1 = 16'h9009;
    defparam equal_1861_15.INJECT1_0 = "YES";
    defparam equal_1861_15.INJECT1_1 = "YES";
    CCU2D equal_1861_11 (.A0(spi_expected_length[15]), .B0(spi_byte_count_15__N_1694[15]), 
          .C0(spi_expected_length[14]), .D0(spi_byte_count_15__N_1694[14]), 
          .A1(spi_expected_length[13]), .B1(spi_byte_count_15__N_1694[13]), 
          .C1(spi_expected_length[12]), .D1(spi_byte_count_15__N_1694[12]), 
          .CIN(n22133), .COUT(n22134));
    defparam equal_1861_11.INIT0 = 16'h9009;
    defparam equal_1861_11.INIT1 = 16'h9009;
    defparam equal_1861_11.INJECT1_0 = "YES";
    defparam equal_1861_11.INJECT1_1 = "YES";
    LUT4 i1_3_lut_4_lut_adj_323 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[28]), 
         .D(ev_bit[28]), .Z(ev_wr_data[28])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_323.init = 16'hddd0;
    CCU2D equal_1861_13 (.A0(spi_expected_length[11]), .B0(spi_byte_count_15__N_1694[11]), 
          .C0(spi_expected_length[10]), .D0(spi_byte_count_15__N_1694[10]), 
          .A1(spi_expected_length[9]), .B1(spi_byte_count_15__N_1694[9]), 
          .C1(spi_expected_length[8]), .D1(spi_byte_count_15__N_1694[8]), 
          .CIN(n22134), .COUT(n22135));
    defparam equal_1861_13.INIT0 = 16'h9009;
    defparam equal_1861_13.INIT1 = 16'h9009;
    defparam equal_1861_13.INJECT1_0 = "YES";
    defparam equal_1861_13.INJECT1_1 = "YES";
    LUT4 i14440_4_lut (.A(spi_bitmap[57]), .B(n23168), .C(n23010), .D(spi_bitmap[77]), 
         .Z(n23224)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14440_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_324 (.A(ev_run_hold_s5[83]), .B(us_tx_c_83), .C(init_shadow[83]), 
         .D(swap_now_s5), .Z(n11572)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_324.init = 16'h5a66;
    LUT4 i14388_4_lut (.A(spi_bitmap[34]), .B(spi_bitmap[50]), .C(spi_bitmap[35]), 
         .D(spi_bitmap[52]), .Z(n23172)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14388_4_lut.init = 16'h8000;
    CCU2D add_164_17 (.A0(spi_byte_count[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22226), .S0(spi_byte_count_15__N_1694[15]), .S1(frame_end_N_2612[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[35:57])
    defparam add_164_17.INIT0 = 16'h5aaa;
    defparam add_164_17.INIT1 = 16'h0000;
    defparam add_164_17.INJECT1_0 = "NO";
    defparam add_164_17.INJECT1_1 = "NO";
    CCU2D fpga_time_1258_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22285), .COUT(n22286), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_27.INJECT1_1 = "NO";
    LUT4 i14234_2_lut (.A(spi_bitmap[48]), .B(spi_bitmap[28]), .Z(n23018)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14234_2_lut.init = 16'h8888;
    CCU2D spi_channel_index_1254_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22261), .COUT(n22262), .S0(n39_adj_3170), 
          .S1(n38_adj_3171));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1254_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1254_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_1254_add_4_3.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_325 (.A(ev_run_hold_s5[70]), .B(us_tx_c_70), .C(init_shadow[70]), 
         .D(swap_now_s5), .Z(n11546)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_325.init = 16'h5a66;
    LUT4 i15_4_lut_rep_73 (.A(n17_adj_3196), .B(n30_adj_3192), .C(n23216), 
         .D(n23130), .Z(n23662)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i15_4_lut_rep_73.init = 16'h0008;
    LUT4 i7_4_lut_adj_326 (.A(ev_run_hold_s5[0]), .B(us_tx_c_0), .C(init_shadow[0]), 
         .D(swap_now_s5), .Z(n10777)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_326.init = 16'h5a66;
    LUT4 i14274_2_lut (.A(spi_bitmap[65]), .B(spi_bitmap[76]), .Z(n23058)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14274_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_327 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[29]), 
         .D(ev_bit[29]), .Z(ev_wr_data[29])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_327.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_328 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[66]), 
         .D(ev_bit[66]), .Z(ev_wr_data[66])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_328.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_329 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[67]), 
         .D(ev_bit[67]), .Z(ev_wr_data[67])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_329.init = 16'hddd0;
    LUT4 i1_3_lut_3_lut_then_4_lut (.A(ev_state[0]), .B(ev_state[2]), .C(ev_state[1]), 
         .D(n7), .Z(n23727)) /* synthesis lut_function=(!(A+(B+(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_3_lut_then_4_lut.init = 16'h0111;
    LUT4 i1_3_lut_3_lut_else_4_lut (.A(ev_state[0]), .B(n23719), .C(ev_state[2]), 
         .D(ev_state[1]), .Z(n23726)) /* synthesis lut_function=(!(A+!(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_3_lut_else_4_lut.init = 16'h5554;
    LUT4 i1_3_lut_4_lut_adj_330 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[68]), 
         .D(ev_bit[68]), .Z(ev_wr_data[68])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_330.init = 16'hddd0;
    CCU2D fpga_time_1258_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22284), .COUT(n22285), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_25.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_331 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[69]), 
         .D(ev_bit[69]), .Z(ev_wr_data[69])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_331.init = 16'hddd0;
    LUT4 i5_4_lut_rep_137 (.A(n9), .B(n23653), .C(spi_byte_count[4]), 
         .D(n23696), .Z(spi1_sck_c_enable_239)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i5_4_lut_rep_137.init = 16'h0080;
    LUT4 i7_4_lut_adj_332 (.A(ev_run_hold_s5[71]), .B(us_tx_c_71), .C(init_shadow[71]), 
         .D(swap_now_s5), .Z(n11548)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_332.init = 16'h5a66;
    LUT4 i14448_4_lut (.A(spi_bitmap[20]), .B(n23184), .C(n23048), .D(spi_bitmap[26]), 
         .Z(n23232)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14448_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_333 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[70]), 
         .D(ev_bit[70]), .Z(ev_wr_data[70])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_333.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_334 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[71]), 
         .D(ev_bit[71]), .Z(ev_wr_data[71])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_334.init = 16'hddd0;
    LUT4 i1_4_lut_adj_335 (.A(spi_command[5]), .B(spi_command[7]), .C(n6_adj_3191), 
         .D(spi_command[2]), .Z(n22966)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[58:78])
    defparam i1_4_lut_adj_335.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_336 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[72]), 
         .D(ev_bit[72]), .Z(ev_wr_data[72])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_336.init = 16'hddd0;
    LUT4 i14404_4_lut (.A(spi_bitmap[42]), .B(spi_bitmap[83]), .C(spi_bitmap[75]), 
         .D(spi_bitmap[10]), .Z(n23188)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14404_4_lut.init = 16'h8000;
    LUT4 i14272_2_lut (.A(spi_bitmap[32]), .B(spi_bitmap[37]), .Z(n23056)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14272_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_337 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[73]), 
         .D(ev_bit[73]), .Z(ev_wr_data[73])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_337.init = 16'hddd0;
    LUT4 i14400_4_lut (.A(spi_bitmap[53]), .B(spi_bitmap[74]), .C(spi_bitmap[61]), 
         .D(spi_bitmap[38]), .Z(n23184)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14400_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_338 (.A(ev_run_hold_s5[72]), .B(us_tx_c_72), .C(init_shadow[72]), 
         .D(swap_now_s5), .Z(n11550)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_338.init = 16'h5a66;
    LUT4 i1_4_lut_rep_139 (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n23694), .Z(pll_clk_enable_743)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i1_4_lut_rep_139.init = 16'h5150;
    LUT4 i7_4_lut_adj_339 (.A(ev_run_hold_s5[73]), .B(us_tx_c_73), .C(init_shadow[73]), 
         .D(swap_now_s5), .Z(n11552)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_339.init = 16'h5a66;
    LUT4 i7_4_lut_adj_340 (.A(spi_version[1]), .B(spi_version[5]), .C(spi_version[0]), 
         .D(spi_version[2]), .Z(n17)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(351[34:54])
    defparam i7_4_lut_adj_340.init = 16'hffef;
    LUT4 i14264_2_lut (.A(spi_bitmap[31]), .B(spi_bitmap[51]), .Z(n23048)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14264_2_lut.init = 16'h8888;
    LUT4 i1635_1_lut (.A(status_bit_index[3]), .Z(spi1_miso_N_2513[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i1635_1_lut.init = 16'h5555;
    LUT4 i14408_4_lut (.A(spi_bitmap[33]), .B(spi_bitmap[49]), .C(spi_bitmap[46]), 
         .D(spi_bitmap[0]), .Z(n23192)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14408_4_lut.init = 16'h8000;
    LUT4 n20258_bdd_4_lut_14717 (.A(ev_state[0]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(ev_state_3__N_1940[1]), .Z(n23587)) /* synthesis lut_function=(!(A (B (C)+!B !(C+(D)))+!A ((C)+!B))) */ ;
    defparam n20258_bdd_4_lut_14717.init = 16'h2e2c;
    LUT4 i14280_2_lut (.A(spi_bitmap[68]), .B(spi_bitmap[16]), .Z(n23064)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14280_2_lut.init = 16'h8888;
    LUT4 i11488_3_lut_then_4_lut (.A(ev_state[0]), .B(ev_state[2]), .C(ev_state[3]), 
         .D(n7), .Z(n23730)) /* synthesis lut_function=(!(A+(B (C)+!B !(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i11488_3_lut_then_4_lut.init = 16'h1404;
    LUT4 i1_3_lut_4_lut_adj_341 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[74]), 
         .D(ev_bit[74]), .Z(ev_wr_data[74])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_341.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_342 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[75]), 
         .D(ev_bit[75]), .Z(ev_wr_data[75])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_342.init = 16'hddd0;
    LUT4 i11488_3_lut_else_4_lut (.A(ev_state[2]), .B(ev_state[3]), .Z(n23729)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i11488_3_lut_else_4_lut.init = 16'h2222;
    LUT4 i2_2_lut_adj_343 (.A(spi_command[6]), .B(spi_command[3]), .Z(n6_adj_3191)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[58:78])
    defparam i2_2_lut_adj_343.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_344 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[76]), 
         .D(ev_bit[76]), .Z(ev_wr_data[76])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_344.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_345 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[77]), 
         .D(ev_bit[77]), .Z(ev_wr_data[77])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_345.init = 16'hddd0;
    LUT4 i1_2_lut_3_lut_4_lut_rep_141 (.A(n23694), .B(n23707), .C(n23688), 
         .D(ev_state[0]), .Z(pll_clk_enable_744)) /* synthesis lut_function=(A (B (C)+!B (C+!(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_3_lut_4_lut_rep_141.init = 16'hf0f2;
    LUT4 i1_3_lut_4_lut_adj_346 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[78]), 
         .D(ev_bit[78]), .Z(ev_wr_data[78])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_346.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_347 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[79]), 
         .D(ev_bit[79]), .Z(ev_wr_data[79])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_347.init = 16'hddd0;
    LUT4 i14384_4_lut (.A(spi_bitmap[23]), .B(spi_bitmap[2]), .C(spi_bitmap[1]), 
         .D(spi_bitmap[11]), .Z(n23168)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14384_4_lut.init = 16'h8000;
    FD1P3IX init_shadow_i55 (.D(n13044), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i55.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_348 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[80]), 
         .D(ev_bit[80]), .Z(ev_wr_data[80])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_348.init = 16'hddd0;
    LUT4 mux_1131_i5_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[4]), 
         .D(ev_clear_addr[4]), .Z(ev_wr_addr[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i5_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i54 (.D(n13038), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i54.GSR = "DISABLED";
    CCU2D fpga_time_1258_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22283), .COUT(n22284), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_23.INJECT1_1 = "NO";
    LUT4 i14226_2_lut (.A(spi_bitmap[3]), .B(spi_bitmap[8]), .Z(n23010)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14226_2_lut.init = 16'h8888;
    FD1P3IX ev_bit_i13 (.D(ev_bit[12]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i13.GSR = "DISABLED";
    FD1P3IX init_shadow_i34 (.D(n12914), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i34.GSR = "DISABLED";
    LUT4 mux_1131_i4_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[3]), 
         .D(ev_clear_addr[3]), .Z(ev_wr_addr[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i4_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i33 (.D(n12908), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i33.GSR = "DISABLED";
    FD1P3IX init_shadow_i32 (.D(n12902), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i32.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_349 (.A(ev_run_hold_s5[74]), .B(us_tx_c_74), .C(init_shadow[74]), 
         .D(swap_now_s5), .Z(n11554)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_349.init = 16'h5a66;
    FD1P3IX init_shadow_i31 (.D(n12896), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i31.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_350 (.A(ev_run_hold_s5[75]), .B(us_tx_c_75), .C(init_shadow[75]), 
         .D(swap_now_s5), .Z(n11556)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_350.init = 16'h5a66;
    LUT4 i2_3_lut_4_lut_rep_143 (.A(status_flags_wire_15__N_1401[4]), .B(pll_locked), 
         .C(pll_clk_enable_6), .D(phase_step_s5), .Z(pll_clk_enable_183)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_rep_143.init = 16'hfff7;
    FD1P3IX init_shadow_i42 (.D(n12962), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i42.GSR = "DISABLED";
    LUT4 i1_4_lut_adj_351 (.A(spi_version[7]), .B(n22922), .C(n23204), 
         .D(n22992), .Z(n22845)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i1_4_lut_adj_351.init = 16'h0004;
    FD1P3IX init_shadow_i41 (.D(n12956), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i41.GSR = "DISABLED";
    LUT4 i1_4_lut_4_lut_4_lut_rep_145 (.A(ev_state[0]), .B(ev_state[3]), 
         .C(ev_state[2]), .D(ev_state[1]), .Z(pll_clk_enable_282)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A ((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_4_lut_4_lut_4_lut_rep_145.init = 16'h0024;
    LUT4 i1636_1_lut (.A(status_bit_index[2]), .Z(spi1_miso_N_2513[2])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(284[18:44])
    defparam i1636_1_lut.init = 16'h5555;
    LUT4 i14209_2_lut (.A(spi_version[1]), .B(spi_version[2]), .Z(n22992)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14209_2_lut.init = 16'heeee;
    LUT4 i7_4_lut_adj_352 (.A(ev_run_hold_s5[76]), .B(us_tx_c_76), .C(init_shadow[76]), 
         .D(swap_now_s5), .Z(n11558)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_352.init = 16'h5a66;
    LUT4 i4_4_lut_adj_353 (.A(spi_command[4]), .B(spi_command[5]), .C(spi_command[6]), 
         .D(spi_command[2]), .Z(n10_adj_3199)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i4_4_lut_adj_353.init = 16'h0002;
    LUT4 mux_1131_i3_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[2]), 
         .D(ev_clear_addr[2]), .Z(ev_wr_addr[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i3_3_lut_4_lut.init = 16'hf2d0;
    spi_mic_stream mic_stream_i (.mic_latest({mic_latest}), .sck_N_3045(sck_N_3045), 
            .spi_mic_cs_n_c(spi_mic_cs_n_c), .spi_mic_miso_c(spi_mic_miso_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(610[20] 613[6])
    FD1P3AX global_phase_s2_1257__i2 (.D(n43), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i2.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1257__i3 (.D(n42), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i3.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1257__i4 (.D(n41), .SP(phase_step_s1), .CK(pll_clk), 
            .Q(global_phase_s2[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i4.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1257__i5 (.D(n40_adj_3168), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i5.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1257__i6 (.D(n39_adj_3167), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i6.GSR = "DISABLED";
    FD1P3AX global_phase_s2_1257__i7 (.D(n38_adj_3166), .SP(phase_step_s1), 
            .CK(pll_clk), .Q(global_phase_s2[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(378[32:54])
    defparam global_phase_s2_1257__i7.GSR = "DISABLED";
    FD1S3IX mic_divider_1261__i1 (.D(n39_adj_3183), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261__i1.GSR = "DISABLED";
    FD1P3IX init_shadow_i30 (.D(n12890), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i30.GSR = "DISABLED";
    FD1P3IX init_shadow_i53 (.D(n13032), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i53.GSR = "DISABLED";
    FD1P3IX init_shadow_i52 (.D(n13026), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i52.GSR = "DISABLED";
    FD1P3IX init_shadow_i51 (.D(n13020), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i51.GSR = "DISABLED";
    FD1P3IX ev_bit_i12 (.D(ev_bit[11]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i12.GSR = "DISABLED";
    LUT4 mux_1131_i2_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[1]), 
         .D(ev_clear_addr[1]), .Z(ev_wr_addr[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i5_3_lut (.A(spi_command[7]), .B(n10_adj_3199), .C(spi_command[3]), 
         .Z(n22922)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i5_3_lut.init = 16'h0404;
    LUT4 i6_4_lut_rep_147 (.A(mic_sample_count[3]), .B(n12), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_439)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_rep_147.init = 16'h0800;
    LUT4 i1_3_lut_4_lut_adj_354 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[81]), 
         .D(ev_bit[81]), .Z(ev_wr_data[81])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_354.init = 16'hddd0;
    LUT4 mux_1131_i1_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(ev_wr_addr_8__N_912[0]), 
         .D(ev_clear_addr[0]), .Z(ev_wr_addr[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam mux_1131_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14262_4_lut (.A(n7), .B(ev_state[2]), .C(ev_state[1]), .D(ev_state[0]), 
         .Z(n23046)) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+(D))) */ ;
    defparam i14262_4_lut.init = 16'hffec;
    LUT4 i1_3_lut_4_lut_adj_355 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[83]), 
         .D(ev_bit[83]), .Z(ev_wr_data[83])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_355.init = 16'hddd0;
    LUT4 i1_4_lut_4_lut_4_lut_adj_356 (.A(n23694), .B(n23707), .C(n23717), 
         .D(ev_state[0]), .Z(n16748)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_4_lut_4_lut_4_lut_adj_356.init = 16'h3022;
    LUT4 i6809_2_lut_4_lut_rep_151 (.A(ev_state[3]), .B(ev_state[0]), .C(ev_state[2]), 
         .D(n23694), .Z(n23972)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:25])
    defparam i6809_2_lut_4_lut_rep_151.init = 16'h0100;
    FD1P3IX ev_clear_addr_i0 (.D(ev_clear_addr_7__N_2237[0]), .SP(pll_clk_enable_727), 
            .CD(n15697), .CK(pll_clk), .Q(ev_clear_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_clear_addr_i0.GSR = "DISABLED";
    FD1P3IX init_shadow_i0 (.D(n11385), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i0.GSR = "DISABLED";
    FD1P3AX ev_state_i0 (.D(ev_state_3__N_697[0]), .SP(pll_clk_enable_729), 
            .CK(pll_clk), .Q(ev_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_state_i0.GSR = "DISABLED";
    FD1P3AX ws2812_toggle_spi_484 (.D(ws2812_toggle_spi_N_2556), .SP(spi1_sck_c_enable_238), 
            .CK(spi1_sck_c), .Q(ws2812_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam ws2812_toggle_spi_484.GSR = "DISABLED";
    FD1S3JX swap_pending_493 (.D(swap_pending_N_2586), .CK(pll_clk), .PD(n22891), 
            .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam swap_pending_493.GSR = "DISABLED";
    FD1P3IX rgb_hold__i1 (.D(rgb_values[0]), .SP(pll_clk_enable_731), .CD(n17128), 
            .CK(pll_clk), .Q(rgb_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam rgb_hold__i1.GSR = "DISABLED";
    FD1P3AX spi_bitmap_i0_i16 (.D(spi_bitmap[8]), .SP(spi1_sck_c_enable_239), 
            .CK(spi1_sck_c), .Q(spi_bitmap[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam spi_bitmap_i0_i16.GSR = "ENABLED";
    FD1S3IX mic_divider_1261__i2 (.D(n38_adj_3184), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261__i2.GSR = "DISABLED";
    FD1S3IX mic_divider_1261__i3 (.D(n37_adj_3185), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261__i3.GSR = "DISABLED";
    FD1S3IX mic_divider_1261__i4 (.D(n36_adj_3186), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261__i4.GSR = "DISABLED";
    FD1S3IX mic_divider_1261__i5 (.D(n35_adj_3187), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261__i5.GSR = "DISABLED";
    FD1S3IX mic_divider_1261__i6 (.D(n34_adj_3188), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(591[28:46])
    defparam mic_divider_1261__i6.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1256__i1 (.D(n19), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[34:54])
    defparam spi_bit_count_1256__i1.GSR = "ENABLED";
    FD1S3AX spi_bit_count_1256__i2 (.D(n18_adj_3179), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(362[34:54])
    defparam spi_bit_count_1256__i2.GSR = "ENABLED";
    FD1P3AX mic_sample_count_1260__i1 (.D(n29), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam mic_sample_count_1260__i1.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1260__i2 (.D(n28_adj_3177), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam mic_sample_count_1260__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1260__i3 (.D(n27), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam mic_sample_count_1260__i3.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1260__i4 (.D(n26_adj_3178), .SP(pll_clk_enable_736), 
            .CK(pll_clk), .Q(mic_sample_count[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(583[37:60])
    defparam mic_sample_count_1260__i4.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_357 (.A(ev_run_hold_s5[78]), .B(us_tx_c_78), .C(init_shadow[78]), 
         .D(swap_now_s5), .Z(n11562)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_357.init = 16'h5a66;
    LUT4 i6700_2_lut_3_lut_4_lut_rep_153 (.A(n23694), .B(n23707), .C(n23688), 
         .D(ev_state[0]), .Z(n23974)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i6700_2_lut_3_lut_4_lut_rep_153.init = 16'h0002;
    LUT4 i4710_2_lut_3_lut_4_lut (.A(n23694), .B(n23707), .C(pll_clk_enable_6), 
         .D(ev_state[0]), .Z(n13590)) /* synthesis lut_function=(A (B (C)+!B (C+!(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i4710_2_lut_3_lut_4_lut.init = 16'hf0f2;
    LUT4 i1_2_lut_3_lut_4_lut_rep_155 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1401[4]), .Z(n23976)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(252[23:61])
    defparam i1_2_lut_3_lut_4_lut_rep_155.init = 16'h6fff;
    LUT4 i6700_2_lut_3_lut_4_lut (.A(n23694), .B(n23707), .C(n23688), 
         .D(ev_state[0]), .Z(n15506)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i6700_2_lut_3_lut_4_lut.init = 16'h0002;
    LUT4 i7_4_lut_adj_358 (.A(ev_run_hold_s5[77]), .B(us_tx_c_77), .C(init_shadow[77]), 
         .D(swap_now_s5), .Z(n11560)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_358.init = 16'h5a66;
    LUT4 i9864_3_lut (.A(ev_ch[0]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(ev_ch_6__N_709[0])) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(482[9] 555[16])
    defparam i9864_3_lut.init = 16'h4848;
    LUT4 i1_2_lut_3_lut_4_lut_adj_359 (.A(n23694), .B(n23707), .C(n23688), 
         .D(ev_state[0]), .Z(pll_clk_enable_705)) /* synthesis lut_function=(A (B (C)+!B (C+!(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_3_lut_4_lut_adj_359.init = 16'hf0f2;
    LUT4 mux_1408_i2_3_lut (.A(n10159), .B(n10160), .C(n10156), .Z(rd_data_15__N_2647[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i2_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_360 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[20]), 
         .D(ev_bit[20]), .Z(ev_wr_data[20])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_360.init = 16'hddd0;
    LUT4 frame_end_I_0_4_lut (.A(n25), .B(frame_end_N_2611), .C(n23665), 
         .D(n26), .Z(frame_end)) /* synthesis lut_function=(A (B (C))+!A (B (C+!(D))+!B !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[29] 293[95])
    defparam frame_end_I_0_4_lut.init = 16'hc0c5;
    LUT4 i7_4_lut_adj_361 (.A(ev_run_hold_s5[79]), .B(us_tx_c_79), .C(init_shadow[79]), 
         .D(swap_now_s5), .Z(n11564)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_361.init = 16'h5a66;
    LUT4 i11_4_lut (.A(expected_next[2]), .B(n22), .C(n23698), .D(expected_next[5]), 
         .Z(n25)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;
    defparam i11_4_lut.init = 16'hfdff;
    LUT4 mux_1408_i3_3_lut (.A(n10161), .B(n10162), .C(n10156), .Z(rd_data_15__N_2647[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_adj_362 (.A(spi1_sck_c_enable_237), .B(spi_channel_field[1]), 
         .C(spi_channel_field[0]), .Z(spi1_sck_c_enable_236)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_3_lut_adj_362.init = 16'h2020;
    LUT4 i7_4_lut_adj_363 (.A(ev_run_hold_s5[80]), .B(us_tx_c_80), .C(init_shadow[80]), 
         .D(swap_now_s5), .Z(n11566)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:28])
    defparam i7_4_lut_adj_363.init = 16'h5a66;
    LUT4 mux_1408_i4_3_lut (.A(n10163), .B(n10164), .C(n10156), .Z(rd_data_15__N_2647[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i4_3_lut.init = 16'hcaca;
    FD1S3AX phase_step_s4_497_rep_149 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(pll_clk_enable_576)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s4_497_rep_149.GSR = "DISABLED";
    LUT4 mux_1408_i5_3_lut (.A(n10165), .B(n10166), .C(n10156), .Z(rd_data_15__N_2647[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i5_3_lut.init = 16'hcaca;
    LUT4 mux_1408_i6_3_lut (.A(n10167), .B(n10168), .C(n10156), .Z(rd_data_15__N_2647[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i6_3_lut.init = 16'hcaca;
    LUT4 i3_4_lut_rep_89 (.A(n23687), .B(ev_state[0]), .C(frame_req), 
         .D(swap_pending), .Z(n23678)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i3_4_lut_rep_89.init = 16'hfffe;
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .\ev_wr_addr[0] (ev_wr_addr[0]), .event_rd_addr({event_rd_addr}), 
            .\ev_wr_addr[1] (ev_wr_addr[1]), .\ev_wr_addr[2] (ev_wr_addr[2]), 
            .\ev_wr_addr[3] (ev_wr_addr[3]), .\ev_wr_addr[4] (ev_wr_addr[4]), 
            .\ev_wr_addr[5] (ev_wr_addr[5]), .\ev_wr_addr[6] (ev_wr_addr[6]), 
            .\ev_wr_addr[7] (ev_wr_addr[7]), .n23720(n23720), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(229[22] 234[6])
    LUT4 i35_1_lut_4_lut (.A(n23687), .B(ev_state[0]), .C(frame_req), 
         .D(swap_pending), .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i35_1_lut_4_lut.init = 16'h0001;
    INV i14818 (.A(spi1_sck_c), .Z(spi1_sck_N_416));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    FD1P3IX ev_bit_i11 (.D(ev_bit[10]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i11.GSR = "DISABLED";
    LUT4 i1502_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23691), .C(ev_ch[5]), 
         .D(ev_ch[4]), .Z(ev_ch_6__N_1948[5])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1502_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i12_4_lut (.A(expected_next[6]), .B(n24), .C(n18), .D(expected_next[12]), 
         .Z(n26)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i12_4_lut.init = 16'hfffe;
    LUT4 mux_1130_i3_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[2]), 
         .D(build_phase[2]), .Z(ev_wr_addr_8__N_912[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1130_i2_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[1]), 
         .D(build_phase[1]), .Z(ev_wr_addr_8__N_912[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i8_4_lut (.A(expected_next[9]), .B(expected_next[13]), .C(expected_next[4]), 
         .D(expected_next[3]), .Z(n22)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i8_4_lut.init = 16'hfffe;
    LUT4 i10_4_lut (.A(expected_next[11]), .B(expected_next[10]), .C(expected_next[14]), 
         .D(expected_next[15]), .Z(n24)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i10_4_lut.init = 16'hfffe;
    FD1P3IX ev_bit_i10 (.D(ev_bit[9]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i10.GSR = "DISABLED";
    PFUMX i14726 (.BLUT(n23729), .ALUT(n23730), .C0(ev_state[1]), .Z(ev_state_3__N_697[2]));
    LUT4 i1_3_lut_4_lut_adj_364 (.A(ev_state[0]), .B(n23687), .C(ev_rd_hold[21]), 
         .D(ev_bit[21]), .Z(ev_wr_data[21])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_3_lut_4_lut_adj_364.init = 16'hddd0;
    LUT4 mux_1130_i1_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[0]), 
         .D(build_phase[0]), .Z(ev_wr_addr_8__N_912[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1408_i7_3_lut (.A(n10169), .B(n10170), .C(n10156), .Z(rd_data_15__N_2647[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i7_3_lut.init = 16'hcaca;
    LUT4 mux_1130_i4_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[3]), 
         .D(build_phase[3]), .Z(ev_wr_addr_8__N_912[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i1509_3_lut_4_lut (.A(ev_ch[4]), .B(n23679), .C(ev_ch[5]), .D(ev_ch[6]), 
         .Z(ev_ch_6__N_1948[6])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[27:39])
    defparam i1509_3_lut_4_lut.init = 16'h7f80;
    GSR GSR_INST (.GSR(fpga_cs_n_N_2521));
    CCU2D fpga_time_1258_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22282), .COUT(n22283), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(414[29:46])
    defparam fpga_time_1258_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_1258_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_1258_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_1258_add_4_21.INJECT1_1 = "NO";
    CCU2D spi_channel_index_1254_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_channel_index[0]), .B1(spi_channel_index[6]), 
          .C1(n8), .D1(n22867), .COUT(n22261), .S1(n40_adj_3169));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(329[64:88])
    defparam spi_channel_index_1254_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_1254_add_4_1.INIT1 = 16'h5559;
    defparam spi_channel_index_1254_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_1254_add_4_1.INJECT1_1 = "NO";
    PFUMX i14724 (.BLUT(n23726), .ALUT(n23727), .C0(ev_state[3]), .Z(ev_state_3__N_697[0]));
    CCU2D expected_next_15__I_0_647_15 (.A0(spi_rx_shift[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22260), .S0(expected_next[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[33] 290[86])
    defparam expected_next_15__I_0_647_15.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_647_15.INIT1 = 16'h0000;
    defparam expected_next_15__I_0_647_15.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_647_15.INJECT1_1 = "NO";
    FD1P3IX init_shadow_i60 (.D(n13074), .SP(pll_clk_enable_739), .CD(n15614), 
            .CK(pll_clk), .Q(init_shadow[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i60.GSR = "DISABLED";
    LUT4 mux_1130_i5_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[4]), 
         .D(build_phase[4]), .Z(ev_wr_addr_8__N_912[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i5_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_1130_i6_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[5]), 
         .D(build_phase[5]), .Z(ev_wr_addr_8__N_912[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i6_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX ev_bit_i9 (.D(ev_bit[8]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i40 (.D(n12950), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i40.GSR = "DISABLED";
    LUT4 mux_1130_i7_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[6]), 
         .D(build_phase[6]), .Z(ev_wr_addr_8__N_912[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i7_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i39 (.D(n12944), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i39.GSR = "DISABLED";
    FD1P3IX init_shadow_i38 (.D(n12938), .SP(pll_clk_enable_743), .CD(n23972), 
            .CK(pll_clk), .Q(init_shadow[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam init_shadow_i38.GSR = "DISABLED";
    ws2812_stream ws2812_i (.n15(n15), .state({state}), .ws2812_enable(ws2812_enable), 
            .pll_clk(pll_clk), .\shift_register[17] (shift_register[17]), 
            .\shift_register_23__N_2908[17] (shift_register_23__N_2908[17]), 
            .\shift_register[18] (shift_register[18]), .\shift_register_23__N_2908[18] (shift_register_23__N_2908[18]), 
            .\shift_register[19] (shift_register[19]), .\shift_register_23__N_2908[19] (shift_register_23__N_2908[19]), 
            .\shift_register[20] (shift_register[20]), .\shift_register_23__N_2908[20] (shift_register_23__N_2908[20]), 
            .\shift_register[21] (shift_register[21]), .\shift_register_23__N_2908[21] (shift_register_23__N_2908[21]), 
            .\shift_register[22] (shift_register[22]), .\shift_register_23__N_2908[22] (shift_register_23__N_2908[22]), 
            .\shift_register_23__N_2908[23] (shift_register_23__N_2908[23]), 
            .\shift_register[16] (shift_register[16]), .\rgb_hold[0] (rgb_hold[0]), 
            .GND_net(GND_net), .rgb_data_c(rgb_data_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(597[19] 609[6])
    LUT4 mux_1130_i8_3_lut_4_lut (.A(ev_state[1]), .B(n23686), .C(build_sum[7]), 
         .D(build_phase[7]), .Z(ev_wr_addr_8__N_912[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[33:53])
    defparam mux_1130_i8_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i4_2_lut (.A(expected_next[8]), .B(expected_next[7]), .Z(n18)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i4_2_lut.init = 16'heeee;
    PFUMX i14722 (.BLUT(n23723), .ALUT(n23724), .C0(status_hold[74]), 
          .Z(n23725));
    LUT4 i14213_3_lut_4_lut (.A(pll_clk_enable_12), .B(n23700), .C(update_flags_sync[1]), 
         .D(pll_clk_enable_6), .Z(n17128)) /* synthesis lut_function=(A ((D)+!C)+!A (B ((D)+!C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(457[9] 468[12])
    defparam i14213_3_lut_4_lut.init = 16'hff0e;
    FD1S3AX phase_step_s4_497_rep_135 (.D(phase_step_s3), .CK(pll_clk), 
            .Q(n23956)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam phase_step_s4_497_rep_135.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_81_3_lut_4_lut (.A(ev_state[0]), .B(n23687), .C(n23707), 
         .D(n23694), .Z(pll_clk_enable_727)) /* synthesis lut_function=(!(A (B (C+!(D)))+!A (C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam i1_2_lut_rep_81_3_lut_4_lut.init = 16'h2f22;
    CCU2D equal_1861_0 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(frame_end_N_2612[16]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22133));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(293[48:95])
    defparam equal_1861_0.INIT0 = 16'hF000;
    defparam equal_1861_0.INIT1 = 16'h5555;
    defparam equal_1861_0.INJECT1_0 = "NO";
    defparam equal_1861_0.INJECT1_1 = "YES";
    LUT4 i1_2_lut_3_lut_adj_365 (.A(spi1_sck_c_enable_237), .B(spi_channel_field[1]), 
         .C(spi_channel_field[0]), .Z(spi1_sck_c_enable_57)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(304[18] 364[12])
    defparam i1_2_lut_3_lut_adj_365.init = 16'h0202;
    FD1P3IX ev_bit_i46 (.D(ev_bit[45]), .SP(pll_clk_enable_744), .CD(n23974), 
            .CK(pll_clk), .Q(ev_bit[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(370[12] 593[8])
    defparam ev_bit_i46.GSR = "DISABLED";
    LUT4 mux_1408_i8_3_lut (.A(n10171), .B(n10172), .C(n10156), .Z(rd_data_15__N_2647[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1408_i8_3_lut.init = 16'hcaca;
    umh_channel_ram18 staging_ram (.n10152(n10152), .spi1_sck_c(spi1_sck_c), 
            .spi_channel_index({spi_channel_index}), .staging_q({staging_q}), 
            .pll_clk(pll_clk), .rd_data_15__N_2647({rd_data_15__N_2647}), 
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
            .n10144(n10144), .n10146(n10146), .n7(n7)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(207[23] 210[6])
    
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
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (mic_latest, sck_N_3045, spi_mic_cs_n_c, spi_mic_miso_c) /* synthesis syn_module_defined=1 */ ;
    input [63:0]mic_latest;
    input sck_N_3045;
    input spi_mic_cs_n_c;
    output spi_mic_miso_c;
    
    wire sck_N_3045 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [6:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(12[11:20])
    
    wire n12, n23689, n23709, n15712;
    wire [95:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [95:0]shift_register_95__N_2948;
    
    wire n23672, n23690, n15710, n23677;
    wire [6:0]bit_count_6__N_3046;
    
    wire sck_N_3045_enable_101, n15772, n15770, n15768, n15766, n15764, 
        n15762, n15760, n15758, n15756, n15754, n15752, n15750, 
        n15748, n15746, n15744, n15742, n15740, n15738, n15736, 
        n15734, n15732, n15730, n15728, n15726, n15724, n15722, 
        n15720, n15718, n15708, n15716, n15714;
    
    LUT4 i6_4_lut_rep_100 (.A(bit_count[2]), .B(n12), .C(bit_count[6]), 
         .D(bit_count[1]), .Z(n23689)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i6_4_lut_rep_100.init = 16'hfffe;
    LUT4 i9879_3_lut_4_lut (.A(bit_count[2]), .B(n23709), .C(n23689), 
         .D(bit_count[3]), .Z(n15712)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9879_3_lut_4_lut.init = 16'h7080;
    LUT4 shift_register_95__I_0_19_i26_3_lut (.A(mic_latest[16]), .B(shift_register[24]), 
         .C(n23689), .Z(shift_register_95__N_2948[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i26_3_lut.init = 16'hcaca;
    LUT4 i1547_2_lut_rep_83_3_lut_4_lut (.A(bit_count[2]), .B(n23709), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n23672)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1547_2_lut_rep_83_3_lut_4_lut.init = 16'h8000;
    LUT4 shift_register_95__I_0_19_i27_3_lut (.A(mic_latest[17]), .B(shift_register[25]), 
         .C(n23689), .Z(shift_register_95__N_2948[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i27_3_lut.init = 16'hcaca;
    LUT4 i1526_2_lut_rep_120 (.A(bit_count[1]), .B(bit_count[0]), .Z(n23709)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1526_2_lut_rep_120.init = 16'h8888;
    LUT4 i1533_2_lut_rep_101_3_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[2]), .Z(n23690)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1533_2_lut_rep_101_3_lut.init = 16'h8080;
    LUT4 i9878_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), .C(n23689), 
         .D(bit_count[2]), .Z(n15710)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9878_3_lut_4_lut.init = 16'h7080;
    LUT4 i1540_2_lut_rep_88_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[3]), .D(bit_count[2]), .Z(n23677)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1540_2_lut_rep_88_3_lut_4_lut.init = 16'h8000;
    LUT4 shift_register_95__I_0_19_i28_3_lut (.A(mic_latest[18]), .B(shift_register[26]), 
         .C(n23689), .Z(shift_register_95__N_2948[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i28_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i29_3_lut (.A(mic_latest[19]), .B(shift_register[27]), 
         .C(n23689), .Z(shift_register_95__N_2948[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i29_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i30_3_lut (.A(mic_latest[20]), .B(shift_register[28]), 
         .C(n23689), .Z(shift_register_95__N_2948[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i30_3_lut.init = 16'hcaca;
    FD1S3DX bit_count_i0 (.D(bit_count_6__N_3046[0]), .CK(sck_N_3045), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i0.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i31_3_lut (.A(mic_latest[21]), .B(shift_register[29]), 
         .C(n23689), .Z(shift_register_95__N_2948[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i31_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i32_3_lut (.A(mic_latest[22]), .B(shift_register[30]), 
         .C(n23689), .Z(shift_register_95__N_2948[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i32_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i33_3_lut (.A(mic_latest[23]), .B(shift_register[31]), 
         .C(n23689), .Z(shift_register_95__N_2948[32])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i33_3_lut.init = 16'hcaca;
    LUT4 i9873_2_lut (.A(mic_latest[0]), .B(n23689), .Z(shift_register_95__N_2948[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9873_2_lut.init = 16'h2222;
    LUT4 shift_register_95__I_0_19_i3_3_lut (.A(mic_latest[1]), .B(shift_register[1]), 
         .C(n23689), .Z(shift_register_95__N_2948[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut (.A(n23689), .B(shift_register[95]), .C(spi_mic_cs_n_c), 
         .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[15] 26[68])
    defparam i1_3_lut.init = 16'h0808;
    LUT4 shift_register_95__I_0_19_i4_3_lut (.A(mic_latest[2]), .B(shift_register[2]), 
         .C(n23689), .Z(shift_register_95__N_2948[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i4_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i5_3_lut (.A(mic_latest[3]), .B(shift_register[3]), 
         .C(n23689), .Z(shift_register_95__N_2948[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i5_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i6_3_lut (.A(mic_latest[4]), .B(shift_register[4]), 
         .C(n23689), .Z(shift_register_95__N_2948[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i6_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i7_3_lut (.A(mic_latest[5]), .B(shift_register[5]), 
         .C(n23689), .Z(shift_register_95__N_2948[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i7_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i8_3_lut (.A(mic_latest[6]), .B(shift_register[6]), 
         .C(n23689), .Z(shift_register_95__N_2948[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i8_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i9_3_lut (.A(mic_latest[7]), .B(shift_register[7]), 
         .C(n23689), .Z(shift_register_95__N_2948[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i9_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i10_3_lut (.A(mic_latest[8]), .B(shift_register[8]), 
         .C(n23689), .Z(shift_register_95__N_2948[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i10_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i11_3_lut (.A(mic_latest[9]), .B(shift_register[9]), 
         .C(n23689), .Z(shift_register_95__N_2948[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i11_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i12_3_lut (.A(mic_latest[10]), .B(shift_register[10]), 
         .C(n23689), .Z(shift_register_95__N_2948[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i12_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i13_3_lut (.A(mic_latest[11]), .B(shift_register[11]), 
         .C(n23689), .Z(shift_register_95__N_2948[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i13_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i14_3_lut (.A(mic_latest[12]), .B(shift_register[12]), 
         .C(n23689), .Z(shift_register_95__N_2948[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i14_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i15_3_lut (.A(mic_latest[13]), .B(shift_register[13]), 
         .C(n23689), .Z(shift_register_95__N_2948[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i15_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i16_3_lut (.A(mic_latest[14]), .B(shift_register[14]), 
         .C(n23689), .Z(shift_register_95__N_2948[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i16_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i17_3_lut (.A(mic_latest[15]), .B(shift_register[15]), 
         .C(n23689), .Z(shift_register_95__N_2948[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i17_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i34_3_lut (.A(mic_latest[24]), .B(shift_register[32]), 
         .C(n23689), .Z(shift_register_95__N_2948[33])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i34_3_lut.init = 16'hcaca;
    LUT4 i9874_2_lut (.A(shift_register[16]), .B(n23689), .Z(shift_register_95__N_2948[17])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9874_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i35_3_lut (.A(mic_latest[25]), .B(shift_register[33]), 
         .C(n23689), .Z(shift_register_95__N_2948[34])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i35_3_lut.init = 16'hcaca;
    LUT4 i9875_2_lut (.A(shift_register[17]), .B(n23689), .Z(shift_register_95__N_2948[18])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9875_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i36_3_lut (.A(mic_latest[26]), .B(shift_register[34]), 
         .C(n23689), .Z(shift_register_95__N_2948[35])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i36_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i37_3_lut (.A(mic_latest[27]), .B(shift_register[35]), 
         .C(n23689), .Z(shift_register_95__N_2948[36])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i37_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i38_3_lut (.A(mic_latest[28]), .B(shift_register[36]), 
         .C(n23689), .Z(shift_register_95__N_2948[37])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i38_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i39_3_lut (.A(mic_latest[29]), .B(shift_register[37]), 
         .C(n23689), .Z(shift_register_95__N_2948[38])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i39_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i40_3_lut (.A(mic_latest[30]), .B(shift_register[38]), 
         .C(n23689), .Z(shift_register_95__N_2948[39])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i40_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i41_3_lut (.A(mic_latest[31]), .B(shift_register[39]), 
         .C(n23689), .Z(shift_register_95__N_2948[40])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i41_3_lut.init = 16'hcaca;
    LUT4 i9876_2_lut (.A(shift_register[41]), .B(n23689), .Z(shift_register_95__N_2948[42])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9876_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i50_3_lut (.A(mic_latest[32]), .B(shift_register[48]), 
         .C(n23689), .Z(shift_register_95__N_2948[49])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i50_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i51_3_lut (.A(mic_latest[33]), .B(shift_register[49]), 
         .C(n23689), .Z(shift_register_95__N_2948[50])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i51_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i52_3_lut (.A(mic_latest[34]), .B(shift_register[50]), 
         .C(n23689), .Z(shift_register_95__N_2948[51])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i52_3_lut.init = 16'hcaca;
    LUT4 i14603_3_lut_4_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n23689), 
         .D(bit_count[0]), .Z(bit_count_6__N_3046[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D))) */ ;
    defparam i14603_3_lut_4_lut.init = 16'h8f7f;
    LUT4 i14606_2_lut_2_lut_3_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n23689), 
         .Z(sck_N_3045_enable_101)) /* synthesis lut_function=(!(A (B (C)))) */ ;
    defparam i14606_2_lut_2_lut_3_lut.init = 16'h7f7f;
    LUT4 shift_register_95__I_0_19_i53_3_lut (.A(mic_latest[35]), .B(shift_register[51]), 
         .C(n23689), .Z(shift_register_95__N_2948[52])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i53_3_lut.init = 16'hcaca;
    LUT4 i9909_2_lut (.A(shift_register[94]), .B(n23689), .Z(n15772)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9909_2_lut.init = 16'h8888;
    LUT4 i9908_2_lut (.A(shift_register[93]), .B(n23689), .Z(n15770)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9908_2_lut.init = 16'h8888;
    LUT4 i9907_2_lut (.A(shift_register[92]), .B(n23689), .Z(n15768)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9907_2_lut.init = 16'h8888;
    LUT4 i9906_2_lut (.A(shift_register[91]), .B(n23689), .Z(n15766)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9906_2_lut.init = 16'h8888;
    LUT4 i9905_2_lut (.A(shift_register[90]), .B(n23689), .Z(n15764)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9905_2_lut.init = 16'h8888;
    LUT4 i9904_2_lut (.A(shift_register[89]), .B(n23689), .Z(n15762)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9904_2_lut.init = 16'h8888;
    LUT4 i9903_2_lut (.A(shift_register[88]), .B(n23689), .Z(n15760)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9903_2_lut.init = 16'h8888;
    LUT4 i9902_2_lut (.A(shift_register[71]), .B(n23689), .Z(n15758)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9902_2_lut.init = 16'h8888;
    LUT4 i9901_2_lut (.A(shift_register[70]), .B(n23689), .Z(n15756)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9901_2_lut.init = 16'h8888;
    LUT4 i9900_2_lut (.A(shift_register[69]), .B(n23689), .Z(n15754)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9900_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i1 (.D(shift_register_95__N_2948[1]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3DX shift_register_i2 (.D(shift_register_95__N_2948[2]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3DX shift_register_i3 (.D(shift_register_95__N_2948[3]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3DX shift_register_i4 (.D(shift_register_95__N_2948[4]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3DX shift_register_i5 (.D(shift_register_95__N_2948[5]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3DX shift_register_i6 (.D(shift_register_95__N_2948[6]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i7 (.D(shift_register_95__N_2948[7]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3DX shift_register_i8 (.D(shift_register_95__N_2948[8]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3DX shift_register_i9 (.D(shift_register_95__N_2948[9]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3DX shift_register_i10 (.D(shift_register_95__N_2948[10]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3DX shift_register_i11 (.D(shift_register_95__N_2948[11]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3DX shift_register_i12 (.D(shift_register_95__N_2948[12]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3DX shift_register_i13 (.D(shift_register_95__N_2948[13]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3DX shift_register_i14 (.D(shift_register_95__N_2948[14]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3DX shift_register_i15 (.D(shift_register_95__N_2948[15]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3DX shift_register_i16 (.D(shift_register_95__N_2948[16]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[16])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3DX shift_register_i17 (.D(shift_register_95__N_2948[17]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[17])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3DX shift_register_i18 (.D(shift_register_95__N_2948[18]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[18])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3DX shift_register_i25 (.D(shift_register_95__N_2948[25]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i25.GSR = "DISABLED";
    FD1P3DX shift_register_i26 (.D(shift_register_95__N_2948[26]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i26.GSR = "DISABLED";
    FD1P3DX shift_register_i27 (.D(shift_register_95__N_2948[27]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i27.GSR = "DISABLED";
    FD1P3DX shift_register_i28 (.D(shift_register_95__N_2948[28]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i28.GSR = "DISABLED";
    FD1P3DX shift_register_i29 (.D(shift_register_95__N_2948[29]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i29.GSR = "DISABLED";
    FD1P3DX shift_register_i30 (.D(shift_register_95__N_2948[30]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i30.GSR = "DISABLED";
    FD1P3DX shift_register_i31 (.D(shift_register_95__N_2948[31]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i31.GSR = "DISABLED";
    FD1P3DX shift_register_i32 (.D(shift_register_95__N_2948[32]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[32])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i32.GSR = "DISABLED";
    FD1P3DX shift_register_i33 (.D(shift_register_95__N_2948[33]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[33])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i33.GSR = "DISABLED";
    FD1P3DX shift_register_i34 (.D(shift_register_95__N_2948[34]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[34])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i34.GSR = "DISABLED";
    FD1P3DX shift_register_i35 (.D(shift_register_95__N_2948[35]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[35])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i35.GSR = "DISABLED";
    FD1P3DX shift_register_i36 (.D(shift_register_95__N_2948[36]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[36])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i36.GSR = "DISABLED";
    FD1P3DX shift_register_i37 (.D(shift_register_95__N_2948[37]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[37])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i37.GSR = "DISABLED";
    FD1P3DX shift_register_i38 (.D(shift_register_95__N_2948[38]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[38])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i38.GSR = "DISABLED";
    FD1P3DX shift_register_i39 (.D(shift_register_95__N_2948[39]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[39])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i39.GSR = "DISABLED";
    FD1P3DX shift_register_i40 (.D(shift_register_95__N_2948[40]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[40])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i40.GSR = "DISABLED";
    FD1P3DX shift_register_i42 (.D(shift_register_95__N_2948[42]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[42])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i42.GSR = "DISABLED";
    FD1P3DX shift_register_i49 (.D(shift_register_95__N_2948[49]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[49])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i49.GSR = "DISABLED";
    FD1P3DX shift_register_i50 (.D(shift_register_95__N_2948[50]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[50])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i50.GSR = "DISABLED";
    FD1P3DX shift_register_i51 (.D(shift_register_95__N_2948[51]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[51])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i51.GSR = "DISABLED";
    FD1P3DX shift_register_i52 (.D(shift_register_95__N_2948[52]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[52])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i52.GSR = "DISABLED";
    FD1P3DX shift_register_i53 (.D(shift_register_95__N_2948[53]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[53])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i53.GSR = "DISABLED";
    FD1P3DX shift_register_i54 (.D(shift_register_95__N_2948[54]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[54])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i54.GSR = "DISABLED";
    FD1P3DX shift_register_i55 (.D(shift_register_95__N_2948[55]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[55])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i55.GSR = "DISABLED";
    FD1P3DX shift_register_i56 (.D(shift_register_95__N_2948[56]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[56])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i56.GSR = "DISABLED";
    FD1P3DX shift_register_i57 (.D(shift_register_95__N_2948[57]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[57])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i57.GSR = "DISABLED";
    FD1P3DX shift_register_i58 (.D(shift_register_95__N_2948[58]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[58])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i58.GSR = "DISABLED";
    FD1P3DX shift_register_i59 (.D(shift_register_95__N_2948[59]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[59])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i59.GSR = "DISABLED";
    FD1P3DX shift_register_i60 (.D(shift_register_95__N_2948[60]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[60])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i60.GSR = "DISABLED";
    FD1P3DX shift_register_i61 (.D(shift_register_95__N_2948[61]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[61])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i61.GSR = "DISABLED";
    FD1P3DX shift_register_i62 (.D(shift_register_95__N_2948[62]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[62])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i62.GSR = "DISABLED";
    FD1P3DX shift_register_i63 (.D(shift_register_95__N_2948[63]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[63])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i63.GSR = "DISABLED";
    FD1P3DX shift_register_i64 (.D(shift_register_95__N_2948[64]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[64])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i64.GSR = "DISABLED";
    FD1P3DX shift_register_i65 (.D(shift_register_95__N_2948[65]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[65])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i65.GSR = "DISABLED";
    FD1P3DX shift_register_i73 (.D(shift_register_95__N_2948[73]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[73])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i73.GSR = "DISABLED";
    FD1P3DX shift_register_i74 (.D(shift_register_95__N_2948[74]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[74])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i74.GSR = "DISABLED";
    FD1P3DX shift_register_i75 (.D(shift_register_95__N_2948[75]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[75])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i75.GSR = "DISABLED";
    FD1P3DX shift_register_i76 (.D(shift_register_95__N_2948[76]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[76])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i76.GSR = "DISABLED";
    FD1P3DX shift_register_i77 (.D(shift_register_95__N_2948[77]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[77])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i77.GSR = "DISABLED";
    FD1P3DX shift_register_i78 (.D(shift_register_95__N_2948[78]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[78])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i78.GSR = "DISABLED";
    FD1P3DX shift_register_i79 (.D(shift_register_95__N_2948[79]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[79])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i79.GSR = "DISABLED";
    FD1P3DX shift_register_i80 (.D(shift_register_95__N_2948[80]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[80])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i80.GSR = "DISABLED";
    FD1P3DX shift_register_i81 (.D(shift_register_95__N_2948[81]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[81])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i81.GSR = "DISABLED";
    FD1P3DX shift_register_i82 (.D(shift_register_95__N_2948[82]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[82])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i82.GSR = "DISABLED";
    FD1P3DX shift_register_i83 (.D(shift_register_95__N_2948[83]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[83])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i83.GSR = "DISABLED";
    FD1P3DX shift_register_i84 (.D(shift_register_95__N_2948[84]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[84])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i84.GSR = "DISABLED";
    FD1P3DX shift_register_i85 (.D(shift_register_95__N_2948[85]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[85])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i85.GSR = "DISABLED";
    FD1P3DX shift_register_i86 (.D(shift_register_95__N_2948[86]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[86])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i86.GSR = "DISABLED";
    FD1P3DX shift_register_i87 (.D(shift_register_95__N_2948[87]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[87])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i87.GSR = "DISABLED";
    FD1P3DX shift_register_i88 (.D(shift_register_95__N_2948[88]), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[88])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i88.GSR = "DISABLED";
    LUT4 i9899_2_lut (.A(shift_register[68]), .B(n23689), .Z(n15752)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9899_2_lut.init = 16'h8888;
    LUT4 i9898_2_lut (.A(shift_register[67]), .B(n23689), .Z(n15750)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9898_2_lut.init = 16'h8888;
    LUT4 i9897_2_lut (.A(shift_register[66]), .B(n23689), .Z(n15748)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9897_2_lut.init = 16'h8888;
    LUT4 i9896_2_lut (.A(shift_register[65]), .B(n23689), .Z(n15746)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9896_2_lut.init = 16'h8888;
    LUT4 i9895_2_lut (.A(shift_register[47]), .B(n23689), .Z(n15744)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9895_2_lut.init = 16'h8888;
    LUT4 i9894_2_lut (.A(shift_register[46]), .B(n23689), .Z(n15742)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9894_2_lut.init = 16'h8888;
    LUT4 i9893_2_lut (.A(shift_register[45]), .B(n23689), .Z(n15740)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9893_2_lut.init = 16'h8888;
    LUT4 i9892_2_lut (.A(shift_register[44]), .B(n23689), .Z(n15738)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9892_2_lut.init = 16'h8888;
    LUT4 i9891_2_lut (.A(shift_register[43]), .B(n23689), .Z(n15736)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9891_2_lut.init = 16'h8888;
    LUT4 i9890_2_lut (.A(shift_register[42]), .B(n23689), .Z(n15734)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9890_2_lut.init = 16'h8888;
    LUT4 i9889_2_lut (.A(shift_register[40]), .B(n23689), .Z(n15732)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9889_2_lut.init = 16'h8888;
    LUT4 i9888_2_lut (.A(shift_register[23]), .B(n23689), .Z(n15730)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9888_2_lut.init = 16'h8888;
    LUT4 i9887_2_lut (.A(shift_register[22]), .B(n23689), .Z(n15728)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9887_2_lut.init = 16'h8888;
    LUT4 i9886_2_lut (.A(shift_register[21]), .B(n23689), .Z(n15726)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9886_2_lut.init = 16'h8888;
    LUT4 i9885_2_lut (.A(shift_register[20]), .B(n23689), .Z(n15724)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9885_2_lut.init = 16'h8888;
    LUT4 i9884_2_lut (.A(shift_register[19]), .B(n23689), .Z(n15722)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9884_2_lut.init = 16'h8888;
    LUT4 i9883_2_lut (.A(shift_register[18]), .B(n23689), .Z(n15720)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9883_2_lut.init = 16'h8888;
    LUT4 i9882_4_lut (.A(bit_count[6]), .B(n23689), .C(bit_count[5]), 
         .D(n23672), .Z(n15718)) /* synthesis lut_function=(!(A ((C (D))+!B)+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9882_4_lut.init = 16'h4888;
    LUT4 shift_register_95__I_0_19_i54_3_lut (.A(mic_latest[36]), .B(shift_register[52]), 
         .C(n23689), .Z(shift_register_95__N_2948[53])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i54_3_lut.init = 16'hcaca;
    LUT4 i9868_3_lut (.A(bit_count[1]), .B(n23689), .C(bit_count[0]), 
         .Z(n15708)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i9868_3_lut.init = 16'h4848;
    LUT4 shift_register_95__I_0_19_i55_3_lut (.A(mic_latest[37]), .B(shift_register[53]), 
         .C(n23689), .Z(shift_register_95__N_2948[54])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i55_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i56_3_lut (.A(mic_latest[38]), .B(shift_register[54]), 
         .C(n23689), .Z(shift_register_95__N_2948[55])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i56_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i57_3_lut (.A(mic_latest[39]), .B(shift_register[55]), 
         .C(n23689), .Z(shift_register_95__N_2948[56])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i57_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i58_3_lut (.A(mic_latest[40]), .B(shift_register[56]), 
         .C(n23689), .Z(shift_register_95__N_2948[57])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i58_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i59_3_lut (.A(mic_latest[41]), .B(shift_register[57]), 
         .C(n23689), .Z(shift_register_95__N_2948[58])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i59_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i60_3_lut (.A(mic_latest[42]), .B(shift_register[58]), 
         .C(n23689), .Z(shift_register_95__N_2948[59])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i60_3_lut.init = 16'hcaca;
    LUT4 i5_4_lut (.A(bit_count[0]), .B(bit_count[5]), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i5_4_lut.init = 16'hfffe;
    LUT4 shift_register_95__I_0_19_i61_3_lut (.A(mic_latest[43]), .B(shift_register[59]), 
         .C(n23689), .Z(shift_register_95__N_2948[60])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i61_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i62_3_lut (.A(mic_latest[44]), .B(shift_register[60]), 
         .C(n23689), .Z(shift_register_95__N_2948[61])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i62_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i63_3_lut (.A(mic_latest[45]), .B(shift_register[61]), 
         .C(n23689), .Z(shift_register_95__N_2948[62])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i63_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i64_3_lut (.A(mic_latest[46]), .B(shift_register[62]), 
         .C(n23689), .Z(shift_register_95__N_2948[63])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i64_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i65_3_lut (.A(mic_latest[47]), .B(shift_register[63]), 
         .C(n23689), .Z(shift_register_95__N_2948[64])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i65_3_lut.init = 16'hcaca;
    LUT4 i9877_2_lut (.A(shift_register[64]), .B(n23689), .Z(shift_register_95__N_2948[65])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i9877_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i74_3_lut (.A(mic_latest[48]), .B(shift_register[72]), 
         .C(n23689), .Z(shift_register_95__N_2948[73])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i74_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i75_3_lut (.A(mic_latest[49]), .B(shift_register[73]), 
         .C(n23689), .Z(shift_register_95__N_2948[74])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i75_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i76_3_lut (.A(mic_latest[50]), .B(shift_register[74]), 
         .C(n23689), .Z(shift_register_95__N_2948[75])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i76_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i77_3_lut (.A(mic_latest[51]), .B(shift_register[75]), 
         .C(n23689), .Z(shift_register_95__N_2948[76])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i77_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i78_3_lut (.A(mic_latest[52]), .B(shift_register[76]), 
         .C(n23689), .Z(shift_register_95__N_2948[77])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i78_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i79_3_lut (.A(mic_latest[53]), .B(shift_register[77]), 
         .C(n23689), .Z(shift_register_95__N_2948[78])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i79_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i80_3_lut (.A(mic_latest[54]), .B(shift_register[78]), 
         .C(n23689), .Z(shift_register_95__N_2948[79])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i80_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i81_3_lut (.A(mic_latest[55]), .B(shift_register[79]), 
         .C(n23689), .Z(shift_register_95__N_2948[80])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i81_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i95 (.D(n15772), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[95])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i95.GSR = "DISABLED";
    FD1P3DX shift_register_i94 (.D(n15770), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[94])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i94.GSR = "DISABLED";
    FD1P3DX shift_register_i93 (.D(n15768), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[93])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i93.GSR = "DISABLED";
    FD1P3DX shift_register_i92 (.D(n15766), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[92])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i92.GSR = "DISABLED";
    FD1P3DX shift_register_i91 (.D(n15764), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[91])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i91.GSR = "DISABLED";
    FD1P3DX shift_register_i90 (.D(n15762), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[90])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i90.GSR = "DISABLED";
    FD1P3DX shift_register_i89 (.D(n15760), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[89])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i89.GSR = "DISABLED";
    FD1P3DX shift_register_i72 (.D(n15758), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[72])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i72.GSR = "DISABLED";
    FD1P3DX shift_register_i71 (.D(n15756), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[71])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i71.GSR = "DISABLED";
    FD1P3DX shift_register_i70 (.D(n15754), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[70])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i70.GSR = "DISABLED";
    FD1P3DX shift_register_i69 (.D(n15752), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[69])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i69.GSR = "DISABLED";
    FD1P3DX shift_register_i68 (.D(n15750), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[68])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i68.GSR = "DISABLED";
    FD1P3DX shift_register_i67 (.D(n15748), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[67])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i67.GSR = "DISABLED";
    FD1P3DX shift_register_i66 (.D(n15746), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[66])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i66.GSR = "DISABLED";
    FD1P3DX shift_register_i48 (.D(n15744), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[48])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i48.GSR = "DISABLED";
    FD1P3DX shift_register_i47 (.D(n15742), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[47])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i47.GSR = "DISABLED";
    FD1P3DX shift_register_i46 (.D(n15740), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[46])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i46.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i82_3_lut (.A(mic_latest[56]), .B(shift_register[80]), 
         .C(n23689), .Z(shift_register_95__N_2948[81])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i82_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i45 (.D(n15738), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[45])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i45.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i83_3_lut (.A(mic_latest[57]), .B(shift_register[81]), 
         .C(n23689), .Z(shift_register_95__N_2948[82])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i83_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i44 (.D(n15736), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[44])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i44.GSR = "DISABLED";
    FD1P3DX shift_register_i43 (.D(n15734), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[43])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i43.GSR = "DISABLED";
    FD1P3DX shift_register_i41 (.D(n15732), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[41])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i41.GSR = "DISABLED";
    FD1P3DX shift_register_i24 (.D(n15730), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i24.GSR = "DISABLED";
    FD1P3DX shift_register_i23 (.D(n15728), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[23])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i23.GSR = "DISABLED";
    FD1P3DX shift_register_i22 (.D(n15726), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[22])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3DX shift_register_i21 (.D(n15724), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[21])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3DX shift_register_i20 (.D(n15722), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[20])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3DX shift_register_i19 (.D(n15720), .SP(sck_N_3045_enable_101), 
            .CK(sck_N_3045), .CD(spi_mic_cs_n_c), .Q(shift_register[19])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i19.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i84_3_lut (.A(mic_latest[58]), .B(shift_register[82]), 
         .C(n23689), .Z(shift_register_95__N_2948[83])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i84_3_lut.init = 16'hcaca;
    FD1P3DX bit_count_i6 (.D(n15718), .SP(sck_N_3045_enable_101), .CK(sck_N_3045), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i6.GSR = "DISABLED";
    FD1P3DX bit_count_i5 (.D(n15716), .SP(sck_N_3045_enable_101), .CK(sck_N_3045), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i5.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i85_3_lut (.A(mic_latest[59]), .B(shift_register[83]), 
         .C(n23689), .Z(shift_register_95__N_2948[84])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i85_3_lut.init = 16'hcaca;
    FD1P3DX bit_count_i4 (.D(n15714), .SP(sck_N_3045_enable_101), .CK(sck_N_3045), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i4.GSR = "DISABLED";
    FD1P3DX bit_count_i3 (.D(n15712), .SP(sck_N_3045_enable_101), .CK(sck_N_3045), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i3.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i86_3_lut (.A(mic_latest[60]), .B(shift_register[84]), 
         .C(n23689), .Z(shift_register_95__N_2948[85])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i86_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i87_3_lut (.A(mic_latest[61]), .B(shift_register[85]), 
         .C(n23689), .Z(shift_register_95__N_2948[86])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i87_3_lut.init = 16'hcaca;
    FD1P3DX bit_count_i2 (.D(n15710), .SP(sck_N_3045_enable_101), .CK(sck_N_3045), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i2.GSR = "DISABLED";
    FD1P3DX bit_count_i1 (.D(n15708), .SP(sck_N_3045_enable_101), .CK(sck_N_3045), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=610, LSE_RLINE=613 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 shift_register_95__I_0_19_i88_3_lut (.A(mic_latest[62]), .B(shift_register[86]), 
         .C(n23689), .Z(shift_register_95__N_2948[87])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i88_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i89_3_lut (.A(mic_latest[63]), .B(shift_register[87]), 
         .C(n23689), .Z(shift_register_95__N_2948[88])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i89_3_lut.init = 16'hcaca;
    LUT4 i9880_3_lut_4_lut (.A(bit_count[3]), .B(n23690), .C(n23689), 
         .D(bit_count[4]), .Z(n15714)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9880_3_lut_4_lut.init = 16'h7080;
    LUT4 i9881_3_lut_4_lut (.A(bit_count[4]), .B(n23677), .C(n23689), 
         .D(bit_count[5]), .Z(n15716)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i9881_3_lut_4_lut.init = 16'h7080;
    
endmodule
//
// Verilog Description of module umh_toggle_ram84
//

module umh_toggle_ram84 (pll_clk, ev_we, VCC_net, GND_net, \ev_wr_addr[0] , 
            event_rd_addr, \ev_wr_addr[1] , \ev_wr_addr[2] , \ev_wr_addr[3] , 
            \ev_wr_addr[4] , \ev_wr_addr[5] , \ev_wr_addr[6] , \ev_wr_addr[7] , 
            n23720, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
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
    input n23720;
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23720), 
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
            .ADW8(n23720), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23720), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23720), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23720), 
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

module ws2812_stream (n15, state, ws2812_enable, pll_clk, \shift_register[17] , 
            \shift_register_23__N_2908[17] , \shift_register[18] , \shift_register_23__N_2908[18] , 
            \shift_register[19] , \shift_register_23__N_2908[19] , \shift_register[20] , 
            \shift_register_23__N_2908[20] , \shift_register[21] , \shift_register_23__N_2908[21] , 
            \shift_register[22] , \shift_register_23__N_2908[22] , \shift_register_23__N_2908[23] , 
            \shift_register[16] , \rgb_hold[0] , GND_net, rgb_data_c) /* synthesis syn_module_defined=1 */ ;
    output n15;
    output [1:0]state;
    input ws2812_enable;
    input pll_clk;
    output \shift_register[17] ;
    input \shift_register_23__N_2908[17] ;
    output \shift_register[18] ;
    input \shift_register_23__N_2908[18] ;
    output \shift_register[19] ;
    input \shift_register_23__N_2908[19] ;
    output \shift_register[20] ;
    input \shift_register_23__N_2908[20] ;
    output \shift_register[21] ;
    input \shift_register_23__N_2908[21] ;
    output \shift_register[22] ;
    input \shift_register_23__N_2908[22] ;
    input \shift_register_23__N_2908[23] ;
    output \shift_register[16] ;
    input \rgb_hold[0] ;
    input GND_net;
    output rgb_data_c;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    
    wire data_out_N_2945;
    wire [4:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(28[15:25])
    wire [4:0]n136;
    
    wire pll_clk_enable_461;
    wire [12:0]n22;
    
    wire n22912, n22910, n22911, n23676, pll_clk_enable_475, n22901, 
        n22904, n22905, n22906, n22907, n22908, n22902, n22903, 
        n22909, n14700;
    wire [1:0]state_1__N_2883;
    wire [12:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(26[16:27])
    
    wire pll_clk_enable_472, n22913, n22836, n19071, n19073, n1;
    wire [7:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(27[15:29])
    
    wire n22988, n19, n22872, n22873, pll_clk_enable_659;
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[16:30])
    
    wire pll_clk_enable_730;
    wire [7:0]n70;
    
    wire n15588, n14171, n23100, n14, n23110, n22805, n26_adj_3158, 
        n23671, n22974, n22885, pll_clk_enable_732, data_out_N_2939, 
        n9868, n6, n22252, n22251, n22250, n22249, n22248, n22247, 
        n22246, n22245, n22244, n22243, n23666, n23682;
    
    LUT4 i1573_2_lut_3_lut_4_lut (.A(data_out_N_2945), .B(n15), .C(bit_number[1]), 
         .D(bit_number[0]), .Z(n136[1])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1573_2_lut_3_lut_4_lut.init = 16'hd2f0;
    LUT4 i1_3_lut_4_lut (.A(n15), .B(state[1]), .C(ws2812_enable), .D(state[0]), 
         .Z(pll_clk_enable_461)) /* synthesis lut_function=(!(A (B+!(C (D)))+!A !(C (D)))) */ ;
    defparam i1_3_lut_4_lut.init = 16'h7000;
    LUT4 i1_2_lut_3_lut (.A(state[1]), .B(ws2812_enable), .C(n22[1]), 
         .Z(n22912)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_42 (.A(state[1]), .B(ws2812_enable), .C(n22[2]), 
         .Z(n22910)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_42.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_43 (.A(state[1]), .B(ws2812_enable), .C(n22[3]), 
         .Z(n22911)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_43.init = 16'h4040;
    LUT4 i14620_4_lut_4_lut (.A(state[1]), .B(n23676), .C(state[0]), .D(ws2812_enable), 
         .Z(pll_clk_enable_475)) /* synthesis lut_function=(!(A (B (D)+!B !(C+!(D)))+!A (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam i14620_4_lut_4_lut.init = 16'h25ff;
    LUT4 i1_2_lut_3_lut_adj_44 (.A(state[1]), .B(ws2812_enable), .C(n22[4]), 
         .Z(n22901)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_44.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_45 (.A(state[1]), .B(ws2812_enable), .C(n22[5]), 
         .Z(n22904)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_45.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_46 (.A(state[1]), .B(ws2812_enable), .C(n22[6]), 
         .Z(n22905)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_46.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_47 (.A(state[1]), .B(ws2812_enable), .C(n22[7]), 
         .Z(n22906)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_47.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_48 (.A(state[1]), .B(ws2812_enable), .C(n22[8]), 
         .Z(n22907)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_48.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_49 (.A(state[1]), .B(ws2812_enable), .C(n22[9]), 
         .Z(n22908)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_49.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_50 (.A(state[1]), .B(ws2812_enable), .C(n22[10]), 
         .Z(n22902)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_50.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_51 (.A(state[1]), .B(ws2812_enable), .C(n22[11]), 
         .Z(n22903)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_51.init = 16'h4040;
    LUT4 i1_2_lut_3_lut_adj_52 (.A(state[1]), .B(ws2812_enable), .C(n22[12]), 
         .Z(n22909)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_52.init = 16'h4040;
    LUT4 i20_1_lut (.A(ws2812_enable), .Z(n14700)) /* synthesis lut_function=(!(A)) */ ;
    defparam i20_1_lut.init = 16'h5555;
    FD1S3IX state__i0 (.D(state_1__N_2883[0]), .CK(pll_clk), .CD(n14700), 
            .Q(state[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam state__i0.GSR = "DISABLED";
    FD1P3AX reset_count__i0 (.D(n22913), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i0.GSR = "DISABLED";
    LUT4 i9822_4_lut (.A(n22836), .B(state[0]), .C(n19071), .D(n19073), 
         .Z(n1)) /* synthesis lut_function=(A (B)+!A (B+(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(39[13] 82[20])
    defparam i9822_4_lut.init = 16'hdccc;
    LUT4 i4_4_lut (.A(bit_cell_count[6]), .B(bit_cell_count[5]), .C(bit_cell_count[0]), 
         .D(n22988), .Z(n19)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i4_4_lut.init = 16'h0020;
    LUT4 i3_4_lut (.A(bit_cell_count[4]), .B(bit_cell_count[5]), .C(bit_cell_count[0]), 
         .D(n22872), .Z(n22873)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i3_4_lut.init = 16'h0400;
    FD1S3IX state__i1 (.D(state_1__N_2883[1]), .CK(pll_clk), .CD(n14700), 
            .Q(state[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam state__i1.GSR = "DISABLED";
    LUT4 i14624_4_lut (.A(ws2812_enable), .B(state[0]), .C(n23676), .D(state[1]), 
         .Z(pll_clk_enable_472)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (D)))) */ ;
    defparam i14624_4_lut.init = 16'h5d77;
    FD1P3AX shift_register_i0_i17 (.D(\shift_register_23__N_2908[17] ), .SP(pll_clk_enable_659), 
            .CK(pll_clk), .Q(\shift_register[17] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i18 (.D(\shift_register_23__N_2908[18] ), .SP(pll_clk_enable_659), 
            .CK(pll_clk), .Q(\shift_register[18] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i19 (.D(\shift_register_23__N_2908[19] ), .SP(pll_clk_enable_659), 
            .CK(pll_clk), .Q(\shift_register[19] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i20 (.D(\shift_register_23__N_2908[20] ), .SP(pll_clk_enable_659), 
            .CK(pll_clk), .Q(\shift_register[20] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i21 (.D(\shift_register_23__N_2908[21] ), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(\shift_register[21] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i22 (.D(\shift_register_23__N_2908[22] ), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(\shift_register[22] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i0_i23 (.D(\shift_register_23__N_2908[23] ), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i23.GSR = "DISABLED";
    FD1P3AX reset_count__i1 (.D(n22912), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i1.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut_adj_53 (.A(state[1]), .B(ws2812_enable), .C(n22[0]), 
         .Z(n22913)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_53.init = 16'h4040;
    FD1P3AX reset_count__i2 (.D(n22910), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i2.GSR = "DISABLED";
    FD1P3AX reset_count__i3 (.D(n22911), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i3.GSR = "DISABLED";
    FD1P3AX reset_count__i4 (.D(n22901), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i4.GSR = "DISABLED";
    FD1P3AX reset_count__i5 (.D(n22904), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i5.GSR = "DISABLED";
    FD1P3AX reset_count__i6 (.D(n22905), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i6.GSR = "DISABLED";
    FD1P3AX reset_count__i7 (.D(n22906), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i7.GSR = "DISABLED";
    FD1P3AX reset_count__i8 (.D(n22907), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i8.GSR = "DISABLED";
    FD1P3AX reset_count__i9 (.D(n22908), .SP(pll_clk_enable_472), .CK(pll_clk), 
            .Q(reset_count[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i9.GSR = "DISABLED";
    FD1P3AX reset_count__i10 (.D(n22902), .SP(pll_clk_enable_475), .CK(pll_clk), 
            .Q(reset_count[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i10.GSR = "DISABLED";
    FD1P3AX reset_count__i11 (.D(n22903), .SP(pll_clk_enable_475), .CK(pll_clk), 
            .Q(reset_count[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i11.GSR = "DISABLED";
    FD1P3AX reset_count__i12 (.D(n22909), .SP(pll_clk_enable_475), .CK(pll_clk), 
            .Q(reset_count[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam reset_count__i12.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i7 (.D(n70[7]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i7.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i6 (.D(n70[6]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i6.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i5 (.D(n70[5]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i5.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i4 (.D(n70[4]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i4.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i3 (.D(n70[3]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i3.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i2 (.D(n70[2]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i2.GSR = "DISABLED";
    FD1P3IX bit_cell_count_i0_i1 (.D(n70[1]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i1.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i0 (.D(n14171), .SP(pll_clk_enable_730), .CD(n15588), 
            .CK(pll_clk), .Q(bit_number[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i0.GSR = "DISABLED";
    LUT4 i7_4_lut (.A(n23100), .B(n14), .C(bit_cell_count[6]), .D(bit_cell_count[3]), 
         .Z(n15)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;
    defparam i7_4_lut.init = 16'hfdff;
    LUT4 i14316_2_lut (.A(bit_cell_count[7]), .B(bit_cell_count[4]), .Z(n23100)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14316_2_lut.init = 16'h8888;
    LUT4 i6_4_lut (.A(bit_cell_count[0]), .B(bit_cell_count[2]), .C(bit_cell_count[5]), 
         .D(bit_cell_count[1]), .Z(n14)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;
    defparam i6_4_lut.init = 16'hf7ff;
    LUT4 i1_4_lut (.A(bit_number[3]), .B(bit_number[1]), .C(n23110), .D(bit_number[0]), 
         .Z(data_out_N_2945)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i1_4_lut.init = 16'hbfff;
    LUT4 i14326_2_lut (.A(bit_number[2]), .B(bit_number[4]), .Z(n23110)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14326_2_lut.init = 16'h8888;
    LUT4 i4711_2_lut_rep_132 (.A(state[0]), .B(ws2812_enable), .Z(pll_clk_enable_730)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i4711_2_lut_rep_132.init = 16'h8888;
    LUT4 i6702_2_lut_3_lut (.A(state[0]), .B(ws2812_enable), .C(state[1]), 
         .Z(n15588)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i6702_2_lut_3_lut.init = 16'h0808;
    LUT4 i6719_2_lut_3_lut_4_lut (.A(state[0]), .B(ws2812_enable), .C(state[1]), 
         .D(n15), .Z(pll_clk_enable_659)) /* synthesis lut_function=(!(((C (D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i6719_2_lut_3_lut_4_lut.init = 16'h0888;
    LUT4 i2_3_lut_4_lut (.A(state[0]), .B(ws2812_enable), .C(state[1]), 
         .D(n15), .Z(n22805)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i2_3_lut_4_lut.init = 16'h0080;
    PFUMX i43 (.BLUT(n22873), .ALUT(n19), .C0(shift_register[23]), .Z(n26_adj_3158));
    FD1P3IX bit_number_i0_i4 (.D(n136[4]), .SP(pll_clk_enable_730), .CD(n15588), 
            .CK(pll_clk), .Q(bit_number[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i4.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i3 (.D(n136[3]), .SP(pll_clk_enable_730), .CD(n15588), 
            .CK(pll_clk), .Q(bit_number[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i3.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i2 (.D(n136[2]), .SP(pll_clk_enable_730), .CD(n15588), 
            .CK(pll_clk), .Q(bit_number[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i2.GSR = "DISABLED";
    FD1P3IX bit_number_i0_i1 (.D(n136[1]), .SP(pll_clk_enable_730), .CD(n15588), 
            .CK(pll_clk), .Q(bit_number[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_number_i0_i1.GSR = "DISABLED";
    FD1P3IX shift_register_i0_i16 (.D(\rgb_hold[0] ), .SP(pll_clk_enable_659), 
            .CD(n22805), .CK(pll_clk), .Q(\shift_register[16] )) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam shift_register_i0_i16.GSR = "DISABLED";
    LUT4 i1587_2_lut_3_lut_4_lut (.A(bit_number[1]), .B(n23671), .C(bit_number[3]), 
         .D(bit_number[2]), .Z(n136[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1587_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i2_4_lut (.A(reset_count[7]), .B(reset_count[3]), .C(reset_count[9]), 
         .D(reset_count[1]), .Z(n22836)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(42[25:52])
    defparam i2_4_lut.init = 16'hf7ff;
    LUT4 i2_4_lut_adj_54 (.A(state[0]), .B(n22974), .C(n15), .D(n22885), 
         .Z(pll_clk_enable_732)) /* synthesis lut_function=(A (((D)+!C)+!B)+!A !(B)) */ ;
    defparam i2_4_lut_adj_54.init = 16'hbb3b;
    LUT4 i14191_2_lut (.A(ws2812_enable), .B(state[1]), .Z(n22974)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14191_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_55 (.A(bit_cell_count[3]), .B(bit_cell_count[1]), 
         .C(n26_adj_3158), .D(bit_cell_count[7]), .Z(n22885)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i2_4_lut_adj_55.init = 16'h0020;
    LUT4 i9826_4_lut (.A(state[0]), .B(ws2812_enable), .C(data_out_N_2939), 
         .D(state[1]), .Z(n9868)) /* synthesis lut_function=(A (B (C+!(D)))+!A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam i9826_4_lut.init = 16'hc088;
    LUT4 i9825_2_lut (.A(data_out_N_2945), .B(n15), .Z(data_out_N_2939)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i9825_2_lut.init = 16'h2222;
    LUT4 i4_4_lut_adj_56 (.A(reset_count[5]), .B(reset_count[6]), .C(reset_count[0]), 
         .D(n6), .Z(n19071)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut_adj_56.init = 16'h8000;
    LUT4 i1_2_lut (.A(reset_count[8]), .B(reset_count[12]), .Z(n6)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 i3_4_lut_adj_57 (.A(reset_count[4]), .B(reset_count[2]), .C(reset_count[11]), 
         .D(reset_count[10]), .Z(n19073)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut_adj_57.init = 16'h8000;
    LUT4 n7598_bdd_4_lut (.A(n23676), .B(state[0]), .C(n1), .D(state[1]), 
         .Z(state_1__N_2883[0])) /* synthesis lut_function=(A (B (C+(D))+!B !((D)+!C))+!A !((D)+!C)) */ ;
    defparam n7598_bdd_4_lut.init = 16'h88f0;
    CCU2D add_1078_13 (.A0(reset_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22252), .S0(n22[11]), .S1(n22[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1078_13.INIT0 = 16'h5aaa;
    defparam add_1078_13.INIT1 = 16'h5aaa;
    defparam add_1078_13.INJECT1_0 = "NO";
    defparam add_1078_13.INJECT1_1 = "NO";
    CCU2D add_1078_11 (.A0(reset_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22251), .COUT(n22252), .S0(n22[9]), .S1(n22[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1078_11.INIT0 = 16'h5aaa;
    defparam add_1078_11.INIT1 = 16'h5aaa;
    defparam add_1078_11.INJECT1_0 = "NO";
    defparam add_1078_11.INJECT1_1 = "NO";
    CCU2D add_1078_9 (.A0(reset_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22250), .COUT(n22251), .S0(n22[7]), .S1(n22[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1078_9.INIT0 = 16'h5aaa;
    defparam add_1078_9.INIT1 = 16'h5aaa;
    defparam add_1078_9.INJECT1_0 = "NO";
    defparam add_1078_9.INJECT1_1 = "NO";
    CCU2D add_1078_7 (.A0(reset_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22249), .COUT(n22250), .S0(n22[5]), .S1(n22[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1078_7.INIT0 = 16'h5aaa;
    defparam add_1078_7.INIT1 = 16'h5aaa;
    defparam add_1078_7.INJECT1_0 = "NO";
    defparam add_1078_7.INJECT1_1 = "NO";
    CCU2D add_1078_5 (.A0(reset_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22248), .COUT(n22249), .S0(n22[3]), .S1(n22[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1078_5.INIT0 = 16'h5aaa;
    defparam add_1078_5.INIT1 = 16'h5aaa;
    defparam add_1078_5.INJECT1_0 = "NO";
    defparam add_1078_5.INJECT1_1 = "NO";
    CCU2D add_1078_3 (.A0(reset_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22247), .COUT(n22248), .S0(n22[1]), .S1(n22[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1078_3.INIT0 = 16'h5aaa;
    defparam add_1078_3.INIT1 = 16'h5aaa;
    defparam add_1078_3.INJECT1_0 = "NO";
    defparam add_1078_3.INJECT1_1 = "NO";
    CCU2D add_1078_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[0]), .B1(n19073), .C1(n22836), .D1(n19071), 
          .COUT(n22247), .S1(n22[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(43[26:61])
    defparam add_1078_1.INIT0 = 16'hF000;
    defparam add_1078_1.INIT1 = 16'h5955;
    defparam add_1078_1.INJECT1_0 = "NO";
    defparam add_1078_1.INJECT1_1 = "NO";
    CCU2D add_15_9 (.A0(bit_cell_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22246), .S0(n70[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_9.INIT0 = 16'h5aaa;
    defparam add_15_9.INIT1 = 16'h0000;
    defparam add_15_9.INJECT1_0 = "NO";
    defparam add_15_9.INJECT1_1 = "NO";
    CCU2D add_15_7 (.A0(bit_cell_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22245), .COUT(n22246), .S0(n70[5]), .S1(n70[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_7.INIT0 = 16'h5aaa;
    defparam add_15_7.INIT1 = 16'h5aaa;
    defparam add_15_7.INJECT1_0 = "NO";
    defparam add_15_7.INJECT1_1 = "NO";
    CCU2D add_15_5 (.A0(bit_cell_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22244), .COUT(n22245), .S0(n70[3]), .S1(n70[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_5.INIT0 = 16'h5aaa;
    defparam add_15_5.INIT1 = 16'h5aaa;
    defparam add_15_5.INJECT1_0 = "NO";
    defparam add_15_5.INJECT1_1 = "NO";
    CCU2D add_15_3 (.A0(bit_cell_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22243), .COUT(n22244), .S0(n70[1]), .S1(n70[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_3.INIT0 = 16'h5aaa;
    defparam add_15_3.INIT1 = 16'h5aaa;
    defparam add_15_3.INJECT1_0 = "NO";
    defparam add_15_3.INJECT1_1 = "NO";
    CCU2D add_15_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22243), .S1(n70[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(69[43:64])
    defparam add_15_1.INIT0 = 16'hF000;
    defparam add_15_1.INIT1 = 16'h5555;
    defparam add_15_1.INJECT1_0 = "NO";
    defparam add_15_1.INJECT1_1 = "NO";
    LUT4 i1594_3_lut_4_lut (.A(bit_number[2]), .B(n23666), .C(bit_number[3]), 
         .D(bit_number[4]), .Z(n136[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1594_3_lut_4_lut.init = 16'h7f80;
    LUT4 i1_2_lut_adj_58 (.A(bit_cell_count[6]), .B(bit_cell_count[2]), 
         .Z(n22872)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_58.init = 16'h4444;
    LUT4 i14205_2_lut (.A(bit_cell_count[4]), .B(bit_cell_count[2]), .Z(n22988)) /* synthesis lut_function=((B)+!A) */ ;
    defparam i14205_2_lut.init = 16'hdddd;
    FD1P3IX bit_cell_count_i0_i0 (.D(n70[0]), .SP(pll_clk_enable_730), .CD(pll_clk_enable_659), 
            .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam bit_cell_count_i0_i0.GSR = "DISABLED";
    FD1P3AX data_out_reg_46 (.D(n9868), .SP(pll_clk_enable_732), .CK(pll_clk), 
            .Q(rgb_data_c)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=597, LSE_RLINE=609 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(33[12] 84[8])
    defparam data_out_reg_46.GSR = "DISABLED";
    LUT4 i674_2_lut_rep_87 (.A(n15), .B(data_out_N_2945), .Z(n23676)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i674_2_lut_rep_87.init = 16'heeee;
    LUT4 i10168_2_lut_3_lut_4_lut (.A(n15), .B(data_out_N_2945), .C(state[0]), 
         .D(state[1]), .Z(state_1__N_2883[1])) /* synthesis lut_function=(A (C)+!A (B (C)+!B !((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i10168_2_lut_3_lut_4_lut.init = 16'he0f0;
    LUT4 i1580_2_lut_3_lut_4_lut (.A(bit_number[0]), .B(n23682), .C(bit_number[2]), 
         .D(bit_number[1]), .Z(n136[2])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(63[34] 66[28])
    defparam i1580_2_lut_3_lut_4_lut.init = 16'hd2f0;
    LUT4 i772_2_lut_rep_93 (.A(data_out_N_2945), .B(n15), .Z(n23682)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i772_2_lut_rep_93.init = 16'hdddd;
    LUT4 i1575_2_lut_rep_77_3_lut_4_lut (.A(data_out_N_2945), .B(n15), .C(bit_number[1]), 
         .D(bit_number[0]), .Z(n23666)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1575_2_lut_rep_77_3_lut_4_lut.init = 16'h2000;
    LUT4 i1_2_lut_3_lut_adj_59 (.A(data_out_N_2945), .B(n15), .C(bit_number[0]), 
         .Z(n14171)) /* synthesis lut_function=(A (B (C)+!B !(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1_2_lut_3_lut_adj_59.init = 16'hd2d2;
    LUT4 i1567_2_lut_rep_82_3_lut (.A(data_out_N_2945), .B(n15), .C(bit_number[0]), 
         .Z(n23671)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(67[30] 79[24])
    defparam i1567_2_lut_rep_82_3_lut.init = 16'h2020;
    
endmodule
//
// Verilog Description of module umh_channel_ram18
//

module umh_channel_ram18 (n10152, spi1_sck_c, spi_channel_index, staging_q, 
            pll_clk, rd_data_15__N_2647, n10142, n10140, n10150, n10148, 
            spi_write, VCC_net, GND_net, staging_rd_addr_6__N_901, spi1_mosi_c_0, 
            \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , \spi_rx_shift[3] , 
            \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , spi_phase_pending, 
            n10157, n10159, n10161, n10163, n10165, n10167, n10169, 
            n10171, n10173, n10175, n10177, n10179, n10181, n10183, 
            n10185, n10187, n10144, n10146, n7) /* synthesis syn_module_defined=1 */ ;
    output n10152;
    input spi1_sck_c;
    input [6:0]spi_channel_index;
    output [15:0]staging_q;
    input pll_clk;
    input [15:0]rd_data_15__N_2647;
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
    output n7;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(94[10:17])
    
    wire n14, n10;
    
    FD1S3AX mem_1389 (.D(spi_channel_index[6]), .CK(spi1_sck_c), .Q(n10152));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1389.GSR = "DISABLED";
    FD1S3AX rd_data_i0 (.D(rd_data_15__N_2647[0]), .CK(pll_clk), .Q(staging_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    FD1S3AX mem_1379 (.D(spi_channel_index[1]), .CK(spi1_sck_c), .Q(n10142));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1379.GSR = "DISABLED";
    FD1S3AX mem_1377 (.D(spi_channel_index[0]), .CK(spi1_sck_c), .Q(n10140));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1377.GSR = "DISABLED";
    FD1S3AX mem_1387 (.D(spi_channel_index[5]), .CK(spi1_sck_c), .Q(n10150));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1387.GSR = "DISABLED";
    FD1S3AX mem_1385 (.D(spi_channel_index[4]), .CK(spi1_sck_c), .Q(n10148));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1385.GSR = "DISABLED";
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
    FD1S3AX mem_1381 (.D(spi_channel_index[2]), .CK(spi1_sck_c), .Q(n10144));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1381.GSR = "DISABLED";
    FD1S3AX mem_1383 (.D(spi_channel_index[3]), .CK(spi1_sck_c), .Q(n10146));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1383.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_15__N_2647[1]), .CK(pll_clk), .Q(staging_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_15__N_2647[2]), .CK(pll_clk), .Q(staging_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_15__N_2647[3]), .CK(pll_clk), .Q(staging_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_15__N_2647[4]), .CK(pll_clk), .Q(staging_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_15__N_2647[5]), .CK(pll_clk), .Q(staging_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_15__N_2647[6]), .CK(pll_clk), .Q(staging_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_15__N_2647[7]), .CK(pll_clk), .Q(staging_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    FD1S3AX rd_data_i8 (.D(rd_data_15__N_2647[8]), .CK(pll_clk), .Q(staging_q[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i8.GSR = "DISABLED";
    FD1S3AX rd_data_i9 (.D(rd_data_15__N_2647[9]), .CK(pll_clk), .Q(staging_q[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i9.GSR = "DISABLED";
    FD1S3AX rd_data_i10 (.D(rd_data_15__N_2647[10]), .CK(pll_clk), .Q(staging_q[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i10.GSR = "DISABLED";
    FD1S3AX rd_data_i11 (.D(rd_data_15__N_2647[11]), .CK(pll_clk), .Q(staging_q[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i11.GSR = "DISABLED";
    FD1S3AX rd_data_i12 (.D(rd_data_15__N_2647[12]), .CK(pll_clk), .Q(staging_q[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i12.GSR = "DISABLED";
    FD1S3AX rd_data_i13 (.D(rd_data_15__N_2647[13]), .CK(pll_clk), .Q(staging_q[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i13.GSR = "DISABLED";
    FD1S3AX rd_data_i14 (.D(rd_data_15__N_2647[14]), .CK(pll_clk), .Q(staging_q[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i14.GSR = "DISABLED";
    FD1S3AX rd_data_i15 (.D(rd_data_15__N_2647[15]), .CK(pll_clk), .Q(staging_q[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=207, LSE_RLINE=210 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i15.GSR = "DISABLED";
    LUT4 i7_4_lut (.A(staging_q[6]), .B(n14), .C(n10), .D(staging_q[2]), 
         .Z(n7)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i6_4_lut (.A(staging_q[4]), .B(staging_q[7]), .C(staging_q[1]), 
         .D(staging_q[0]), .Z(n14)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i6_4_lut.init = 16'hfffe;
    LUT4 i2_2_lut (.A(staging_q[3]), .B(staging_q[5]), .Z(n10)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i2_2_lut.init = 16'heeee;
    
endmodule
