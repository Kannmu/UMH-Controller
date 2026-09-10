// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Thu Sep 10 20:16:53 2026
//
// Verilog Description of module umh_fpga_top
//

module umh_fpga_top (fpga_clk_8m, fpga_cs_n, spi1_sck, spi1_mosi, spi1_miso, 
            us_tx, rgb_data, mic_clk, mic_data_0, mic_data_1, spi_mic_cs_n, 
            spi_mic_sck, spi_mic_miso) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(58[8:20])
    input fpga_clk_8m;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(59[24:35])
    input fpga_cs_n;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(60[24:33])
    input spi1_sck;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:32])
    input spi1_mosi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:33])
    output spi1_miso;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(63[24:33])
    output [83:0]us_tx;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    output rgb_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:32])
    output mic_clk;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(66[24:31])
    input mic_data_0;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(67[24:34])
    input mic_data_1;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(68[24:34])
    input spi_mic_cs_n;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(69[24:36])
    input spi_mic_sck;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(70[24:35])
    output spi_mic_miso;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:36])
    
    wire fpga_clk_8m_c /* synthesis is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(59[24:35])
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:32])
    wire spi_mic_sck_c /* synthesis is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(70[24:35])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[10:17])
    wire spi1_sck_N_457 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(126[17:33])
    wire sck_N_2908 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
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
    wire [7:0]spi_rx_shift;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(117[17:29])
    wire [7:0]spi_command;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(117[31:42])
    wire [7:0]spi_version;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(117[44:55])
    wire [7:0]spi_phase_pending;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(117[57:74])
    wire [2:0]spi_bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(118[17:30])
    wire [15:0]spi_byte_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[17:31])
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[51:71])
    wire [31:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:35])
    
    wire spi1_sck_c_enable_74, n12976;
    wire [31:0]spi_expected_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[37:56])
    wire [31:0]accepted_sequence_spi;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[58:79])
    wire [6:0]spi_channel_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(121[17:34])
    wire [1:0]spi_channel_field;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(122[17:34])
    wire [87:0]spi_bitmap;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(123[17:27])
    
    wire frame_toggle_spi, stop_toggle_spi, invalid_frame_spi;
    wire [95:0]rgb_values;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(125[17:27])
    wire [6:0]status_bit_index;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(126[17:33])
    
    wire spi_write;
    wire [23:0]phase_frac;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(142[17:27])
    
    wire phase_step_reg;
    wire [7:0]global_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(144[17:29])
    wire [31:0]fpga_time;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(145[17:26])
    wire [6:0]time_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(146[17:29])
    
    wire phase_step_d1, phase_step_d2, phase_step_d3, swap_now_d1, swap_now_d2, 
        swap_now_d3;
    wire [8:0]run_addr_reg;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(153[17:29])
    wire [83:0]ev_run_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(154[17:28])
    wire [24:0]phase_frac_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[17:31])
    wire [7:0]next_global_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[17:34])
    wire [6:0]mic_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(161[17:28])
    
    wire mic_tick;
    wire [15:0]mic_shift_0_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(164[17:30])
    
    wire n37;
    wire [15:0]mic_shift_0_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(164[32:45])
    wire [15:0]mic_shift_1_l;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:30])
    
    wire n36;
    wire [15:0]mic_shift_1_r;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[32:45])
    wire [4:0]mic_sample_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(166[17:33])
    wire [63:0]mic_latest;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(167[17:27])
    wire [3:0]ev_state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(181[17:25])
    wire [7:0]ev_clear_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(182[17:30])
    wire [6:0]ev_ch;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(183[17:22])
    wire [83:0]ev_bit;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(184[17:23])
    wire [83:0]init_shadow;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(184[25:36])
    wire [7:0]build_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(185[17:28])
    wire [8:0]build_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(186[17:26])
    wire [6:0]staging_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(188[17:32])
    
    wire frame_req, swap_pending, active_bank;
    wire [15:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[17:26])
    wire [7:0]ev_rd_slot;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[17:27])
    
    wire swap_now, run_bank;
    wire [8:0]event_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[17:30])
    
    wire ev_we;
    wire [8:0]ev_wr_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(214[17:27])
    wire [83:0]ev_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(217[17:27])
    wire [83:0]ev_rd_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(218[17:27])
    wire [83:0]ev_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[17:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen, invalid_frame_meta, n12870;
    wire [31:0]accepted_sequence_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[17:39])
    wire [31:0]accepted_sequence_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[41:63])
    wire [31:0]pending_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[17:33])
    wire [31:0]accepted_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[35:52])
    wire [3:0]frame_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[17:29])
    wire [95:0]rgb_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[17:25])
    
    wire n35, n34, cs_meta, cs_sync, cs_sync_d;
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(243[18:29])
    
    wire cs_fall;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(251[17:33])
    wire [15:0]expected_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[17:30])
    
    wire frame_end, fpga_cs_n_N_2502;
    wire [8:0]ev_wr_addr_8__N_860;
    
    wire n22711;
    wire [15:0]status_flags_wire_15__N_1333;
    wire [15:0]status_flags_wire_15__N_1349;
    wire [6:0]spi1_miso_N_2465;
    
    wire n12876, pll_clk_enable_276;
    wire [15:0]expected_next_15__N_1406;
    wire [15:0]expected_next_15__N_1365;
    wire [31:0]frame_end_N_2544;
    wire [15:0]rd_data_15__N_2579;
    
    wire frame_end_N_2543, n17517, n14530, n14528, n14524, n60, 
        n12, n14801, n12924, n22670, n24, n63, n12772, n22025, 
        n40, n39, n38, n37_adj_3021, n36_adj_3022, n35_adj_3023, 
        n34_adj_3024, n12784, n12778, n22007, n8, n21990, n21982, 
        n5, n21981, n21989, n23568, n174, n12796, n12802, n12808, 
        n12814, n10, n22006, n22024, spi1_sck_c_enable_36;
    wire [15:0]spi_byte_count_15__N_1643;
    
    wire spi1_sck_c_enable_325, stop_toggle_spi_N_2487, frame_toggle_spi_N_2476, 
        spi1_sck_c_enable_26, n12790, n12960, n22039, n166, n23518, 
        n12_adj_3025, n44, pll_clk_enable_193, pll_clk_enable_566, pll_clk_enable_555, 
        n19458, n23517, n19130, n22022, n22069, n22038, n19122, 
        n22021, n22020, n22037, n22036, n22035, pll_clk_enable_449, 
        n36_adj_3026, n22005, n23516, n18996, spi1_sck_c_enable_271, 
        n23018, n22004, n22034, spi1_sck_c_enable_176, n55, n1, 
        spi1_sck_c_enable_323, n165, n98, n23515, n23514, n19396, 
        spi1_sck_c_enable_89, n23016, n6, n5_adj_3027, n9892, n18981, 
        pll_clk_enable_413, pll_clk_enable_322, pll_clk_enable_445, pll_clk_enable_77, 
        spi1_sck_c_enable_317, pll_clk_enable_162, n21988, n22725, n22099, 
        spi1_sck_c_enable_27, n22723, n23513, n23010, spi1_sck_c_enable_58, 
        n15, n14, n14778, n14781, n21971, n21888, n34_adj_3028, 
        n35_adj_3029, n36_adj_3030, n21891;
    wire [3:0]frame_settle_3__N_1924;
    
    wire n21970, n23006, n21980, n21969, n22033, n22032, n21890, 
        n23004, n21889, n22031, n23002, n22050, active_bank_N_859, 
        n12945, n9326, n22996;
    wire [7:0]ev_clear_addr_7__N_2189;
    wire [3:0]ev_state_3__N_1976;
    wire [8:0]build_sum_8__N_2005;
    wire [3:0]ev_state_3__N_1964;
    
    wire n22030;
    wire [6:0]ev_ch_6__N_1984;
    
    wire n21978;
    wire [3:0]ev_state_3__N_645;
    
    wire swap_pending_N_2519;
    wire [6:0]ev_ch_6__N_657;
    
    wire n22709, n23511, n21987, n22992, n22003, n22049, n22756, 
        n22048, n22002, n22001, n22988, n22000, n21986, n133, 
        n21975, n21979, n21977, n21999, n21976, n21998, n21974, 
        n22984, n21985, n21997, n22029, n22046, n22028, n22980, 
        n21996, n21995, n22008, n21994;
    wire [6:0]staging_rd_addr_6__N_849;
    
    wire n21984, n21993, n21992, n22045, n22974, n22972, n21983, 
        n21991, n22968, n22964, n22960, n22958, n22956, n22952, 
        n22948, n22944, n22942, spi1_sck_N_457_enable_7, n40_adj_3031, 
        n39_adj_3032, n38_adj_3033, n37_adj_3034, n36_adj_3035, n35_adj_3036, 
        n34_adj_3037, spi1_sck_c_enable_51, n7, n37_adj_3038, n22940, 
        n22938, n30, n29, n28, n27, n26, n38_adj_3039, n39_adj_3040, 
        n22934, n28_adj_3041, n16497, mic_tick_N_2507, n23493, n40_adj_3042, 
        n23054, n164, n163, n162, n161, n160, n159, n158, n157, 
        n156, n155, n154, n153, n152, n151, n150, n149, n148, 
        n147, n146, n145, n144, n143, n142, n141, n140, n139, 
        n138, n137, n136, n135, n21887, n14_adj_3043, n20, n134, 
        n19, n18, n38_adj_3044, n39_adj_3045, n40_adj_3046, pll_clk_enable_7, 
        n22044, n22926, mic_clk_N_2472, n22922, n22918, n5_adj_3047, 
        n9940, n9939, n9938, n9937, n9936, n9935, n9934, n9933, 
        n9932, n9931, n22916, n9930, n9929, n9928, n9927, n9926, 
        n9925, n9924, n9923, n9922, n9921, n9920, n9919, n9918, 
        n9917, n9916, n9915, n9914, n9913, n9912, n9911, n9910, 
        n9909, n9908, n9907, n9905, n9904, n9903, n9902, n9901, 
        n9900, n9899, n9898, n9897, n9896, n9895, n9894, n9893, 
        n23840, n12882, spi1_sck_c_enable_44, n12888, n12894, n12900, 
        n23838, n23087, n12906, n23086, n12912, n14_adj_3048, n23085, 
        n23084, n12864, spi1_sck_c_enable_66, n15793, n23083, n23082, 
        n23081, spi1_sck_c_enable_43, n23080, n22900, n23079, n23078, 
        n23077, n23076, n23075, n10_adj_3049, n22614, pll_clk_enable_565, 
        n23491, n23074, n23073, n22890, n23072, n22660, n23490, 
        n23506, spi1_sck_c_enable_28, n23071, n22888, n23070, n23069, 
        n4, n23068, n23067, n23066, n15876, n22884, n91, n23065, 
        n23064, n23063, n23062, n23061, n12858, n23060, n22876, 
        n12852, n23059, n23058, pll_clk_enable_574, n12846, n23056, 
        n23055, n12840, n23053, n12834, n22026, n12826, n17, n16, 
        n23052, n22724, n22667, pll_clk_enable_477, n23051, n23050, 
        n12820, n22721, n23836, n23049, pll_clk_enable_399, n23048, 
        n14345, n28_adj_3050, n14347, n14423, n14446, n23504, n23503, 
        n8_adj_3051, n23047, pll_clk_enable_511, n22854, n23046, n26_adj_3052, 
        n22852, n14_adj_3053, pll_clk_enable_600, n23045, n7_adj_3054, 
        n23044, n23043, n22842, n22840, n11598, n11596, n11594, 
        n11592, n11590, n8_adj_3055, n11588, n11586, n11584, n11582, 
        n11580, n11578, n11576, n11574, n11572, n11570, n11568, 
        n11566, n11564, n11562, n11560, n11558, n11556, n11554, 
        n7_adj_3056, n11552, n11550, n11548, n11546, n11544, n11542, 
        n11540, n11538, n23501, n11536, n11534, n11532, n11530, 
        n11528, n11526, n11524, n11522, n11520, pll_clk_enable_307, 
        n11518, n11516, n11514, n11512, n11510, n11508, n11506, 
        n11504, n11502, n11500, n11498, n11496, n11494, n11492, 
        n11490, n11488, n11486, n11484, n11482, n11480, n11478, 
        n11476, n11474, n11472, n11470, n11468, n11466, n11464, 
        n11462, n11460, n11458, n11456, n11454, n11452, n11450, 
        n11448, n11446, n11444, n11442, n11440, n11438, n11436, 
        n11434, n23042, n11417, n23041, n22681, n38_adj_3057, n14359, 
        n23040, n23039, n23038, n23037, n23036, n22693, n6_adj_3058, 
        n23567, n23035, n22830, n15010, n22739, n23565, n11144, 
        pll_clk_enable_242, n24_adj_3059, n14_adj_3060, pll_clk_enable_580, 
        n23034, n23033, n23324, n23032, n23031, spi1_sck_c_enable_82, 
        n22043, pll_clk_enable_128, spi1_sck_c_enable_225, n23030, n23029, 
        n23323, n22042, n23028, n10486, n20_adj_3061, n22810, n10_adj_3062, 
        n16_adj_3063, n23027, n22804, n4_adj_3064, n22802, n23026, 
        n12990, n13004, n23497, n13025, n13037, n13043, n13049, 
        n13055, n13061, n13067, n13073, n13079, n13085, n13095, 
        n13103, n13109, n13115, n13121, n13129, n13135, n13141, 
        n13147, n13153, n13159, n13165, n13171, n13177, n22792, 
        n13183, n13189, n13195, n13201, n23562, n13207, n13213, 
        n23561, n13223, n13231, n22041, n13241, n13247, n13253, 
        n13259, n13265, n13271, n13277, n13283, n13289, n13295, 
        n13301, n13307, n13313, n23560, n13319, n13325, n13331, 
        n13337, n13343, n13349, n23025, n13355, n13361, n23557, 
        n23494, n14425, n23556, n23555, n23554, n23553, n23564, 
        n23024, n23551, n22040, n14624, n14622, n23550, n23549, 
        n14616, n14612, n23548, n23547, spi1_sck_c_enable_326, n14608, 
        n23546, n14604, n14600, n23545, n14596, n5_adj_3065, n14592, 
        n23544, n7_adj_3066, n14588, n23543, n14580, n14427, n14378, 
        n23542, n23541, n14576, n23540, n23539, n14173, n14175, 
        pll_clk_enable_18, n23537, n14178, spi1_sck_c_enable_138, pll_clk_enable_17, 
        n14568, n23820, n23314, n23535, n14181, n23313, n14566, 
        n23534, n23533, n14185, n14404, n23023, n23532, n23531, 
        n15663, n23530, n14408, n14552, n23529, n14550, n15009, 
        n15008, n14383, n14546, n14385, n14387, n22772, n23528, 
        n14396, n14394, n23312, n5_adj_3067, n23526, n12_adj_3068, 
        n23569, pll_clk_enable_79, pll_clk_enable_19, pll_clk_enable_562, 
        n23523, n23522, spi1_sck_c_enable_286, n23520, n14390, n23519, 
        n13840, n15027, n23057, n13851;
    
    VHI i2 (.Z(VCC_net));
    INV i14933 (.A(spi_mic_sck_c), .Z(sck_N_2908));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(70[24:35])
    LUT4 i21_4_lut (.A(spi_rx_shift[5]), .B(n23004), .C(n36_adj_3026), 
         .D(n22756), .Z(n44)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i21_4_lut.init = 16'h0010;
    LUT4 i14443_4_lut (.A(spi_rx_shift[4]), .B(spi_extension_length[6]), 
         .C(spi_rx_shift[3]), .D(spi_extension_length[7]), .Z(n22964)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14443_4_lut.init = 16'hfffe;
    FD1S3IX ev_bit_i43 (.D(n14178), .CK(pll_clk), .CD(n23555), .Q(ev_bit[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i43.GSR = "DISABLED";
    LUT4 i2_4_lut_rep_157 (.A(n22667), .B(spi_byte_count[4]), .C(n55), 
         .D(n19122), .Z(spi1_sck_c_enable_138)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i2_4_lut_rep_157.init = 16'h0080;
    FD1S3AX mem_1413 (.D(staging_rd_addr_6__N_849[6]), .CK(pll_clk), .Q(n9905));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1413.GSR = "DISABLED";
    FD1S3AY cs_sync_360 (.D(cs_meta), .CK(pll_clk), .Q(cs_sync)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[12] 249[8])
    defparam cs_sync_360.GSR = "DISABLED";
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
    LUT4 i14419_4_lut (.A(spi_bitmap[12]), .B(spi_bitmap[24]), .C(spi_bitmap[13]), 
         .D(spi_bitmap[27]), .Z(n22940)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14419_4_lut.init = 16'h8000;
    LUT4 i14333_3_lut (.A(n22724), .B(spi1_mosi_c_0), .C(spi_extension_length[5]), 
         .Z(n22854)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i14333_3_lut.init = 16'hecec;
    LUT4 mux_1430_i16_3_lut (.A(n9939), .B(n9940), .C(n9908), .Z(rd_data_15__N_2579[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i16_3_lut.init = 16'hcaca;
    LUT4 i14483_4_lut (.A(spi_version[3]), .B(n22968), .C(n22876), .D(spi_version[4]), 
         .Z(n23004)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14483_4_lut.init = 16'hfffe;
    FD1S3AY cs_sync_d_361 (.D(cs_sync), .CK(pll_clk), .Q(cs_sync_d)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[12] 249[8])
    defparam cs_sync_d_361.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    LUT4 i3_4_lut (.A(spi_extension_length[4]), .B(spi_extension_length[2]), 
         .C(spi_extension_length[3]), .D(n22723), .Z(n22724)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[17:30])
    defparam i3_4_lut.init = 16'hfffe;
    LUT4 i1484_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23523), .C(ev_ch[5]), 
         .D(ev_ch[4]), .Z(ev_ch_6__N_1984[5])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1484_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i14397_4_lut (.A(spi_bitmap[64]), .B(spi_bitmap[25]), .C(spi_bitmap[19]), 
         .D(spi_bitmap[30]), .Z(n22918)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14397_4_lut.init = 16'h8000;
    LUT4 i1_4_lut (.A(ev_state[3]), .B(n16497), .C(ev_state[0]), .D(n12), 
         .Z(ev_state_3__N_645[0])) /* synthesis lut_function=(A (B)+!A (B+!(C+!(D)))) */ ;
    defparam i1_4_lut.init = 16'hcdcc;
    LUT4 i1_4_lut_adj_38 (.A(ev_state[1]), .B(n23539), .C(n19130), .D(ev_state[2]), 
         .Z(n12)) /* synthesis lut_function=(A+!(B (C (D))+!B (C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(181[17:25])
    defparam i1_4_lut_adj_38.init = 16'hafee;
    FD1S3IX ev_bit_i45 (.D(n14592), .CK(pll_clk), .CD(n23555), .Q(ev_bit[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i45.GSR = "DISABLED";
    FD1S3IX ev_bit_i38 (.D(n14175), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i38.GSR = "DISABLED";
    LUT4 i2_4_lut (.A(phase_step_d1), .B(phase_step_reg), .C(n22721), 
         .D(n23526), .Z(n19130)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i2_4_lut.init = 16'hfeee;
    FD1S3IX ev_bit_i10 (.D(n14546), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i10.GSR = "DISABLED";
    FD1S3IX ev_bit_i46 (.D(n14181), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i46.GSR = "DISABLED";
    FD1S3IX ev_bit_i75 (.D(n14552), .CK(pll_clk), .CD(n23555), .Q(ev_bit[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i75.GSR = "DISABLED";
    FD1S3IX ev_bit_i47 (.D(n14181), .CK(pll_clk), .CD(n23555), .Q(ev_bit[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i47.GSR = "DISABLED";
    LUT4 i2_4_lut_adj_39 (.A(n22693), .B(ev_state[1]), .C(ev_state[2]), 
         .D(ev_state_3__N_1964[2]), .Z(n16497)) /* synthesis lut_function=(!((B (C+(D))+!B (C))+!A)) */ ;
    defparam i2_4_lut_adj_39.init = 16'h020a;
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    FD1S3IX ev_bit_i49 (.D(n14596), .CK(pll_clk), .CD(n23555), .Q(ev_bit[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i49.GSR = "DISABLED";
    FD1S3IX ev_bit_i50 (.D(n14185), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i50.GSR = "DISABLED";
    FD1S3IX ev_bit_i51 (.D(n14185), .CK(pll_clk), .CD(n23555), .Q(ev_bit[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i51.GSR = "DISABLED";
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    FD1P3IX init_shadow_i16 (.D(n12864), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i16.GSR = "DISABLED";
    FD1S3IX ev_bit_i53 (.D(n14600), .CK(pll_clk), .CD(n23555), .Q(ev_bit[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i53.GSR = "DISABLED";
    FD1S3IX ev_bit_i54 (.D(n14404), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i54.GSR = "DISABLED";
    FD1S3IX ev_bit_i55 (.D(n14404), .CK(pll_clk), .CD(n23555), .Q(ev_bit[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i55.GSR = "DISABLED";
    FD1P3AX spi_update_flags_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1365[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_update_flags_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(expected_next[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i0.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i0.GSR = "DISABLED";
    LUT4 i14497_4_lut (.A(n22958), .B(n23010), .C(n22992), .D(n22956), 
         .Z(n23018)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14497_4_lut.init = 16'h8000;
    FD1S3IX ev_bit_i11 (.D(n14546), .CK(pll_clk), .CD(n23555), .Q(ev_bit[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i11.GSR = "DISABLED";
    FD1P3AX rgb_hold__i1 (.D(rgb_values[72]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i0 (.D(spi_byte_count_15__N_1643[0]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i0.GSR = "ENABLED";
    FD1S3IX frame_settle__i0 (.D(n15027), .CK(pll_clk), .CD(n11144), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    LUT4 i7_4_lut (.A(staging_q[0]), .B(n14_adj_3060), .C(n10_adj_3062), 
         .D(staging_q[6]), .Z(ev_state_3__N_1964[2])) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[32:56])
    defparam i7_4_lut.init = 16'hfffe;
    FD1S3IX ev_bit_i57 (.D(n14604), .CK(pll_clk), .CD(n23555), .Q(ev_bit[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i57.GSR = "DISABLED";
    FD1P3IX us_tx__i1 (.D(n10486), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i1.GSR = "DISABLED";
    FD1S3AX global_phase_i0 (.D(next_global_phase[0]), .CK(pll_clk), .Q(global_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i0.GSR = "DISABLED";
    LUT4 i13_4_lut (.A(spi_version[0]), .B(n18981), .C(n24), .D(spi_command[7]), 
         .Z(n36_adj_3026)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i13_4_lut.init = 16'h0002;
    LUT4 i6_4_lut (.A(staging_q[3]), .B(staging_q[1]), .C(staging_q[5]), 
         .D(staging_q[7]), .Z(n14_adj_3060)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[32:56])
    defparam i6_4_lut.init = 16'hfffe;
    FD1P3IX init_shadow_i15 (.D(n12858), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i15.GSR = "DISABLED";
    LUT4 i14235_2_lut (.A(spi_version[7]), .B(spi_command[1]), .Z(n22756)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14235_2_lut.init = 16'heeee;
    FD1S3IX ev_bit_i58 (.D(n14446), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i58.GSR = "DISABLED";
    FD1S3IX ev_bit_i5 (.D(n15009), .CK(pll_clk), .CD(n23555), .Q(ev_bit[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i5.GSR = "DISABLED";
    FD1S3AX phase_step_reg_383 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_reg)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_step_reg_383.GSR = "DISABLED";
    FD1S3AX phase_step_d1_385 (.D(phase_step_reg), .CK(pll_clk), .Q(phase_step_d1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_step_d1_385.GSR = "DISABLED";
    FD1S3AX phase_step_d2_386 (.D(phase_step_d1), .CK(pll_clk), .Q(phase_step_d2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_step_d2_386.GSR = "DISABLED";
    FD1S3AX phase_step_d3_387 (.D(phase_step_d2), .CK(pll_clk), .Q(phase_step_d3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_step_d3_387.GSR = "DISABLED";
    FD1S3AX swap_now_d1_388 (.D(swap_now), .CK(pll_clk), .Q(swap_now_d1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam swap_now_d1_388.GSR = "DISABLED";
    FD1S3AX swap_now_d2_389 (.D(swap_now_d1), .CK(pll_clk), .Q(swap_now_d2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam swap_now_d2_389.GSR = "DISABLED";
    FD1S3AX swap_now_d3_390 (.D(swap_now_d2), .CK(pll_clk), .Q(swap_now_d3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam swap_now_d3_390.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i0 (.D(ev_rd_data[0]), .CK(pll_clk), .Q(ev_run_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i0.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i0 (.D(accepted_sequence_spi[0]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i0.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_396 (.D(frame_toggle_spi), .CK(pll_clk), .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_toggle_meta_396.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut (.A(ev_state[1]), .B(n23513), .C(ev_state[3]), 
         .D(ev_state[2]), .Z(pll_clk_enable_580)) /* synthesis lut_function=(!(A ((D)+!C)+!A (B (D)+!B ((D)+!C)))) */ ;
    defparam i1_3_lut_4_lut.init = 16'h00f4;
    FD1S3AX frame_toggle_sync_397 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_toggle_sync_397.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_398 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam stop_toggle_meta_398.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_399 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam stop_toggle_sync_399.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i0 (.D(accepted_sequence_meta[0]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i0 (.D(mic_data_0_c), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i0.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_402 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam invalid_frame_meta_402.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_403 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(status_flags_wire_15__N_1333[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam invalid_frame_sync_403.GSR = "DISABLED";
    FD1P3IX frame_req_406 (.D(n23820), .SP(pll_clk_enable_77), .CD(n13840), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_req_406.GSR = "DISABLED";
    LUT4 i2_2_lut (.A(staging_q[2]), .B(staging_q[4]), .Z(n10_adj_3062)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[32:56])
    defparam i2_2_lut.init = 16'heeee;
    LUT4 i14447_4_lut (.A(spi_version[1]), .B(spi_rx_shift[6]), .C(spi_version[2]), 
         .D(spi_rx_shift[1]), .Z(n22968)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14447_4_lut.init = 16'hfffe;
    FD1P3AX swap_pending_410 (.D(swap_pending_N_2519), .SP(pll_clk_enable_7), 
            .CK(pll_clk), .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam swap_pending_410.GSR = "DISABLED";
    FD1P3AX ev_ch_i0 (.D(ev_ch_6__N_657[0]), .SP(pll_clk_enable_580), .CK(pll_clk), 
            .Q(ev_ch[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i0.GSR = "DISABLED";
    FD1P3AX build_phase_i0 (.D(staging_q[8]), .SP(pll_clk_enable_193), .CK(pll_clk), 
            .Q(build_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i0.GSR = "DISABLED";
    FD1P3AX build_sum_i0 (.D(build_sum_8__N_2005[0]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i0.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i0.GSR = "DISABLED";
    LUT4 i78_4_lut (.A(n133), .B(n22996), .C(n91), .D(n22888), .Z(n166)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;
    defparam i78_4_lut.init = 16'hfbff;
    FD1S3AX staging_rd_addr_i0 (.D(staging_rd_addr_6__N_849[0]), .CK(pll_clk), 
            .Q(staging_rd_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam staging_rd_addr_i0.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0 (.D(accepted_sequence_sync[0]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i1.GSR = "DISABLED";
    FD1S3AX mic_tick_425 (.D(mic_tick_N_2507), .CK(pll_clk), .Q(mic_tick)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_tick_425.GSR = "DISABLED";
    FD1S3AX mic_clock_reg_427 (.D(mic_clk_N_2472), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_clock_reg_427.GSR = "DISABLED";
    LUT4 i14395_4_lut (.A(spi_bitmap[47]), .B(spi_bitmap[66]), .C(spi_bitmap[71]), 
         .D(spi_bitmap[17]), .Z(n22916)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14395_4_lut.init = 16'h8000;
    FD1P3AX mic_shift_1_r__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    FD1S3AY cs_meta_359 (.D(fpga_cs_n_c), .CK(pll_clk), .Q(cs_meta)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(245[12] 249[8])
    defparam cs_meta_359.GSR = "DISABLED";
    LUT4 i14355_2_lut (.A(spi_rx_shift[2]), .B(spi_version[5]), .Z(n22876)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14355_2_lut.init = 16'heeee;
    FD1P3AX spi_expected_length_i0 (.D(expected_next[0]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i0.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i1 (.D(expected_next[1]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i1.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i2 (.D(expected_next[2]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i3 (.D(expected_next[3]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i4 (.D(expected_next[4]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i5 (.D(expected_next[5]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i6 (.D(expected_next[6]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i7 (.D(expected_next[7]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i8 (.D(expected_next[8]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i8.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i9 (.D(expected_next[9]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i9.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i10 (.D(expected_next[10]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i10.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i11 (.D(expected_next[11]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i11.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i12 (.D(expected_next[12]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i12.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i13 (.D(expected_next[13]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i13.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i14 (.D(expected_next[14]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i14.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i15 (.D(expected_next[15]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_expected_length_i15.GSR = "ENABLED";
    LUT4 n19130_bdd_3_lut_14736 (.A(n19130), .B(ev_state[1]), .C(ev_state[0]), 
         .Z(n23312)) /* synthesis lut_function=(!(A (B (C)+!B !(C)))) */ ;
    defparam n19130_bdd_3_lut_14736.init = 16'h7d7d;
    LUT4 i1_4_lut_4_lut_else_3_lut (.A(status_bit_index[1]), .B(status_hold[76]), 
         .C(status_bit_index[0]), .Z(n23567)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i1_4_lut_4_lut_else_3_lut.init = 16'h8080;
    LUT4 i10464_3_lut (.A(expected_next_15__N_1365[7]), .B(n23016), .C(n174), 
         .Z(n18981)) /* synthesis lut_function=(A ((C)+!B)) */ ;
    defparam i10464_3_lut.init = 16'ha2a2;
    LUT4 i14495_4_lut (.A(n22942), .B(n23006), .C(n22984), .D(n22940), 
         .Z(n23016)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14495_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_4_lut (.A(n23491), .B(spi_command[0]), .C(spi_command[1]), 
         .D(n22099), .Z(n5_adj_3067)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[30] 337[24])
    defparam i1_3_lut_4_lut_4_lut.init = 16'hfbff;
    LUT4 i45_4_lut (.A(spi_bitmap[84]), .B(spi_bitmap[87]), .C(spi_bitmap[14]), 
         .D(spi_bitmap[85]), .Z(n133)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i45_4_lut.init = 16'hffef;
    CCU2D mic_divider_1060_add_4_7 (.A0(mic_divider[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22046), .S0(n35_adj_3023), .S1(n34_adj_3024));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_divider_1060_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_divider_1060_add_4_7.INJECT1_0 = "NO";
    defparam mic_divider_1060_add_4_7.INJECT1_1 = "NO";
    LUT4 mux_930_i1_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[0]), 
         .D(build_phase[0]), .Z(ev_wr_addr_8__N_860[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14251_2_lut_4_lut (.A(n19122), .B(n23519), .C(n23534), .D(n23515), 
         .Z(n22772)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i14251_2_lut_4_lut.init = 16'hfffe;
    LUT4 i14475_4_lut (.A(spi_bitmap[82]), .B(n22960), .C(n22884), .D(spi_bitmap[7]), 
         .Z(n22996)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14475_4_lut.init = 16'h8000;
    FD1S3AX mem_1400 (.D(staging_rd_addr_6__N_849[0]), .CK(pll_clk), .Q(n9893));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1400.GSR = "DISABLED";
    LUT4 mux_930_i2_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[1]), 
         .D(build_phase[1]), .Z(ev_wr_addr_8__N_860[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14696_4_lut (.A(n23557), .B(status_bit_index[5]), .C(status_bit_index[3]), 
         .D(n6_adj_3058), .Z(spi1_sck_N_457_enable_7)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i14696_4_lut.init = 16'h7fff;
    LUT4 i1_2_lut (.A(status_bit_index[4]), .B(status_bit_index[6]), .Z(n6_adj_3058)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 mux_930_i3_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[2]), 
         .D(build_phase[2]), .Z(ev_wr_addr_8__N_860[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i3_2_lut (.A(spi_bitmap[86]), .B(spi_bitmap[62]), .Z(n91)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam i3_2_lut.init = 16'hbbbb;
    LUT4 mux_930_i4_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[3]), 
         .D(build_phase[3]), .Z(ev_wr_addr_8__N_860[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_930_i5_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[4]), 
         .D(build_phase[4]), .Z(ev_wr_addr_8__N_860[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i5_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i1_3_lut_4_lut_adj_40 (.A(n23494), .B(spi1_sck_c_enable_286), .C(n23517), 
         .D(spi_byte_count[0]), .Z(n22614)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_3_lut_4_lut_adj_40.init = 16'hffbf;
    LUT4 i14367_2_lut (.A(spi_bitmap[41]), .B(spi_bitmap[72]), .Z(n22888)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14367_2_lut.init = 16'h8888;
    LUT4 mux_930_i6_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[5]), 
         .D(build_phase[5]), .Z(ev_wr_addr_8__N_860[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i6_3_lut_4_lut.init = 16'hf2d0;
    LUT4 ev_state_3__I_0_448_Mux_1_i7_4_lut_4_lut (.A(ev_state_3__N_1976[1]), 
         .B(ev_state[1]), .C(ev_state[0]), .D(ev_state[2]), .Z(n7)) /* synthesis lut_function=(!(A (B (C (D))+!B !(C))+!A (B (C (D))+!B !(C (D))))) */ ;
    defparam ev_state_3__I_0_448_Mux_1_i7_4_lut_4_lut.init = 16'h3cec;
    LUT4 i6_4_lut_adj_41 (.A(time_divider[2]), .B(n12_adj_3068), .C(time_divider[6]), 
         .D(time_divider[1]), .Z(pll_clk_enable_555)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_41.init = 16'h8000;
    LUT4 i14439_4_lut (.A(spi_bitmap[81]), .B(spi_bitmap[54]), .C(spi_bitmap[40]), 
         .D(spi_bitmap[59]), .Z(n22960)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14439_4_lut.init = 16'h8000;
    LUT4 i3_4_lut_rep_101 (.A(n23522), .B(swap_pending), .C(frame_req), 
         .D(ev_state[0]), .Z(n23511)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_rep_101.init = 16'hfffe;
    LUT4 i5_4_lut (.A(time_divider[0]), .B(time_divider[5]), .C(time_divider[4]), 
         .D(time_divider[3]), .Z(n12_adj_3068)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut.init = 16'h8000;
    LUT4 i10932_1_lut_4_lut (.A(n23522), .B(swap_pending), .C(frame_req), 
         .D(ev_state[0]), .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;
    defparam i10932_1_lut_4_lut.init = 16'h0001;
    LUT4 mux_930_i7_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[6]), 
         .D(build_phase[6]), .Z(ev_wr_addr_8__N_860[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i7_3_lut_4_lut.init = 16'hf2d0;
    FD1S3IX ev_bit_i3 (.D(n14550), .CK(pll_clk), .CD(n23555), .Q(ev_bit[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i3.GSR = "DISABLED";
    FD1S3IX ev_bit_i59 (.D(n14446), .CK(pll_clk), .CD(n23555), .Q(ev_bit[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i59.GSR = "DISABLED";
    FD1S3IX ev_bit_i2 (.D(n14550), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i2.GSR = "DISABLED";
    FD1S3IX ev_bit_i61 (.D(n14608), .CK(pll_clk), .CD(n23555), .Q(ev_bit[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i61.GSR = "DISABLED";
    FD1S3IX ev_bit_i62 (.D(n14408), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i62.GSR = "DISABLED";
    FD1S3IX ev_bit_i63 (.D(n14408), .CK(pll_clk), .CD(n23555), .Q(ev_bit[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i63.GSR = "DISABLED";
    LUT4 i14363_2_lut (.A(spi_bitmap[9]), .B(spi_bitmap[55]), .Z(n22884)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14363_2_lut.init = 16'h8888;
    FD1P3AX stop_toggle_seen_408 (.D(stop_toggle_sync), .SP(pll_clk_enable_17), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam stop_toggle_seen_408.GSR = "DISABLED";
    LUT4 i13466_1_lut (.A(spi_bit_count[0]), .Z(n20)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam i13466_1_lut.init = 16'h5555;
    LUT4 i86_4_lut (.A(n22918), .B(n23018), .C(n166), .D(n22916), .Z(n174)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;
    defparam i86_4_lut.init = 16'hf7ff;
    LUT4 i14437_4_lut (.A(spi_bitmap[15]), .B(spi_bitmap[36]), .C(spi_bitmap[29]), 
         .D(spi_bitmap[39]), .Z(n22958)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14437_4_lut.init = 16'h8000;
    LUT4 i13488_1_lut (.A(mic_sample_count[0]), .Z(n30)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam i13488_1_lut.init = 16'h5555;
    LUT4 n19130_bdd_2_lut_14737_4_lut (.A(n23539), .B(ev_state_3__N_1976[1]), 
         .C(ev_state[0]), .D(ev_state[1]), .Z(n23313)) /* synthesis lut_function=(A (B+((D)+!C))+!A (B (C+(D))+!B (D))) */ ;
    defparam n19130_bdd_2_lut_14737_4_lut.init = 16'hffca;
    LUT4 i1_2_lut_rep_87_4_lut (.A(n23539), .B(ev_state_3__N_1976[1]), .C(ev_state[0]), 
         .D(ev_state[1]), .Z(n23497)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam i1_2_lut_rep_87_4_lut.init = 16'h00ca;
    FD1P3AX frame_toggle_seen_404 (.D(frame_toggle_sync), .SP(pll_clk_enable_18), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_toggle_seen_404.GSR = "DISABLED";
    LUT4 mux_930_i8_3_lut_4_lut (.A(ev_state[1]), .B(n23514), .C(build_sum[7]), 
         .D(build_phase[7]), .Z(ev_wr_addr_8__N_860[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam mux_930_i8_3_lut_4_lut.init = 16'hf2d0;
    FD1P3AX active_bank_412 (.D(active_bank_N_859), .SP(pll_clk_enable_19), 
            .CK(pll_clk), .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam active_bank_412.GSR = "DISABLED";
    LUT4 i14489_4_lut (.A(n22842), .B(n22988), .C(n22948), .D(n22840), 
         .Z(n23010)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14489_4_lut.init = 16'h8000;
    LUT4 i14471_4_lut (.A(spi_bitmap[79]), .B(n22952), .C(n22852), .D(spi_bitmap[4]), 
         .Z(n22992)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14471_4_lut.init = 16'h8000;
    FD1S3IX ev_bit_i65 (.D(n14378), .CK(pll_clk), .CD(n23555), .Q(ev_bit[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i65.GSR = "DISABLED";
    FD1S3IX ev_bit_i66 (.D(n14612), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i66.GSR = "DISABLED";
    FD1S3IX ev_bit_i67 (.D(n14612), .CK(pll_clk), .CD(n23555), .Q(ev_bit[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i67.GSR = "DISABLED";
    FD1S3IX ev_bit_i22 (.D(n14530), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i22.GSR = "DISABLED";
    FD1S3IX ev_bit_i69 (.D(n15008), .CK(pll_clk), .CD(n23555), .Q(ev_bit[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i69.GSR = "DISABLED";
    FD1S3IX ev_bit_i70 (.D(n14616), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i70.GSR = "DISABLED";
    FD1P3AX invalid_frame_spi_380 (.D(n23490), .SP(spi1_sck_c_enable_27), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam invalid_frame_spi_380.GSR = "DISABLED";
    FD1P3AX frame_toggle_spi_378 (.D(frame_toggle_spi_N_2476), .SP(spi1_sck_c_enable_28), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam frame_toggle_spi_378.GSR = "DISABLED";
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    FD1S3IX ev_bit_i71 (.D(n14616), .CK(pll_clk), .CD(n23555), .Q(ev_bit[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i71.GSR = "DISABLED";
    LUT4 i14435_4_lut (.A(spi_bitmap[58]), .B(spi_bitmap[69]), .C(spi_bitmap[63]), 
         .D(spi_bitmap[78]), .Z(n22956)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14435_4_lut.init = 16'h8000;
    LUT4 i14421_4_lut (.A(spi_bitmap[56]), .B(spi_bitmap[67]), .C(spi_bitmap[60]), 
         .D(spi_bitmap[6]), .Z(n22942)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14421_4_lut.init = 16'h8000;
    LUT4 i14485_4_lut (.A(n22804), .B(n22980), .C(n22926), .D(n22802), 
         .Z(n23006)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14485_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_4_lut (.A(spi_byte_count[2]), .B(n23543), .C(spi_byte_count[6]), 
         .D(spi_byte_count[5]), .Z(n38_adj_3057)) /* synthesis lut_function=(A (C+(D))+!A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_2_lut_3_lut_4_lut.init = 16'hfef0;
    FD1S3IX ev_bit_i20 (.D(n14423), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i20.GSR = "DISABLED";
    FD1S3IX ev_bit_i26 (.D(n14528), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i26.GSR = "DISABLED";
    FD1S3IX ev_bit_i73 (.D(n14383), .CK(pll_clk), .CD(n23555), .Q(ev_bit[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i73.GSR = "DISABLED";
    LUT4 i14431_4_lut (.A(spi_bitmap[33]), .B(spi_bitmap[49]), .C(spi_bitmap[46]), 
         .D(spi_bitmap[0]), .Z(n22952)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14431_4_lut.init = 16'h8000;
    FD1S3IX ev_bit_i13 (.D(n14359), .CK(pll_clk), .CD(n23555), .Q(ev_bit[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i13.GSR = "DISABLED";
    LUT4 i1_4_lut_rep_155 (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n23518), .Z(pll_clk_enable_600)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;
    defparam i1_4_lut_rep_155.init = 16'h5150;
    LUT4 i1_3_lut_4_lut_adj_42 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[17]), 
         .D(ev_bit[17]), .Z(ev_wr_data[17])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_42.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_43 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[70]), 
         .D(ev_bit[70]), .Z(ev_wr_data[70])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_43.init = 16'hddd0;
    LUT4 i14463_4_lut (.A(spi_bitmap[45]), .B(n22934), .C(n22810), .D(spi_bitmap[5]), 
         .Z(n22984)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14463_4_lut.init = 16'h8000;
    FD1S3IX ev_bit_i77 (.D(n14347), .CK(pll_clk), .CD(n23555), .Q(ev_bit[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i77.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_44 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[75]), 
         .D(ev_bit[75]), .Z(ev_wr_data[75])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_44.init = 16'hddd0;
    FD1S3IX ev_bit_i7 (.D(n14568), .CK(pll_clk), .CD(n23555), .Q(ev_bit[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i7.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_45 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[4]), 
         .D(ev_bit[4]), .Z(ev_wr_data[4])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_45.init = 16'hddd0;
    LUT4 i14331_2_lut (.A(spi_bitmap[68]), .B(spi_bitmap[16]), .Z(n22852)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14331_2_lut.init = 16'h8888;
    FD1S3IX ev_bit_i78 (.D(n14524), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i78.GSR = "DISABLED";
    LUT4 i14321_2_lut (.A(spi_bitmap[65]), .B(spi_bitmap[76]), .Z(n22842)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14321_2_lut.init = 16'h8888;
    FD1S3IX ev_bit_i79 (.D(n14524), .CK(pll_clk), .CD(n23555), .Q(ev_bit[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i79.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_46 (.A(spi_byte_count[2]), .B(n23535), .C(spi_byte_count[5]), 
         .D(spi_byte_count[4]), .Z(n14778)) /* synthesis lut_function=(!(A+((C (D)+!C !(D))+!B))) */ ;
    defparam i1_3_lut_4_lut_adj_46.init = 16'h0440;
    LUT4 i1_3_lut_4_lut_adj_47 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[6]), 
         .D(ev_bit[6]), .Z(ev_wr_data[6])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_47.init = 16'hddd0;
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    LUT4 i13511_3_lut_4_lut (.A(mic_sample_count[2]), .B(n23562), .C(mic_sample_count[3]), 
         .D(mic_sample_count[4]), .Z(n26)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam i13511_3_lut_4_lut.init = 16'h7f80;
    FD1P3IX running_409 (.D(n23820), .SP(swap_now_d1), .CD(pll_clk_enable_17), 
            .CK(pll_clk), .Q(status_flags_wire_15__N_1349[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam running_409.GSR = "DISABLED";
    FD1S3AX mem (.D(spi_phase_pending[7]), .CK(spi1_sck_c), .Q(n9940));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem.GSR = "DISABLED";
    FD1S3IX ev_bit_i9 (.D(n14385), .CK(pll_clk), .CD(n23555), .Q(ev_bit[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i9.GSR = "DISABLED";
    LUT4 i14467_4_lut (.A(spi_bitmap[20]), .B(n22944), .C(n22830), .D(spi_bitmap[26]), 
         .Z(n22988)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14467_4_lut.init = 16'h8000;
    LUT4 i14682_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23549), .C(n23551), 
         .D(ev_ch[4]), .Z(n15010)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14682_2_lut_2_lut_3_lut_4_lut.init = 16'h0001;
    FD1S3AX mem_1430 (.D(spi_phase_pending[6]), .CK(spi1_sck_c), .Q(n9938));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1430.GSR = "DISABLED";
    FD1S3AX mem_1429 (.D(spi_phase_pending[5]), .CK(spi1_sck_c), .Q(n9936));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1429.GSR = "DISABLED";
    OB spi1_miso_pad (.I(spi1_miso_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(63[24:33])
    FD1S3AX mem_1428 (.D(spi_phase_pending[4]), .CK(spi1_sck_c), .Q(n9934));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1428.GSR = "DISABLED";
    FD1S3AX mem_1427 (.D(spi_phase_pending[3]), .CK(spi1_sck_c), .Q(n9932));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1427.GSR = "DISABLED";
    FD1S3AX mem_1426 (.D(spi_phase_pending[2]), .CK(spi1_sck_c), .Q(n9930));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1426.GSR = "DISABLED";
    FD1S3AX mem_1425 (.D(spi_phase_pending[1]), .CK(spi1_sck_c), .Q(n9928));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1425.GSR = "DISABLED";
    FD1S3AX mem_1424 (.D(spi_phase_pending[0]), .CK(spi1_sck_c), .Q(n9926));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1424.GSR = "DISABLED";
    FD1S3IX ev_bit_i15 (.D(n14622), .CK(pll_clk), .CD(n23555), .Q(ev_bit[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i15.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_48 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[9]), 
         .D(ev_bit[9]), .Z(ev_wr_data[9])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_48.init = 16'hddd0;
    FD1S3AX mem_1423 (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n9924));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1423.GSR = "DISABLED";
    FD1S3AX mem_1422 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n9922));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1422.GSR = "DISABLED";
    FD1S3AX mem_1421 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n9920));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1421.GSR = "DISABLED";
    FD1P3AX status_hold__i1 (.D(accepted_sequence[24]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i1.GSR = "DISABLED";
    FD1S3AX mem_1420 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n9918));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1420.GSR = "DISABLED";
    FD1S3AX mem_1419 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n9916));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1419.GSR = "DISABLED";
    FD1S3AX mem_1418 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n9914));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1418.GSR = "DISABLED";
    LUT4 i14427_4_lut (.A(spi_bitmap[42]), .B(spi_bitmap[83]), .C(spi_bitmap[75]), 
         .D(spi_bitmap[10]), .Z(n22948)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14427_4_lut.init = 16'h8000;
    FD1S3IX ev_bit_i81 (.D(n14345), .CK(pll_clk), .CD(n23555), .Q(ev_bit[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i81.GSR = "DISABLED";
    FD1S3AX mem_1417 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n9912));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1417.GSR = "DISABLED";
    FD1S3IX ev_bit_i17 (.D(n14387), .CK(pll_clk), .CD(n23555), .Q(ev_bit[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i17.GSR = "DISABLED";
    FD1S3IX ev_bit_i82 (.D(n14566), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i82.GSR = "DISABLED";
    FD1S3IX ev_bit_i18 (.D(n14624), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i18.GSR = "DISABLED";
    FD1S3AX mem_1416 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n9910));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1416.GSR = "DISABLED";
    LUT4 i14319_2_lut (.A(spi_bitmap[32]), .B(spi_bitmap[37]), .Z(n22840)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14319_2_lut.init = 16'h8888;
    FD1S3AX mem_1415 (.D(spi_write), .CK(pll_clk), .Q(n9907));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1415.GSR = "DISABLED";
    LUT4 i14423_4_lut (.A(spi_bitmap[53]), .B(spi_bitmap[74]), .C(spi_bitmap[61]), 
         .D(spi_bitmap[38]), .Z(n22944)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14423_4_lut.init = 16'h8000;
    FD1S3AX mem_1402 (.D(staging_rd_addr_6__N_849[1]), .CK(pll_clk), .Q(n9895));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1402.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i0 (.D(pending_sequence[0]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i0.GSR = "DISABLED";
    FD1S3AX mem_1404 (.D(staging_rd_addr_6__N_849[2]), .CK(pll_clk), .Q(n9897));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1404.GSR = "DISABLED";
    FD1S3IX ev_bit_i14 (.D(n14622), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i14.GSR = "DISABLED";
    FD1S3AX mem_1406 (.D(staging_rd_addr_6__N_849[3]), .CK(pll_clk), .Q(n9899));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1406.GSR = "DISABLED";
    FD1P3IX init_shadow_i7 (.D(n12808), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i7.GSR = "DISABLED";
    FD1S3AX mem_1408 (.D(staging_rd_addr_6__N_849[4]), .CK(pll_clk), .Q(n9901));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1408.GSR = "DISABLED";
    FD1S3AX mem_1410 (.D(staging_rd_addr_6__N_849[5]), .CK(pll_clk), .Q(n9903));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mem_1410.GSR = "DISABLED";
    FD1S3IX ev_bit_i27 (.D(n14528), .CK(pll_clk), .CD(n23555), .Q(ev_bit[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i27.GSR = "DISABLED";
    FD1S3IX ev_bit_i83 (.D(n14566), .CK(pll_clk), .CD(n23555), .Q(ev_bit[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i83.GSR = "DISABLED";
    FD1S3IX ev_bit_i19 (.D(n14624), .CK(pll_clk), .CD(n23555), .Q(ev_bit[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i19.GSR = "DISABLED";
    FD1S3IX ev_bit_i6 (.D(n14568), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i6.GSR = "DISABLED";
    FD1S3IX ev_bit_i24 (.D(n14425), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i24.GSR = "DISABLED";
    FD1S3IX ev_bit_i23 (.D(n14530), .CK(pll_clk), .CD(n23555), .Q(ev_bit[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i23.GSR = "DISABLED";
    FD1S3IX ev_bit_i39 (.D(n14175), .CK(pll_clk), .CD(n23555), .Q(ev_bit[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i39.GSR = "DISABLED";
    LUT4 mux_1430_i11_3_lut (.A(n9929), .B(n9930), .C(n9908), .Z(rd_data_15__N_2579[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i11_3_lut.init = 16'hcaca;
    LUT4 i14309_2_lut (.A(spi_bitmap[31]), .B(spi_bitmap[51]), .Z(n22830)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14309_2_lut.init = 16'h8888;
    FD1S3IX ev_bit_i29 (.D(n14427), .CK(pll_clk), .CD(n23555), .Q(ev_bit[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i29.GSR = "DISABLED";
    FD1S3IX ev_bit_i30 (.D(n14576), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i30.GSR = "DISABLED";
    FD1S3IX ev_bit_i31 (.D(n14576), .CK(pll_clk), .CD(n23555), .Q(ev_bit[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i31.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_49 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[12]), 
         .D(ev_bit[12]), .Z(ev_wr_data[12])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_49.init = 16'hddd0;
    FD1S3IX ev_bit_i33 (.D(n14580), .CK(pll_clk), .CD(n23555), .Q(ev_bit[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i33.GSR = "DISABLED";
    FD1S3IX ev_bit_i1 (.D(n14390), .CK(pll_clk), .CD(n23555), .Q(ev_bit[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i1.GSR = "DISABLED";
    FD1S3IX ev_bit_i74 (.D(n14552), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i74.GSR = "DISABLED";
    FD1S3IX ev_bit_i34 (.D(n14173), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i34.GSR = "DISABLED";
    FD1S3IX ev_bit_i35 (.D(n14173), .CK(pll_clk), .CD(n23555), .Q(ev_bit[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i35.GSR = "DISABLED";
    LUT4 i14652_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23549), .C(n23528), 
         .D(ev_ch[4]), .Z(n15008)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14652_2_lut_2_lut_3_lut_4_lut.init = 16'h0001;
    LUT4 i14616_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23549), .C(n23546), 
         .D(ev_ch[4]), .Z(n15009)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14616_2_lut_2_lut_3_lut_4_lut.init = 16'h0001;
    PFUMX i14817 (.BLUT(n23564), .ALUT(n23565), .C0(ev_state[0]), .Z(n15663));
    LUT4 i14283_2_lut (.A(spi_bitmap[80]), .B(spi_bitmap[18]), .Z(n22804)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14283_2_lut.init = 16'h8888;
    LUT4 i14625_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[4]), .B(n23551), .C(n23549), 
         .D(ev_ch[3]), .Z(n14600)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i14625_2_lut_2_lut_3_lut_4_lut.init = 16'h0002;
    LUT4 i14628_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[4]), .B(n23551), .C(ev_ch[3]), 
         .D(n23544), .Z(n14604)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i14628_2_lut_2_lut_3_lut_4_lut.init = 16'h0020;
    LUT4 i1_3_lut_4_lut_adj_50 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[15]), 
         .D(ev_bit[15]), .Z(ev_wr_data[15])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_50.init = 16'hddd0;
    FD1P3IX init_shadow_i6 (.D(n12802), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i6.GSR = "DISABLED";
    LUT4 i5231_2_lut_3_lut_4_lut (.A(ev_state[0]), .B(n23539), .C(pll_clk_enable_17), 
         .D(n23522), .Z(n13840)) /* synthesis lut_function=(A (C)+!A (B (C+!(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i5231_2_lut_3_lut_4_lut.init = 16'hf0f4;
    LUT4 i14459_4_lut (.A(spi_bitmap[57]), .B(n22922), .C(n22792), .D(spi_bitmap[77]), 
         .Z(n22980)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14459_4_lut.init = 16'h8000;
    FD1S3IX ev_bit_i37 (.D(n15010), .CK(pll_clk), .CD(n23555), .Q(ev_bit[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i37.GSR = "DISABLED";
    FD1S3IX ev_bit_i68 (.D(n15008), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i68.GSR = "DISABLED";
    FD1S3IX ev_bit_i40 (.D(n14588), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i40.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_51 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[16]), 
         .D(ev_bit[16]), .Z(ev_wr_data[16])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_51.init = 16'hddd0;
    LUT4 i14405_4_lut (.A(spi_bitmap[34]), .B(spi_bitmap[50]), .C(spi_bitmap[35]), 
         .D(spi_bitmap[52]), .Z(n22926)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14405_4_lut.init = 16'h8000;
    LUT4 i14281_2_lut (.A(spi_bitmap[48]), .B(spi_bitmap[28]), .Z(n22802)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14281_2_lut.init = 16'h8888;
    FD1P3AX spi_channel_index_1054__i0 (.D(n40_adj_3046), .SP(spi1_sck_c_enable_323), 
            .CK(spi1_sck_c), .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054__i0.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_52 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[5]), 
         .D(ev_bit[5]), .Z(ev_wr_data[5])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_52.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_53 (.A(build_sum[8]), .B(n23529), .C(ev_bit[63]), 
         .D(init_shadow[63]), .Z(n13241)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_53.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_54 (.A(build_sum[8]), .B(n23529), .C(ev_bit[58]), 
         .D(init_shadow[58]), .Z(n13201)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_54.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_55 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[18]), 
         .D(ev_bit[18]), .Z(ev_wr_data[18])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_55.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_56 (.A(build_sum[8]), .B(n23529), .C(ev_bit[46]), 
         .D(init_shadow[46]), .Z(n13129)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_56.init = 16'hff80;
    LUT4 i2_3_lut (.A(spi_byte_count[8]), .B(fpga_cs_n_c), .C(expected_next_15__N_1365[7]), 
         .Z(n22711)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam i2_3_lut.init = 16'h1010;
    LUT4 i1_3_lut_4_lut_adj_57 (.A(build_sum[8]), .B(n23529), .C(ev_bit[44]), 
         .D(init_shadow[44]), .Z(n13115)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_57.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_58 (.A(build_sum[8]), .B(n23529), .C(ev_bit[12]), 
         .D(init_shadow[12]), .Z(n12840)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_58.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_59 (.A(build_sum[8]), .B(n23529), .C(ev_bit[9]), 
         .D(init_shadow[9]), .Z(n12820)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_59.init = 16'hff80;
    FD1P3AX status_bit_index_1053__i0 (.D(n40_adj_3042), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053__i0.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_60 (.A(build_sum[8]), .B(n23529), .C(ev_bit[41]), 
         .D(init_shadow[41]), .Z(n13095)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_60.init = 16'hff80;
    FD1P3AX fpga_time_1057__i0 (.D(n165), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i0.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_61 (.A(build_sum[8]), .B(n23529), .C(ev_bit[40]), 
         .D(init_shadow[40]), .Z(n13085)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_61.init = 16'hff80;
    LUT4 i14401_4_lut (.A(spi_bitmap[23]), .B(spi_bitmap[2]), .C(spi_bitmap[1]), 
         .D(spi_bitmap[11]), .Z(n22922)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14401_4_lut.init = 16'h8000;
    FD1S3IX mic_divider_1060__i0 (.D(n40), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060__i0.GSR = "DISABLED";
    LUT4 i14271_2_lut (.A(spi_bitmap[3]), .B(spi_bitmap[8]), .Z(n22792)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14271_2_lut.init = 16'h8888;
    LUT4 i14413_4_lut (.A(spi_bitmap[43]), .B(spi_bitmap[70]), .C(spi_bitmap[21]), 
         .D(spi_bitmap[73]), .Z(n22934)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14413_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_62 (.A(build_sum[8]), .B(n23529), .C(ev_bit[39]), 
         .D(init_shadow[39]), .Z(n13079)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_62.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_63 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[7]), 
         .D(ev_bit[7]), .Z(ev_wr_data[7])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_63.init = 16'hddd0;
    FD1S3AX spi_bit_count_1056__i0 (.D(n20), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam spi_bit_count_1056__i0.GSR = "ENABLED";
    FD1P3AX mic_sample_count_1059__i0 (.D(n30), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam mic_sample_count_1059__i0.GSR = "DISABLED";
    CCU2D global_phase_7__I_0_445_4 (.A0(global_phase[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21969), .COUT(n21970), .S0(next_global_phase[2]), 
          .S1(next_global_phase[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[37:56])
    defparam global_phase_7__I_0_445_4.INIT0 = 16'h5aaa;
    defparam global_phase_7__I_0_445_4.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_445_4.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_445_4.INJECT1_1 = "NO";
    LUT4 i14289_2_lut (.A(spi_bitmap[22]), .B(spi_bitmap[44]), .Z(n22810)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14289_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_64 (.A(build_sum[8]), .B(n23529), .C(ev_bit[37]), 
         .D(init_shadow[37]), .Z(n13067)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_64.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_65 (.A(build_sum[8]), .B(n23529), .C(ev_bit[35]), 
         .D(init_shadow[35]), .Z(n13055)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_65.init = 16'hff80;
    FD1S3AX time_divider_1058__i0 (.D(n40_adj_3031), .CK(pll_clk), .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058__i0.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_66 (.A(build_sum[8]), .B(n23529), .C(ev_bit[34]), 
         .D(init_shadow[34]), .Z(n13049)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_66.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_67 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[19]), 
         .D(ev_bit[19]), .Z(ev_wr_data[19])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_67.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_68 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[20]), 
         .D(ev_bit[20]), .Z(ev_wr_data[20])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_68.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_69 (.A(build_sum[8]), .B(n23529), .C(ev_bit[33]), 
         .D(init_shadow[33]), .Z(n13043)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_69.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_70 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[21]), 
         .D(ev_bit[21]), .Z(ev_wr_data[21])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_70.init = 16'hddd0;
    CCU2D add_499_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21994), .COUT(n21995), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_11.INIT0 = 16'h5555;
    defparam add_499_11.INIT1 = 16'h5aaa;
    defparam add_499_11.INJECT1_0 = "NO";
    defparam add_499_11.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_71 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[22]), 
         .D(ev_bit[22]), .Z(ev_wr_data[22])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_71.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_72 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[23]), 
         .D(ev_bit[23]), .Z(ev_wr_data[23])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_72.init = 16'hddd0;
    CCU2D add_135_5 (.A0(spi_byte_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21979), .COUT(n21980), .S0(spi_byte_count_15__N_1643[3]), 
          .S1(spi_byte_count_15__N_1643[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_5.INIT0 = 16'h5aaa;
    defparam add_135_5.INIT1 = 16'h5aaa;
    defparam add_135_5.INJECT1_0 = "NO";
    defparam add_135_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_73 (.A(build_sum[8]), .B(n23529), .C(ev_bit[32]), 
         .D(init_shadow[32]), .Z(n13037)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_73.init = 16'hff80;
    LUT4 frame_end_I_0_4_lut (.A(n22725), .B(frame_end_N_2543), .C(n7_adj_3054), 
         .D(n8_adj_3051), .Z(frame_end)) /* synthesis lut_function=(A (B (C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[29] 277[95])
    defparam frame_end_I_0_4_lut.init = 16'hccc5;
    LUT4 i2_4_lut_adj_74 (.A(n22667), .B(spi_byte_count[4]), .C(n55), 
         .D(n19122), .Z(spi1_sck_c_enable_176)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i2_4_lut_adj_74.init = 16'h0080;
    LUT4 i1_3_lut_4_lut_adj_75 (.A(build_sum[8]), .B(n23529), .C(ev_bit[31]), 
         .D(init_shadow[31]), .Z(n13025)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_75.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_76 (.A(build_sum[8]), .B(n23529), .C(ev_bit[30]), 
         .D(init_shadow[30]), .Z(n13004)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_76.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_77 (.A(build_sum[8]), .B(n23529), .C(ev_bit[29]), 
         .D(init_shadow[29]), .Z(n12990)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_77.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_78 (.A(build_sum[8]), .B(n23529), .C(ev_bit[28]), 
         .D(init_shadow[28]), .Z(n12976)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_78.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_79 (.A(build_sum[8]), .B(n23529), .C(ev_bit[27]), 
         .D(init_shadow[27]), .Z(n12960)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_79.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_80 (.A(build_sum[8]), .B(n23529), .C(ev_bit[26]), 
         .D(init_shadow[26]), .Z(n12945)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_80.init = 16'hff80;
    CCU2D status_bit_index_1053_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22024), .S1(n40_adj_3042));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053_add_4_1.INIT0 = 16'hF000;
    defparam status_bit_index_1053_add_4_1.INIT1 = 16'h0555;
    defparam status_bit_index_1053_add_4_1.INJECT1_0 = "NO";
    defparam status_bit_index_1053_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_81 (.A(build_sum[8]), .B(n23529), .C(ev_bit[25]), 
         .D(init_shadow[25]), .Z(n12924)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_81.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_82 (.A(build_sum[8]), .B(n23529), .C(ev_bit[24]), 
         .D(init_shadow[24]), .Z(n12912)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_82.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_83 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[24]), 
         .D(ev_bit[24]), .Z(ev_wr_data[24])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_83.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_84 (.A(build_sum[8]), .B(n23529), .C(ev_bit[23]), 
         .D(init_shadow[23]), .Z(n12906)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_84.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_85 (.A(build_sum[8]), .B(n23529), .C(ev_bit[22]), 
         .D(init_shadow[22]), .Z(n12900)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_85.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_86 (.A(build_sum[8]), .B(n23529), .C(ev_bit[21]), 
         .D(init_shadow[21]), .Z(n12894)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_86.init = 16'hff80;
    LUT4 i14_4_lut (.A(n22723), .B(n28_adj_3050), .C(n24_adj_3059), .D(n16_adj_3063), 
         .Z(n22725)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[48:79])
    defparam i14_4_lut.init = 16'hfffe;
    CCU2D add_501_8 (.A0(staging_q[14]), .B0(staging_q[6]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[15]), .B1(staging_q[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n21976), .COUT(n21977), .S0(build_sum_8__N_2005[6]), 
          .S1(build_sum_8__N_2005[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_501_8.INIT0 = 16'h5666;
    defparam add_501_8.INIT1 = 16'h5666;
    defparam add_501_8.INJECT1_0 = "NO";
    defparam add_501_8.INJECT1_1 = "NO";
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    LUT4 i1_3_lut_4_lut_adj_87 (.A(build_sum[8]), .B(n23529), .C(ev_bit[20]), 
         .D(init_shadow[20]), .Z(n12888)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_87.init = 16'hff80;
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    FD1P3IX init_shadow_i14 (.D(n12852), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i14.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_88 (.A(build_sum[8]), .B(n23529), .C(ev_bit[19]), 
         .D(init_shadow[19]), .Z(n12882)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_88.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_89 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[10]), 
         .D(ev_bit[10]), .Z(ev_wr_data[10])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_89.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_90 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[25]), 
         .D(ev_bit[25]), .Z(ev_wr_data[25])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_90.init = 16'hddd0;
    FD1P3IX ev_clear_addr_i7 (.D(ev_clear_addr_7__N_2189[7]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i7.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i6 (.D(ev_clear_addr_7__N_2189[6]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i6.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i5 (.D(ev_clear_addr_7__N_2189[5]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i5.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i4 (.D(ev_clear_addr_7__N_2189[4]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i4.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_91 (.A(build_sum[8]), .B(n23529), .C(ev_bit[18]), 
         .D(init_shadow[18]), .Z(n12876)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_91.init = 16'hff80;
    PFUMX i14554 (.BLUT(n23063), .ALUT(n23064), .C0(n23556), .Z(n23076));
    LUT4 i1_3_lut_4_lut_adj_92 (.A(build_sum[8]), .B(n23529), .C(ev_bit[17]), 
         .D(init_shadow[17]), .Z(n12870)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_92.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_93 (.A(build_sum[8]), .B(n23529), .C(ev_bit[16]), 
         .D(init_shadow[16]), .Z(n12864)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_93.init = 16'hff80;
    CCU2D add_135_3 (.A0(spi_byte_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21978), .COUT(n21979), .S0(spi_byte_count_15__N_1643[1]), 
          .S1(spi_byte_count_15__N_1643[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_3.INIT0 = 16'h5aaa;
    defparam add_135_3.INIT1 = 16'h5aaa;
    defparam add_135_3.INJECT1_0 = "NO";
    defparam add_135_3.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_94 (.A(build_sum[8]), .B(n23529), .C(ev_bit[15]), 
         .D(init_shadow[15]), .Z(n12858)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_94.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_95 (.A(build_sum[8]), .B(n23529), .C(ev_bit[7]), 
         .D(init_shadow[7]), .Z(n12808)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_95.init = 16'hff80;
    CCU2D add_499_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21993), .COUT(n21994), .S0(phase_frac_sum[7]), .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_9.INIT0 = 16'h5555;
    defparam add_499_9.INIT1 = 16'h5aaa;
    defparam add_499_9.INJECT1_0 = "NO";
    defparam add_499_9.INJECT1_1 = "NO";
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_96 (.A(build_sum[8]), .B(n23529), .C(ev_bit[6]), 
         .D(init_shadow[6]), .Z(n12802)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_96.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_97 (.A(build_sum[8]), .B(n23529), .C(ev_bit[14]), 
         .D(init_shadow[14]), .Z(n12852)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_97.init = 16'hff80;
    LUT4 i13_4_lut_adj_98 (.A(expected_next[2]), .B(n26_adj_3052), .C(n20_adj_3061), 
         .D(expected_next[7]), .Z(n28_adj_3050)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[48:79])
    defparam i13_4_lut_adj_98.init = 16'hfffd;
    LUT4 i1_3_lut_4_lut_adj_99 (.A(build_sum[8]), .B(n23529), .C(ev_bit[13]), 
         .D(init_shadow[13]), .Z(n12846)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_99.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_100 (.A(build_sum[8]), .B(n23529), .C(ev_bit[5]), 
         .D(init_shadow[5]), .Z(n12796)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_100.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_101 (.A(build_sum[8]), .B(n23529), .C(ev_bit[4]), 
         .D(init_shadow[4]), .Z(n12790)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_101.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_102 (.A(build_sum[8]), .B(n23529), .C(ev_bit[3]), 
         .D(init_shadow[3]), .Z(n12784)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_102.init = 16'hff80;
    PFUMX i14553 (.BLUT(n23061), .ALUT(n23062), .C0(n23556), .Z(n23075));
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_74 (.I(us_tx_c_74), .O(us_tx[74]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_73 (.I(us_tx_c_73), .O(us_tx[73]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_72 (.I(us_tx_c_72), .O(us_tx[72]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_71 (.I(us_tx_c_71), .O(us_tx[71]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_70 (.I(us_tx_c_70), .O(us_tx[70]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_69 (.I(us_tx_c_69), .O(us_tx[69]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_68 (.I(us_tx_c_68), .O(us_tx[68]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_67 (.I(us_tx_c_67), .O(us_tx[67]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_66 (.I(us_tx_c_66), .O(us_tx[66]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_65 (.I(us_tx_c_65), .O(us_tx[65]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_64 (.I(us_tx_c_64), .O(us_tx[64]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_63 (.I(us_tx_c_63), .O(us_tx[63]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_62 (.I(us_tx_c_62), .O(us_tx[62]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_61 (.I(us_tx_c_61), .O(us_tx[61]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_60 (.I(us_tx_c_60), .O(us_tx[60]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_59 (.I(us_tx_c_59), .O(us_tx[59]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_58 (.I(us_tx_c_58), .O(us_tx[58]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_57 (.I(us_tx_c_57), .O(us_tx[57]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_56 (.I(us_tx_c_56), .O(us_tx[56]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_55 (.I(us_tx_c_55), .O(us_tx[55]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_54 (.I(us_tx_c_54), .O(us_tx[54]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_53 (.I(us_tx_c_53), .O(us_tx[53]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_52 (.I(us_tx_c_52), .O(us_tx[52]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_51 (.I(us_tx_c_51), .O(us_tx[51]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_50 (.I(us_tx_c_50), .O(us_tx[50]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_49 (.I(us_tx_c_49), .O(us_tx[49]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_48 (.I(us_tx_c_48), .O(us_tx[48]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_47 (.I(us_tx_c_47), .O(us_tx[47]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_46 (.I(us_tx_c_46), .O(us_tx[46]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_45 (.I(us_tx_c_45), .O(us_tx[45]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_44 (.I(us_tx_c_44), .O(us_tx[44]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_43 (.I(us_tx_c_43), .O(us_tx[43]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_42 (.I(us_tx_c_42), .O(us_tx[42]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_41 (.I(us_tx_c_41), .O(us_tx[41]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_40 (.I(us_tx_c_40), .O(us_tx[40]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_39 (.I(us_tx_c_39), .O(us_tx[39]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_38 (.I(us_tx_c_38), .O(us_tx[38]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_37 (.I(us_tx_c_37), .O(us_tx[37]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_36 (.I(us_tx_c_36), .O(us_tx[36]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_35 (.I(us_tx_c_35), .O(us_tx[35]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_34 (.I(us_tx_c_34), .O(us_tx[34]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_33 (.I(us_tx_c_33), .O(us_tx[33]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_32 (.I(us_tx_c_32), .O(us_tx[32]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_31 (.I(us_tx_c_31), .O(us_tx[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_30 (.I(us_tx_c_30), .O(us_tx[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_29 (.I(us_tx_c_29), .O(us_tx[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_28 (.I(us_tx_c_28), .O(us_tx[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_27 (.I(us_tx_c_27), .O(us_tx[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_26 (.I(us_tx_c_26), .O(us_tx[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_25 (.I(us_tx_c_25), .O(us_tx[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_24 (.I(us_tx_c_24), .O(us_tx[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_23 (.I(us_tx_c_23), .O(us_tx[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_22 (.I(us_tx_c_22), .O(us_tx[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_21 (.I(us_tx_c_21), .O(us_tx[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_20 (.I(us_tx_c_20), .O(us_tx[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_19 (.I(us_tx_c_19), .O(us_tx[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_18 (.I(us_tx_c_18), .O(us_tx[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_17 (.I(us_tx_c_17), .O(us_tx[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_16 (.I(us_tx_c_16), .O(us_tx[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_15 (.I(us_tx_c_15), .O(us_tx[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_14 (.I(us_tx_c_14), .O(us_tx[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_13 (.I(us_tx_c_13), .O(us_tx[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_12 (.I(us_tx_c_12), .O(us_tx[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_11 (.I(us_tx_c_11), .O(us_tx[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_10 (.I(us_tx_c_10), .O(us_tx[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_9 (.I(us_tx_c_9), .O(us_tx[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_8 (.I(us_tx_c_8), .O(us_tx[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_7 (.I(us_tx_c_7), .O(us_tx[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_6 (.I(us_tx_c_6), .O(us_tx[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_5 (.I(us_tx_c_5), .O(us_tx[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_4 (.I(us_tx_c_4), .O(us_tx[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_3 (.I(us_tx_c_3), .O(us_tx[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_2 (.I(us_tx_c_2), .O(us_tx[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_1 (.I(us_tx_c_1), .O(us_tx[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_0 (.I(us_tx_c_0), .O(us_tx[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB rgb_data_pad (.I(rgb_data_c), .O(rgb_data));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(65[24:32])
    OB mic_clk_pad (.I(mic_clk_c), .O(mic_clk));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(66[24:31])
    OB spi_mic_miso_pad (.I(spi_mic_miso_c), .O(spi_mic_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(71[24:36])
    IB fpga_clk_8m_pad (.I(fpga_clk_8m), .O(fpga_clk_8m_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(59[24:35])
    IB fpga_cs_n_pad (.I(fpga_cs_n), .O(fpga_cs_n_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(60[24:33])
    IB spi1_sck_pad (.I(spi1_sck), .O(spi1_sck_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:32])
    IB spi1_mosi_pad (.I(spi1_mosi), .O(spi1_mosi_c_0));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(62[24:33])
    IB mic_data_0_pad (.I(mic_data_0), .O(mic_data_0_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(67[24:34])
    IB mic_data_1_pad (.I(mic_data_1), .O(mic_data_1_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(68[24:34])
    IB spi_mic_cs_n_pad (.I(spi_mic_cs_n), .O(spi_mic_cs_n_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(69[24:36])
    IB spi_mic_sck_pad (.I(spi_mic_sck), .O(spi_mic_sck_c));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(70[24:35])
    LUT4 run_addr_reg_8__I_0_i1_3_lut (.A(global_phase[0]), .B(ev_rd_slot[0]), 
         .C(n19130), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i1_3_lut.init = 16'hacac;
    FD1P3IX init_shadow_i13 (.D(n12846), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i13.GSR = "DISABLED";
    LUT4 i9_4_lut (.A(expected_next[13]), .B(expected_next[6]), .C(expected_next[3]), 
         .D(expected_next[4]), .Z(n24_adj_3059)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[48:79])
    defparam i9_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_4_lut_adj_103 (.A(build_sum[8]), .B(n23529), .C(ev_bit[0]), 
         .D(init_shadow[0]), .Z(n11417)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_103.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_104 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[27]), 
         .D(ev_bit[27]), .Z(ev_wr_data[27])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_104.init = 16'hddd0;
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_update_flags_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1406[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_update_flags_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(expected_next[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_105 (.A(build_sum[8]), .B(n23529), .C(ev_bit[2]), 
         .D(init_shadow[2]), .Z(n12778)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_105.init = 16'hff80;
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_106 (.A(build_sum[8]), .B(n23529), .C(ev_bit[83]), 
         .D(init_shadow[83]), .Z(n13361)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_106.init = 16'hff80;
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_frame_sequence_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_phase_pending_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i8 (.D(spi_bitmap[0]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i9 (.D(spi_bitmap[1]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i10 (.D(spi_bitmap[2]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i11 (.D(spi_bitmap[3]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i12 (.D(spi_bitmap[4]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i13 (.D(spi_bitmap[5]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i14 (.D(spi_bitmap[6]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i15 (.D(spi_bitmap[7]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i16 (.D(spi_bitmap[8]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i17 (.D(spi_bitmap[9]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i18 (.D(spi_bitmap[10]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i19 (.D(spi_bitmap[11]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i20 (.D(spi_bitmap[12]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i21 (.D(spi_bitmap[13]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i22 (.D(spi_bitmap[14]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i23 (.D(spi_bitmap[15]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i24 (.D(spi_bitmap[16]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i25 (.D(spi_bitmap[17]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i26 (.D(spi_bitmap[18]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i27 (.D(spi_bitmap[19]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i28 (.D(spi_bitmap[20]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i29 (.D(spi_bitmap[21]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i30 (.D(spi_bitmap[22]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i31 (.D(spi_bitmap[23]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i32 (.D(spi_bitmap[24]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i32.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i33 (.D(spi_bitmap[25]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i33.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i34 (.D(spi_bitmap[26]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i34.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i35 (.D(spi_bitmap[27]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i35.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i36 (.D(spi_bitmap[28]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i36.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i37 (.D(spi_bitmap[29]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i37.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i38 (.D(spi_bitmap[30]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i38.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i39 (.D(spi_bitmap[31]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i39.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i40 (.D(spi_bitmap[32]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i40.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i41 (.D(spi_bitmap[33]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i41.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i42 (.D(spi_bitmap[34]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i42.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i43 (.D(spi_bitmap[35]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i43.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i44 (.D(spi_bitmap[36]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i44.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i45 (.D(spi_bitmap[37]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i45.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i46 (.D(spi_bitmap[38]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i46.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i47 (.D(spi_bitmap[39]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i47.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i48 (.D(spi_bitmap[40]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i48.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i49 (.D(spi_bitmap[41]), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .Q(spi_bitmap[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i49.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i50 (.D(spi_bitmap[42]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i50.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i51 (.D(spi_bitmap[43]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i51.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i52 (.D(spi_bitmap[44]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i52.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i53 (.D(spi_bitmap[45]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i53.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i54 (.D(spi_bitmap[46]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i54.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i55 (.D(spi_bitmap[47]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i55.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i56 (.D(spi_bitmap[48]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i56.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i57 (.D(spi_bitmap[49]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i57.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i58 (.D(spi_bitmap[50]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i58.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i59 (.D(spi_bitmap[51]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i59.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i60 (.D(spi_bitmap[52]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i60.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i61 (.D(spi_bitmap[53]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i61.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i62 (.D(spi_bitmap[54]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i62.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i63 (.D(spi_bitmap[55]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i63.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i64 (.D(spi_bitmap[56]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i64.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i65 (.D(spi_bitmap[57]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i65.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i66 (.D(spi_bitmap[58]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i66.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i67 (.D(spi_bitmap[59]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i67.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i68 (.D(spi_bitmap[60]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i68.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i69 (.D(spi_bitmap[61]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i69.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i70 (.D(spi_bitmap[62]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i70.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i71 (.D(spi_bitmap[63]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i71.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i72 (.D(spi_bitmap[64]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i72.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i73 (.D(spi_bitmap[65]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i73.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i74 (.D(spi_bitmap[66]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i74.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i75 (.D(spi_bitmap[67]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i75.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i76 (.D(spi_bitmap[68]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i76.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i77 (.D(spi_bitmap[69]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i77.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i78 (.D(spi_bitmap[70]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i78.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i79 (.D(spi_bitmap[71]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i79.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i80 (.D(spi_bitmap[72]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i80.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i81 (.D(spi_bitmap[73]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i81.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i82 (.D(spi_bitmap[74]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i82.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i83 (.D(spi_bitmap[75]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i83.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i84 (.D(spi_bitmap[76]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i84.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i85 (.D(spi_bitmap[77]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i85.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i86 (.D(spi_bitmap[78]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i86.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i87 (.D(spi_bitmap[79]), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .Q(spi_bitmap[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_bitmap_i0_i87.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i1.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i2.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i3.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i4.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i5.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i6.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i7.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i8 (.D(rgb_values[0]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i8.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i9 (.D(rgb_values[1]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i9.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i10 (.D(rgb_values[2]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i10.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i11 (.D(rgb_values[3]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i11.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i12 (.D(rgb_values[4]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i12.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i13 (.D(rgb_values[5]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i13.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i14 (.D(rgb_values[6]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i14.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i15 (.D(rgb_values[7]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i15.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i16 (.D(rgb_values[8]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i16.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i17 (.D(rgb_values[9]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i17.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i18 (.D(rgb_values[10]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i18.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i19 (.D(rgb_values[11]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i19.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i20 (.D(rgb_values[12]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i20.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i21 (.D(rgb_values[13]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i21.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i22 (.D(rgb_values[14]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i22.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i23 (.D(rgb_values[15]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i23.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i24 (.D(rgb_values[16]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i24.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i25 (.D(rgb_values[17]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i25.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i26 (.D(rgb_values[18]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i26.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i27 (.D(rgb_values[19]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i27.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i28 (.D(rgb_values[20]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i28.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i29 (.D(rgb_values[21]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i29.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i30 (.D(rgb_values[22]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i30.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i31 (.D(rgb_values[23]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i31.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i32 (.D(rgb_values[24]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i32.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i33 (.D(rgb_values[25]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i33.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i34 (.D(rgb_values[26]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i34.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i35 (.D(rgb_values[27]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i35.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i36 (.D(rgb_values[28]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i36.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i37 (.D(rgb_values[29]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i37.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i38 (.D(rgb_values[30]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i38.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i39 (.D(rgb_values[31]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i39.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i40 (.D(rgb_values[32]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i40.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i41 (.D(rgb_values[33]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i41.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i42 (.D(rgb_values[34]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i42.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i43 (.D(rgb_values[35]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i43.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i44 (.D(rgb_values[36]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i44.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i45 (.D(rgb_values[37]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i45.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i46 (.D(rgb_values[38]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i46.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i47 (.D(rgb_values[39]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i47.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i48 (.D(rgb_values[40]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i48.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i49 (.D(rgb_values[41]), .SP(spi1_sck_c_enable_225), 
            .CK(spi1_sck_c), .Q(rgb_values[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i49.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i50 (.D(rgb_values[42]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i50.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i51 (.D(rgb_values[43]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i51.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i52 (.D(rgb_values[44]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i52.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i53 (.D(rgb_values[45]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i53.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i54 (.D(rgb_values[46]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i54.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i55 (.D(rgb_values[47]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i55.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i56 (.D(rgb_values[48]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i56.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i57 (.D(rgb_values[49]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i57.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i58 (.D(rgb_values[50]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i58.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i59 (.D(rgb_values[51]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i59.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i60 (.D(rgb_values[52]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i60.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i61 (.D(rgb_values[53]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i61.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i62 (.D(rgb_values[54]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i62.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i63 (.D(rgb_values[55]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i63.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i64 (.D(rgb_values[56]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i64.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i65 (.D(rgb_values[57]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i65.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i66 (.D(rgb_values[58]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i66.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i67 (.D(rgb_values[59]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i67.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i68 (.D(rgb_values[60]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i68.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i69 (.D(rgb_values[61]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i69.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i70 (.D(rgb_values[62]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i70.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i71 (.D(rgb_values[63]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i71.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i72 (.D(rgb_values[64]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i72.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i73 (.D(rgb_values[65]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i73.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i74 (.D(rgb_values[66]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i74.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i75 (.D(rgb_values[67]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i75.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i76 (.D(rgb_values[68]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i76.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i77 (.D(rgb_values[69]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i77.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i78 (.D(rgb_values[70]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i78.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i79 (.D(rgb_values[71]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i79.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i80 (.D(rgb_values[72]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i80.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i81 (.D(rgb_values[73]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i81.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i82 (.D(rgb_values[74]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i82.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i83 (.D(rgb_values[75]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i83.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i84 (.D(rgb_values[76]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i84.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i85 (.D(rgb_values[77]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i85.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i86 (.D(rgb_values[78]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i86.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i87 (.D(rgb_values[79]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i87.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i88 (.D(rgb_values[80]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i88.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i89 (.D(rgb_values[81]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[89])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i89.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i90 (.D(rgb_values[82]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[90])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i90.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i91 (.D(rgb_values[83]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[91])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i91.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i92 (.D(rgb_values[84]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[92])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i92.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i93 (.D(rgb_values[85]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[93])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i93.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i94 (.D(rgb_values[86]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[94])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i94.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i95 (.D(rgb_values[87]), .SP(spi1_sck_c_enable_271), 
            .CK(spi1_sck_c), .Q(rgb_values[95])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam rgb_values_i0_i95.GSR = "DISABLED";
    FD1P3AX rgb_hold__i2 (.D(rgb_values[73]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i2.GSR = "DISABLED";
    FD1P3AX rgb_hold__i3 (.D(rgb_values[74]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i3.GSR = "DISABLED";
    FD1P3AX rgb_hold__i4 (.D(rgb_values[75]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i4.GSR = "DISABLED";
    FD1P3AX rgb_hold__i5 (.D(rgb_values[76]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i5.GSR = "DISABLED";
    FD1P3AX rgb_hold__i6 (.D(rgb_values[77]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i6.GSR = "DISABLED";
    FD1P3AX rgb_hold__i7 (.D(rgb_values[78]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i7.GSR = "DISABLED";
    FD1P3AX rgb_hold__i8 (.D(rgb_values[79]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i8.GSR = "DISABLED";
    FD1P3AX rgb_hold__i9 (.D(rgb_values[80]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i9.GSR = "DISABLED";
    FD1P3AX rgb_hold__i10 (.D(rgb_values[81]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i10.GSR = "DISABLED";
    FD1P3AX rgb_hold__i11 (.D(rgb_values[82]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i11.GSR = "DISABLED";
    FD1P3AX rgb_hold__i12 (.D(rgb_values[83]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i12.GSR = "DISABLED";
    FD1P3AX rgb_hold__i13 (.D(rgb_values[84]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i13.GSR = "DISABLED";
    FD1P3AX rgb_hold__i14 (.D(rgb_values[85]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i14.GSR = "DISABLED";
    FD1P3AX rgb_hold__i15 (.D(rgb_values[86]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i15.GSR = "DISABLED";
    FD1P3AX rgb_hold__i16 (.D(rgb_values[87]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i16.GSR = "DISABLED";
    FD1P3AX rgb_hold__i17 (.D(rgb_values[88]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i17.GSR = "DISABLED";
    FD1P3AX rgb_hold__i18 (.D(rgb_values[89]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[89])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i18.GSR = "DISABLED";
    FD1P3AX rgb_hold__i19 (.D(rgb_values[90]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[90])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i19.GSR = "DISABLED";
    FD1P3AX rgb_hold__i20 (.D(rgb_values[91]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[91])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i20.GSR = "DISABLED";
    FD1P3AX rgb_hold__i21 (.D(rgb_values[92]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[92])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i21.GSR = "DISABLED";
    FD1P3AX rgb_hold__i22 (.D(rgb_values[93]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[93])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i22.GSR = "DISABLED";
    FD1P3AX rgb_hold__i23 (.D(rgb_values[94]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[94])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i23.GSR = "DISABLED";
    FD1P3AX rgb_hold__i24 (.D(rgb_values[95]), .SP(pll_clk_enable_77), .CK(pll_clk), 
            .Q(rgb_hold[95])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam rgb_hold__i24.GSR = "DISABLED";
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i1 (.D(spi_byte_count_15__N_1643[1]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i2 (.D(spi_byte_count_15__N_1643[2]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i3 (.D(spi_byte_count_15__N_1643[3]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i4 (.D(spi_byte_count_15__N_1643[4]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i5 (.D(spi_byte_count_15__N_1643[5]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i6 (.D(spi_byte_count_15__N_1643[6]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i7 (.D(spi_byte_count_15__N_1643[7]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i8 (.D(spi_byte_count_15__N_1643[8]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i9 (.D(spi_byte_count_15__N_1643[9]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i10 (.D(spi_byte_count_15__N_1643[10]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i11 (.D(spi_byte_count_15__N_1643[11]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i12 (.D(spi_byte_count_15__N_1643[12]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i13 (.D(spi_byte_count_15__N_1643[13]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i14 (.D(spi_byte_count_15__N_1643[14]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i15 (.D(spi_byte_count_15__N_1643[15]), .SP(spi1_sck_c_enable_286), 
            .CK(spi1_sck_c), .Q(spi_byte_count[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam spi_byte_count_i0_i15.GSR = "ENABLED";
    FD1P3IX frame_settle__i1 (.D(n14394), .SP(pll_clk_enable_79), .CD(n11144), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    FD1P3IX frame_settle__i2 (.D(n14396), .SP(pll_clk_enable_79), .CD(n11144), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    FD1P3IX us_tx__i2 (.D(n11434), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i2.GSR = "DISABLED";
    FD1P3IX us_tx__i3 (.D(n11436), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i3.GSR = "DISABLED";
    FD1P3IX us_tx__i4 (.D(n11438), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i4.GSR = "DISABLED";
    FD1P3IX us_tx__i5 (.D(n11440), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i5.GSR = "DISABLED";
    FD1P3IX us_tx__i6 (.D(n11442), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i6.GSR = "DISABLED";
    FD1P3IX us_tx__i7 (.D(n11444), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i7.GSR = "DISABLED";
    FD1P3IX us_tx__i8 (.D(n11446), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i8.GSR = "DISABLED";
    FD1P3IX us_tx__i9 (.D(n11448), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i9.GSR = "DISABLED";
    FD1P3IX us_tx__i10 (.D(n11450), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i10.GSR = "DISABLED";
    FD1P3IX us_tx__i11 (.D(n11452), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i11.GSR = "DISABLED";
    FD1P3IX us_tx__i12 (.D(n11454), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i12.GSR = "DISABLED";
    FD1P3IX us_tx__i13 (.D(n11456), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i13.GSR = "DISABLED";
    FD1P3IX us_tx__i14 (.D(n11458), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i14.GSR = "DISABLED";
    FD1P3IX us_tx__i15 (.D(n11460), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i15.GSR = "DISABLED";
    FD1P3IX us_tx__i16 (.D(n11462), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i16.GSR = "DISABLED";
    FD1P3IX us_tx__i17 (.D(n11464), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i17.GSR = "DISABLED";
    FD1P3IX us_tx__i18 (.D(n11466), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i18.GSR = "DISABLED";
    FD1P3IX us_tx__i19 (.D(n11468), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i19.GSR = "DISABLED";
    FD1P3IX us_tx__i20 (.D(n11470), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i20.GSR = "DISABLED";
    FD1P3IX us_tx__i21 (.D(n11472), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i21.GSR = "DISABLED";
    FD1P3IX us_tx__i22 (.D(n11474), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i22.GSR = "DISABLED";
    FD1P3IX us_tx__i23 (.D(n11476), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i23.GSR = "DISABLED";
    FD1P3IX us_tx__i24 (.D(n11478), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i24.GSR = "DISABLED";
    FD1P3IX us_tx__i25 (.D(n11480), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i25.GSR = "DISABLED";
    FD1P3IX us_tx__i26 (.D(n11482), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i26.GSR = "DISABLED";
    FD1P3IX us_tx__i27 (.D(n11484), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i27.GSR = "DISABLED";
    FD1P3IX us_tx__i28 (.D(n11486), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i28.GSR = "DISABLED";
    FD1P3IX us_tx__i29 (.D(n11488), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i29.GSR = "DISABLED";
    FD1P3IX us_tx__i30 (.D(n11490), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i30.GSR = "DISABLED";
    FD1P3IX us_tx__i31 (.D(n11492), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i31.GSR = "DISABLED";
    FD1P3IX us_tx__i32 (.D(n11494), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i32.GSR = "DISABLED";
    FD1P3IX us_tx__i33 (.D(n11496), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i33.GSR = "DISABLED";
    FD1P3IX us_tx__i34 (.D(n11498), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i34.GSR = "DISABLED";
    FD1P3IX us_tx__i35 (.D(n11500), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i35.GSR = "DISABLED";
    FD1P3IX us_tx__i36 (.D(n11502), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i36.GSR = "DISABLED";
    FD1P3IX us_tx__i37 (.D(n11504), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i37.GSR = "DISABLED";
    FD1P3IX us_tx__i38 (.D(n11506), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i38.GSR = "DISABLED";
    FD1P3IX us_tx__i39 (.D(n11508), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i39.GSR = "DISABLED";
    FD1P3IX us_tx__i40 (.D(n11510), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i40.GSR = "DISABLED";
    FD1P3IX us_tx__i41 (.D(n11512), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i41.GSR = "DISABLED";
    FD1P3IX us_tx__i42 (.D(n11514), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i42.GSR = "DISABLED";
    FD1P3IX us_tx__i43 (.D(n11516), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i43.GSR = "DISABLED";
    FD1P3IX us_tx__i44 (.D(n11518), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i44.GSR = "DISABLED";
    FD1P3IX us_tx__i45 (.D(n11520), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i45.GSR = "DISABLED";
    FD1P3IX us_tx__i46 (.D(n11522), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i46.GSR = "DISABLED";
    FD1P3IX us_tx__i47 (.D(n11524), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i47.GSR = "DISABLED";
    FD1P3IX us_tx__i48 (.D(n11526), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i48.GSR = "DISABLED";
    FD1P3IX us_tx__i49 (.D(n11528), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i49.GSR = "DISABLED";
    FD1P3IX us_tx__i50 (.D(n11530), .SP(pll_clk_enable_128), .CD(n23840), 
            .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i50.GSR = "DISABLED";
    FD1P3IX us_tx__i51 (.D(n11532), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i51.GSR = "DISABLED";
    FD1P3IX us_tx__i52 (.D(n11534), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i52.GSR = "DISABLED";
    FD1P3IX us_tx__i53 (.D(n11536), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i53.GSR = "DISABLED";
    FD1P3IX us_tx__i54 (.D(n11538), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i54.GSR = "DISABLED";
    FD1P3IX us_tx__i55 (.D(n11540), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i55.GSR = "DISABLED";
    FD1P3IX us_tx__i56 (.D(n11542), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i56.GSR = "DISABLED";
    FD1P3IX us_tx__i57 (.D(n11544), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i57.GSR = "DISABLED";
    FD1P3IX us_tx__i58 (.D(n11546), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i58.GSR = "DISABLED";
    FD1P3IX us_tx__i59 (.D(n11548), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i59.GSR = "DISABLED";
    FD1P3IX us_tx__i60 (.D(n11550), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i60.GSR = "DISABLED";
    FD1P3IX us_tx__i61 (.D(n11552), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i61.GSR = "DISABLED";
    FD1P3IX us_tx__i62 (.D(n11554), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i62.GSR = "DISABLED";
    FD1P3IX us_tx__i63 (.D(n11556), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i63.GSR = "DISABLED";
    FD1P3IX us_tx__i64 (.D(n11558), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i64.GSR = "DISABLED";
    FD1P3IX us_tx__i65 (.D(n11560), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i65.GSR = "DISABLED";
    FD1P3IX us_tx__i66 (.D(n11562), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i66.GSR = "DISABLED";
    FD1P3IX us_tx__i67 (.D(n11564), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i67.GSR = "DISABLED";
    FD1P3IX us_tx__i68 (.D(n11566), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i68.GSR = "DISABLED";
    FD1P3IX us_tx__i69 (.D(n11568), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i69.GSR = "DISABLED";
    FD1P3IX us_tx__i70 (.D(n11570), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i70.GSR = "DISABLED";
    FD1P3IX us_tx__i71 (.D(n11572), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i71.GSR = "DISABLED";
    FD1P3IX us_tx__i72 (.D(n11574), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i72.GSR = "DISABLED";
    FD1P3IX us_tx__i73 (.D(n11576), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i73.GSR = "DISABLED";
    FD1P3IX us_tx__i74 (.D(n11578), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i74.GSR = "DISABLED";
    FD1P3IX us_tx__i75 (.D(n11580), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i75.GSR = "DISABLED";
    FD1P3IX us_tx__i76 (.D(n11582), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i76.GSR = "DISABLED";
    FD1P3IX us_tx__i77 (.D(n11584), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i77.GSR = "DISABLED";
    FD1P3IX us_tx__i78 (.D(n11586), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i78.GSR = "DISABLED";
    FD1P3IX us_tx__i79 (.D(n11588), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i79.GSR = "DISABLED";
    FD1P3IX us_tx__i80 (.D(n11590), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i80.GSR = "DISABLED";
    FD1P3IX us_tx__i81 (.D(n11592), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i81.GSR = "DISABLED";
    FD1P3IX us_tx__i82 (.D(n11594), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i82.GSR = "DISABLED";
    FD1P3IX us_tx__i83 (.D(n11596), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i83.GSR = "DISABLED";
    FD1P3IX us_tx__i84 (.D(n11598), .SP(pll_clk_enable_162), .CD(n9326), 
            .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam us_tx__i84.GSR = "DISABLED";
    FD1S3AX global_phase_i1 (.D(next_global_phase[1]), .CK(pll_clk), .Q(global_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_107 (.A(build_sum[8]), .B(n23529), .C(ev_bit[82]), 
         .D(init_shadow[82]), .Z(n13355)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_107.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_108 (.A(build_sum[8]), .B(n23529), .C(ev_bit[81]), 
         .D(init_shadow[81]), .Z(n13349)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_108.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_109 (.A(build_sum[8]), .B(n23529), .C(ev_bit[80]), 
         .D(init_shadow[80]), .Z(n13343)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_109.init = 16'hff80;
    LUT4 i1_2_lut_4_lut (.A(frame_end), .B(fpga_cs_n_c), .C(spi1_sck_c_enable_286), 
         .D(n23491), .Z(spi1_sck_c_enable_317)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_2_lut_4_lut.init = 16'h2000;
    LUT4 i1_3_lut_4_lut_adj_110 (.A(build_sum[8]), .B(n23529), .C(ev_bit[79]), 
         .D(init_shadow[79]), .Z(n13337)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_110.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_111 (.A(build_sum[8]), .B(n23529), .C(ev_bit[78]), 
         .D(init_shadow[78]), .Z(n13331)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_111.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_112 (.A(build_sum[8]), .B(n23529), .C(ev_bit[77]), 
         .D(init_shadow[77]), .Z(n13325)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_112.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_113 (.A(build_sum[8]), .B(n23529), .C(ev_bit[76]), 
         .D(init_shadow[76]), .Z(n13319)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_113.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_114 (.A(build_sum[8]), .B(n23529), .C(ev_bit[75]), 
         .D(init_shadow[75]), .Z(n13313)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_114.init = 16'hff80;
    CCU2D mic_divider_1060_add_4_5 (.A0(mic_divider[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22045), .COUT(n22046), .S0(n37_adj_3021), 
          .S1(n36_adj_3022));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_divider_1060_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_divider_1060_add_4_5.INJECT1_0 = "NO";
    defparam mic_divider_1060_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_115 (.A(expected_next[9]), .B(expected_next[5]), .Z(n16_adj_3063)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[48:79])
    defparam i1_2_lut_adj_115.init = 16'hbbbb;
    CCU2D mic_divider_1060_add_4_3 (.A0(mic_divider[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22044), .COUT(n22045), .S0(n39), .S1(n38));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_divider_1060_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_divider_1060_add_4_3.INJECT1_0 = "NO";
    defparam mic_divider_1060_add_4_3.INJECT1_1 = "NO";
    LUT4 i11_4_lut (.A(expected_next[12]), .B(expected_next[15]), .C(expected_next[8]), 
         .D(expected_next[10]), .Z(n26_adj_3052)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[48:79])
    defparam i11_4_lut.init = 16'hfffe;
    CCU2D spi_channel_index_1054_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22022), .S0(n35), .S1(n34));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1054_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1054_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_1054_add_4_7.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_496_15 (.A0(spi_rx_shift[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22008), .S0(expected_next[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_15.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_496_15.INIT1 = 16'h0000;
    defparam expected_next_15__I_0_496_15.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_15.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_116 (.A(build_sum[8]), .B(n23529), .C(ev_bit[74]), 
         .D(init_shadow[74]), .Z(n13307)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_116.init = 16'hff80;
    CCU2D mic_divider_1060_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22044), .S1(n40));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060_add_4_1.INIT0 = 16'hF000;
    defparam mic_divider_1060_add_4_1.INIT1 = 16'h0555;
    defparam mic_divider_1060_add_4_1.INJECT1_0 = "NO";
    defparam mic_divider_1060_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_117 (.A(build_sum[8]), .B(n23529), .C(ev_bit[73]), 
         .D(init_shadow[73]), .Z(n13301)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_117.init = 16'hff80;
    LUT4 i5_2_lut (.A(expected_next[11]), .B(expected_next[14]), .Z(n20_adj_3061)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[48:79])
    defparam i5_2_lut.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_118 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[26]), 
         .D(ev_bit[26]), .Z(ev_wr_data[26])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_118.init = 16'hddd0;
    CCU2D spi_channel_index_1054_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22021), .COUT(n22022), .S0(n37), 
          .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1054_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1054_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_1054_add_4_5.INJECT1_1 = "NO";
    CCU2D add_499_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21992), .COUT(n21993), .S0(phase_frac_sum[5]), .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_7.INIT0 = 16'h5555;
    defparam add_499_7.INIT1 = 16'h5555;
    defparam add_499_7.INJECT1_0 = "NO";
    defparam add_499_7.INJECT1_1 = "NO";
    CCU2D spi_channel_index_1054_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22020), .COUT(n22021), .S0(n39_adj_3045), 
          .S1(n38_adj_3044));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1054_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1054_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_1054_add_4_3.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_119 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[28]), 
         .D(ev_bit[28]), .Z(ev_wr_data[28])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_119.init = 16'hddd0;
    CCU2D expected_next_15__I_0_496_13 (.A0(spi_rx_shift[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22007), .COUT(n22008), .S0(expected_next[13]), 
          .S1(expected_next[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_13.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_496_13.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_496_13.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_13.INJECT1_1 = "NO";
    CCU2D fpga_time_1057_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22043), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_1057_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_33.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_120 (.A(build_sum[8]), .B(n23529), .C(ev_bit[72]), 
         .D(init_shadow[72]), .Z(n13295)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_120.init = 16'hff80;
    CCU2D spi_channel_index_1054_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_channel_index[0]), .B1(n22069), .C1(spi_channel_index[2]), 
          .D1(n5_adj_3065), .COUT(n22020), .S1(n40_adj_3046));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_1054_add_4_1.INIT1 = 16'h5559;
    defparam spi_channel_index_1054_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_1054_add_4_1.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_496_11 (.A0(spi_rx_shift[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22006), .COUT(n22007), .S0(expected_next[11]), 
          .S1(expected_next[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_11.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_496_11.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_496_11.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_11.INJECT1_1 = "NO";
    CCU2D fpga_time_1057_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22042), .COUT(n22043), .S0(n136), .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_31.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_121 (.A(expected_next[1]), .B(expected_next[0]), .Z(n22723)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[17:30])
    defparam i1_2_lut_adj_121.init = 16'heeee;
    FD1S3IX ev_bit_i64 (.D(n14378), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i64.GSR = "DISABLED";
    FD1P3IX init_shadow_i5 (.D(n12796), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i5.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_122 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[29]), 
         .D(ev_bit[29]), .Z(ev_wr_data[29])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_122.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_123 (.A(build_sum[8]), .B(n23529), .C(ev_bit[71]), 
         .D(init_shadow[71]), .Z(n13289)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_123.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_124 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[30]), 
         .D(ev_bit[30]), .Z(ev_wr_data[30])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_124.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_125 (.A(build_sum[8]), .B(n23529), .C(ev_bit[70]), 
         .D(init_shadow[70]), .Z(n13283)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_125.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_126 (.A(build_sum[8]), .B(n23529), .C(ev_bit[69]), 
         .D(init_shadow[69]), .Z(n13277)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_126.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_127 (.A(build_sum[8]), .B(n23529), .C(ev_bit[68]), 
         .D(init_shadow[68]), .Z(n13271)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_127.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_128 (.A(build_sum[8]), .B(n23529), .C(ev_bit[67]), 
         .D(init_shadow[67]), .Z(n13265)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_128.init = 16'hff80;
    LUT4 i3_4_lut_adj_129 (.A(spi_command[6]), .B(spi_command[5]), .C(spi_command[3]), 
         .D(spi_command[2]), .Z(n24)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_129.init = 16'hfffe;
    LUT4 i3_4_lut_adj_130 (.A(spi_byte_count[5]), .B(n22772), .C(n23531), 
         .D(n22660), .Z(spi1_sck_c_enable_26)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i3_4_lut_adj_130.init = 16'h2000;
    LUT4 i1_3_lut_4_lut_adj_131 (.A(build_sum[8]), .B(n23529), .C(ev_bit[66]), 
         .D(init_shadow[66]), .Z(n13259)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_131.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_132 (.A(build_sum[8]), .B(n23529), .C(ev_bit[65]), 
         .D(init_shadow[65]), .Z(n13253)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_132.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_133 (.A(build_sum[8]), .B(n23529), .C(ev_bit[64]), 
         .D(init_shadow[64]), .Z(n13247)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_133.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_134 (.A(build_sum[8]), .B(n23529), .C(ev_bit[62]), 
         .D(init_shadow[62]), .Z(n13231)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_134.init = 16'hff80;
    LUT4 i14391_2_lut_rep_83 (.A(ev_state[2]), .B(n19130), .Z(n23493)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14391_2_lut_rep_83.init = 16'h8888;
    FD1S3AX global_phase_i2 (.D(next_global_phase[2]), .CK(pll_clk), .Q(global_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i2.GSR = "DISABLED";
    FD1S3AX global_phase_i3 (.D(next_global_phase[3]), .CK(pll_clk), .Q(global_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i3.GSR = "DISABLED";
    FD1S3AX global_phase_i4 (.D(next_global_phase[4]), .CK(pll_clk), .Q(global_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i4.GSR = "DISABLED";
    FD1S3AX global_phase_i5 (.D(next_global_phase[5]), .CK(pll_clk), .Q(global_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i5.GSR = "DISABLED";
    FD1S3AX global_phase_i6 (.D(next_global_phase[6]), .CK(pll_clk), .Q(global_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i6.GSR = "DISABLED";
    FD1S3AX global_phase_i7 (.D(next_global_phase[7]), .CK(pll_clk), .Q(global_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam global_phase_i7.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i8 (.D(run_bank), .CK(pll_clk), .Q(run_addr_reg[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam run_addr_reg_i8.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i1 (.D(ev_rd_data[1]), .CK(pll_clk), .Q(ev_run_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i1.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i2 (.D(ev_rd_data[2]), .CK(pll_clk), .Q(ev_run_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i2.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i3 (.D(ev_rd_data[3]), .CK(pll_clk), .Q(ev_run_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i3.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i4 (.D(ev_rd_data[4]), .CK(pll_clk), .Q(ev_run_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i4.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i5 (.D(ev_rd_data[5]), .CK(pll_clk), .Q(ev_run_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i5.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i6 (.D(ev_rd_data[6]), .CK(pll_clk), .Q(ev_run_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i6.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i7 (.D(ev_rd_data[7]), .CK(pll_clk), .Q(ev_run_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i7.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i8 (.D(ev_rd_data[8]), .CK(pll_clk), .Q(ev_run_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i8.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i9 (.D(ev_rd_data[9]), .CK(pll_clk), .Q(ev_run_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i9.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i10 (.D(ev_rd_data[10]), .CK(pll_clk), .Q(ev_run_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i10.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i11 (.D(ev_rd_data[11]), .CK(pll_clk), .Q(ev_run_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i11.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i12 (.D(ev_rd_data[12]), .CK(pll_clk), .Q(ev_run_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i12.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i13 (.D(ev_rd_data[13]), .CK(pll_clk), .Q(ev_run_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i13.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i14 (.D(ev_rd_data[14]), .CK(pll_clk), .Q(ev_run_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i14.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i15 (.D(ev_rd_data[15]), .CK(pll_clk), .Q(ev_run_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i15.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i16 (.D(ev_rd_data[16]), .CK(pll_clk), .Q(ev_run_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i16.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i17 (.D(ev_rd_data[17]), .CK(pll_clk), .Q(ev_run_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i17.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i18 (.D(ev_rd_data[18]), .CK(pll_clk), .Q(ev_run_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i18.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i19 (.D(ev_rd_data[19]), .CK(pll_clk), .Q(ev_run_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i19.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i20 (.D(ev_rd_data[20]), .CK(pll_clk), .Q(ev_run_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i20.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i21 (.D(ev_rd_data[21]), .CK(pll_clk), .Q(ev_run_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i21.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i22 (.D(ev_rd_data[22]), .CK(pll_clk), .Q(ev_run_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i22.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i23 (.D(ev_rd_data[23]), .CK(pll_clk), .Q(ev_run_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i23.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i24 (.D(ev_rd_data[24]), .CK(pll_clk), .Q(ev_run_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i24.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i25 (.D(ev_rd_data[25]), .CK(pll_clk), .Q(ev_run_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i25.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i26 (.D(ev_rd_data[26]), .CK(pll_clk), .Q(ev_run_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i26.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i27 (.D(ev_rd_data[27]), .CK(pll_clk), .Q(ev_run_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i27.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i28 (.D(ev_rd_data[28]), .CK(pll_clk), .Q(ev_run_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i28.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i29 (.D(ev_rd_data[29]), .CK(pll_clk), .Q(ev_run_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i29.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i30 (.D(ev_rd_data[30]), .CK(pll_clk), .Q(ev_run_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i30.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i31 (.D(ev_rd_data[31]), .CK(pll_clk), .Q(ev_run_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i31.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i32 (.D(ev_rd_data[32]), .CK(pll_clk), .Q(ev_run_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i32.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i33 (.D(ev_rd_data[33]), .CK(pll_clk), .Q(ev_run_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i33.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i34 (.D(ev_rd_data[34]), .CK(pll_clk), .Q(ev_run_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i34.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i35 (.D(ev_rd_data[35]), .CK(pll_clk), .Q(ev_run_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i35.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i36 (.D(ev_rd_data[36]), .CK(pll_clk), .Q(ev_run_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i36.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i37 (.D(ev_rd_data[37]), .CK(pll_clk), .Q(ev_run_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i37.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i38 (.D(ev_rd_data[38]), .CK(pll_clk), .Q(ev_run_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i38.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i39 (.D(ev_rd_data[39]), .CK(pll_clk), .Q(ev_run_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i39.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i40 (.D(ev_rd_data[40]), .CK(pll_clk), .Q(ev_run_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i40.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i41 (.D(ev_rd_data[41]), .CK(pll_clk), .Q(ev_run_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i41.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i42 (.D(ev_rd_data[42]), .CK(pll_clk), .Q(ev_run_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i42.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i43 (.D(ev_rd_data[43]), .CK(pll_clk), .Q(ev_run_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i43.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i44 (.D(ev_rd_data[44]), .CK(pll_clk), .Q(ev_run_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i44.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i45 (.D(ev_rd_data[45]), .CK(pll_clk), .Q(ev_run_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i45.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i46 (.D(ev_rd_data[46]), .CK(pll_clk), .Q(ev_run_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i46.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i47 (.D(ev_rd_data[47]), .CK(pll_clk), .Q(ev_run_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i47.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i48 (.D(ev_rd_data[48]), .CK(pll_clk), .Q(ev_run_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i48.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i49 (.D(ev_rd_data[49]), .CK(pll_clk), .Q(ev_run_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i49.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i50 (.D(ev_rd_data[50]), .CK(pll_clk), .Q(ev_run_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i50.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i51 (.D(ev_rd_data[51]), .CK(pll_clk), .Q(ev_run_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i51.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i52 (.D(ev_rd_data[52]), .CK(pll_clk), .Q(ev_run_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i52.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i53 (.D(ev_rd_data[53]), .CK(pll_clk), .Q(ev_run_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i53.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i54 (.D(ev_rd_data[54]), .CK(pll_clk), .Q(ev_run_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i54.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i55 (.D(ev_rd_data[55]), .CK(pll_clk), .Q(ev_run_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i55.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i56 (.D(ev_rd_data[56]), .CK(pll_clk), .Q(ev_run_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i56.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i57 (.D(ev_rd_data[57]), .CK(pll_clk), .Q(ev_run_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i57.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i58 (.D(ev_rd_data[58]), .CK(pll_clk), .Q(ev_run_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i58.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i59 (.D(ev_rd_data[59]), .CK(pll_clk), .Q(ev_run_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i59.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i60 (.D(ev_rd_data[60]), .CK(pll_clk), .Q(ev_run_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i60.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i61 (.D(ev_rd_data[61]), .CK(pll_clk), .Q(ev_run_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i61.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i62 (.D(ev_rd_data[62]), .CK(pll_clk), .Q(ev_run_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i62.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i63 (.D(ev_rd_data[63]), .CK(pll_clk), .Q(ev_run_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i63.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i64 (.D(ev_rd_data[64]), .CK(pll_clk), .Q(ev_run_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i64.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i65 (.D(ev_rd_data[65]), .CK(pll_clk), .Q(ev_run_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i65.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i66 (.D(ev_rd_data[66]), .CK(pll_clk), .Q(ev_run_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i66.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i67 (.D(ev_rd_data[67]), .CK(pll_clk), .Q(ev_run_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i67.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i68 (.D(ev_rd_data[68]), .CK(pll_clk), .Q(ev_run_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i68.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i69 (.D(ev_rd_data[69]), .CK(pll_clk), .Q(ev_run_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i69.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i70 (.D(ev_rd_data[70]), .CK(pll_clk), .Q(ev_run_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i70.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i71 (.D(ev_rd_data[71]), .CK(pll_clk), .Q(ev_run_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i71.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i72 (.D(ev_rd_data[72]), .CK(pll_clk), .Q(ev_run_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i72.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i73 (.D(ev_rd_data[73]), .CK(pll_clk), .Q(ev_run_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i73.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i74 (.D(ev_rd_data[74]), .CK(pll_clk), .Q(ev_run_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i74.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i75 (.D(ev_rd_data[75]), .CK(pll_clk), .Q(ev_run_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i75.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i76 (.D(ev_rd_data[76]), .CK(pll_clk), .Q(ev_run_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i76.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i77 (.D(ev_rd_data[77]), .CK(pll_clk), .Q(ev_run_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i77.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i78 (.D(ev_rd_data[78]), .CK(pll_clk), .Q(ev_run_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i78.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i79 (.D(ev_rd_data[79]), .CK(pll_clk), .Q(ev_run_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i79.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i80 (.D(ev_rd_data[80]), .CK(pll_clk), .Q(ev_run_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i80.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i81 (.D(ev_rd_data[81]), .CK(pll_clk), .Q(ev_run_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i81.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i82 (.D(ev_rd_data[82]), .CK(pll_clk), .Q(ev_run_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i82.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i83 (.D(ev_rd_data[83]), .CK(pll_clk), .Q(ev_run_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_run_hold_i83.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i1 (.D(accepted_sequence_spi[1]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i2 (.D(accepted_sequence_spi[2]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i3 (.D(accepted_sequence_spi[3]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i4 (.D(accepted_sequence_spi[4]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i5 (.D(accepted_sequence_spi[5]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i6 (.D(accepted_sequence_spi[6]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i7 (.D(accepted_sequence_spi[7]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i8 (.D(accepted_sequence_spi[8]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i9 (.D(accepted_sequence_spi[9]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i10 (.D(accepted_sequence_spi[10]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i11 (.D(accepted_sequence_spi[11]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i12 (.D(accepted_sequence_spi[12]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i13 (.D(accepted_sequence_spi[13]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i14 (.D(accepted_sequence_spi[14]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i15 (.D(accepted_sequence_spi[15]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i16 (.D(accepted_sequence_spi[16]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i17 (.D(accepted_sequence_spi[17]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i18 (.D(accepted_sequence_spi[18]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i19 (.D(accepted_sequence_spi[19]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i20 (.D(accepted_sequence_spi[20]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i21 (.D(accepted_sequence_spi[21]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i22 (.D(accepted_sequence_spi[22]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i23 (.D(accepted_sequence_spi[23]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i24 (.D(accepted_sequence_spi[24]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i25 (.D(accepted_sequence_spi[25]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i26 (.D(accepted_sequence_spi[26]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i27 (.D(accepted_sequence_spi[27]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i28 (.D(accepted_sequence_spi[28]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i29 (.D(accepted_sequence_spi[29]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i30 (.D(accepted_sequence_spi[30]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i31 (.D(accepted_sequence_spi[31]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_meta_i31.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i1 (.D(accepted_sequence_meta[1]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i2 (.D(accepted_sequence_meta[2]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i3 (.D(accepted_sequence_meta[3]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i4 (.D(accepted_sequence_meta[4]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i5 (.D(accepted_sequence_meta[5]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i6 (.D(accepted_sequence_meta[6]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i7 (.D(accepted_sequence_meta[7]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i8 (.D(accepted_sequence_meta[8]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i9 (.D(accepted_sequence_meta[9]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i10 (.D(accepted_sequence_meta[10]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i11 (.D(accepted_sequence_meta[11]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i12 (.D(accepted_sequence_meta[12]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i13 (.D(accepted_sequence_meta[13]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i14 (.D(accepted_sequence_meta[14]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i15 (.D(accepted_sequence_meta[15]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i16 (.D(accepted_sequence_meta[16]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i17 (.D(accepted_sequence_meta[17]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i18 (.D(accepted_sequence_meta[18]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i19 (.D(accepted_sequence_meta[19]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i20 (.D(accepted_sequence_meta[20]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i21 (.D(accepted_sequence_meta[21]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i22 (.D(accepted_sequence_meta[22]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i23 (.D(accepted_sequence_meta[23]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i24 (.D(accepted_sequence_meta[24]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i25 (.D(accepted_sequence_meta[25]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i26 (.D(accepted_sequence_meta[26]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i27 (.D(accepted_sequence_meta[27]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i28 (.D(accepted_sequence_meta[28]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i29 (.D(accepted_sequence_meta[29]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i30 (.D(accepted_sequence_meta[30]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i31 (.D(accepted_sequence_meta[31]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_sync_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i1 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i2 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i3 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i4 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i5 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i6 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i7 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i8 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i9 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i10 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i11 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i12 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i13 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i14 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_l_i0_i15 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_0_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_l_i0_i15.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_135 (.A(build_sum[8]), .B(n23529), .C(ev_bit[61]), 
         .D(init_shadow[61]), .Z(n13223)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_135.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_136 (.A(build_sum[8]), .B(n23529), .C(ev_bit[60]), 
         .D(init_shadow[60]), .Z(n13213)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_136.init = 16'hff80;
    FD1P3AX build_phase_i1 (.D(staging_q[9]), .SP(pll_clk_enable_193), .CK(pll_clk), 
            .Q(build_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i1.GSR = "DISABLED";
    FD1P3AX build_phase_i2 (.D(staging_q[10]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i2.GSR = "DISABLED";
    FD1P3AX build_phase_i3 (.D(staging_q[11]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i4 (.D(staging_q[12]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i4.GSR = "DISABLED";
    FD1P3AX build_phase_i5 (.D(staging_q[13]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i5.GSR = "DISABLED";
    FD1P3AX build_phase_i6 (.D(staging_q[14]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i6.GSR = "DISABLED";
    FD1P3AX build_phase_i7 (.D(staging_q[15]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_phase_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i1 (.D(build_sum_8__N_2005[1]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i1.GSR = "DISABLED";
    FD1P3AX build_sum_i2 (.D(build_sum_8__N_2005[2]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i2.GSR = "DISABLED";
    FD1P3AX build_sum_i3 (.D(build_sum_8__N_2005[3]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i3.GSR = "DISABLED";
    FD1P3AX build_sum_i4 (.D(build_sum_8__N_2005[4]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i4.GSR = "DISABLED";
    FD1P3AX build_sum_i5 (.D(build_sum_8__N_2005[5]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i5.GSR = "DISABLED";
    FD1P3AX build_sum_i6 (.D(build_sum_8__N_2005[6]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i6.GSR = "DISABLED";
    FD1P3AX build_sum_i7 (.D(build_sum_8__N_2005[7]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i8 (.D(build_sum_8__N_2005[8]), .SP(pll_clk_enable_193), 
            .CK(pll_clk), .Q(build_sum[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam build_sum_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i1.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i2.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i3.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i4.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i5.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i6.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i7.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_242), .CK(pll_clk), 
            .Q(ev_rd_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i9.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i10.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i11.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i12.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i13.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i14.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i15.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i16.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i17.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i18.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i19.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i20.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i21.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i22.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i23.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i24.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i25.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i26.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i27.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i28.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i29.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i30.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i31.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i32.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i33.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i34.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i35.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i36.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i37.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i38.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i39.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i40.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i41.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i42.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i43.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i44.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i45.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i46.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i47.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i48.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_242), 
            .CK(pll_clk), .Q(ev_rd_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i49.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i50.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i51.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i52.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i53.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i54.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i55.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i56.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i57.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i58.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i59.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i60.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i61.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i62.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i63.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i64.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i65.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i66.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i67.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i68.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i69.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i70.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i71.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i72.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i73.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i74.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i75.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i76.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i77.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i78.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i79.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i80.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i81.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i82.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_276), 
            .CK(pll_clk), .Q(ev_rd_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_rd_hold_i83.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_137 (.A(build_sum[8]), .B(n23529), .C(ev_bit[59]), 
         .D(init_shadow[59]), .Z(n13207)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_137.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_138 (.A(build_sum[8]), .B(n23529), .C(ev_bit[57]), 
         .D(init_shadow[57]), .Z(n13195)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_138.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_139 (.A(build_sum[8]), .B(n23529), .C(ev_bit[56]), 
         .D(init_shadow[56]), .Z(n13189)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_139.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_140 (.A(build_sum[8]), .B(n23529), .C(ev_bit[55]), 
         .D(init_shadow[55]), .Z(n13183)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_140.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_141 (.A(build_sum[8]), .B(n23529), .C(ev_bit[54]), 
         .D(init_shadow[54]), .Z(n13177)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_141.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_142 (.A(build_sum[8]), .B(n23529), .C(ev_bit[53]), 
         .D(init_shadow[53]), .Z(n13171)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_142.init = 16'hff80;
    LUT4 mux_1430_i1_3_lut (.A(n9909), .B(n9910), .C(n9908), .Z(rd_data_15__N_2579[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i1_3_lut.init = 16'hcaca;
    LUT4 i7_4_lut_adj_143 (.A(n9907), .B(n22972), .C(n22900), .D(n6), 
         .Z(n9908)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7_4_lut_adj_143.init = 16'h0002;
    LUT4 i1_3_lut_4_lut_adj_144 (.A(build_sum[8]), .B(n23529), .C(ev_bit[52]), 
         .D(init_shadow[52]), .Z(n13165)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_144.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_145 (.A(build_sum[8]), .B(n23529), .C(ev_bit[51]), 
         .D(init_shadow[51]), .Z(n13159)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_145.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_146 (.A(build_sum[8]), .B(n23529), .C(ev_bit[50]), 
         .D(init_shadow[50]), .Z(n13153)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_146.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_147 (.A(build_sum[8]), .B(n23529), .C(ev_bit[49]), 
         .D(init_shadow[49]), .Z(n13147)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_147.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_148 (.A(build_sum[8]), .B(n23529), .C(ev_bit[48]), 
         .D(init_shadow[48]), .Z(n13141)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_148.init = 16'hff80;
    LUT4 i14451_4_lut (.A(n9896), .B(n22890), .C(n5_adj_3027), .D(n9897), 
         .Z(n22972)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+(C+(D)))) */ ;
    defparam i14451_4_lut.init = 16'hfdfe;
    LUT4 i25_4_lut_4_lut (.A(ev_state[2]), .B(n19130), .C(ev_state[1]), 
         .D(ev_state_3__N_1976[1]), .Z(n14_adj_3053)) /* synthesis lut_function=(A (B (C))+!A !(C+(D))) */ ;
    defparam i25_4_lut_4_lut.init = 16'h8085;
    LUT4 i14379_4_lut (.A(n9898), .B(n9892), .C(n9899), .D(n9893), .Z(n22900)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14379_4_lut.init = 16'h7bde;
    LUT4 i1_3_lut_4_lut_adj_149 (.A(build_sum[8]), .B(n23529), .C(ev_bit[47]), 
         .D(init_shadow[47]), .Z(n13135)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_149.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_150 (.A(build_sum[8]), .B(n23529), .C(ev_bit[45]), 
         .D(init_shadow[45]), .Z(n13121)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_150.init = 16'hff80;
    LUT4 equal_1411_i6_2_lut (.A(n9902), .B(n9903), .Z(n6)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam equal_1411_i6_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_151 (.A(build_sum[8]), .B(n23529), .C(ev_bit[43]), 
         .D(init_shadow[43]), .Z(n13109)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_151.init = 16'hff80;
    LUT4 i14369_4_lut (.A(n9904), .B(n9894), .C(n9905), .D(n9895), .Z(n22890)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i14369_4_lut.init = 16'h7bde;
    LUT4 i1_3_lut_4_lut_adj_152 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[31]), 
         .D(ev_bit[31]), .Z(ev_wr_data[31])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_152.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_153 (.A(build_sum[8]), .B(n23529), .C(ev_bit[42]), 
         .D(init_shadow[42]), .Z(n13103)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_153.init = 16'hff80;
    LUT4 equal_1411_i5_2_lut (.A(n9900), .B(n9901), .Z(n5_adj_3027)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam equal_1411_i5_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_154 (.A(build_sum[8]), .B(n23529), .C(ev_bit[11]), 
         .D(init_shadow[11]), .Z(n12834)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_154.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_155 (.A(build_sum[8]), .B(n23529), .C(ev_bit[10]), 
         .D(init_shadow[10]), .Z(n12826)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_155.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_156 (.A(build_sum[8]), .B(n23529), .C(ev_bit[8]), 
         .D(init_shadow[8]), .Z(n12814)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_156.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_157 (.A(build_sum[8]), .B(n23529), .C(ev_bit[1]), 
         .D(init_shadow[1]), .Z(n12772)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_157.init = 16'hff80;
    LUT4 i1_2_lut_adj_158 (.A(frame_settle[1]), .B(frame_settle[0]), .Z(n14394)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_158.init = 16'h9999;
    FD1S3AX staging_rd_addr_i1 (.D(staging_rd_addr_6__N_849[1]), .CK(pll_clk), 
            .Q(staging_rd_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam staging_rd_addr_i1.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i2 (.D(staging_rd_addr_6__N_849[2]), .CK(pll_clk), 
            .Q(staging_rd_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam staging_rd_addr_i2.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i3 (.D(staging_rd_addr_6__N_849[3]), .CK(pll_clk), 
            .Q(staging_rd_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam staging_rd_addr_i3.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i4 (.D(staging_rd_addr_6__N_849[4]), .CK(pll_clk), 
            .Q(staging_rd_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam staging_rd_addr_i4.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i5 (.D(staging_rd_addr_6__N_849[5]), .CK(pll_clk), 
            .Q(staging_rd_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam staging_rd_addr_i5.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i6 (.D(staging_rd_addr_6__N_849[6]), .CK(pll_clk), 
            .Q(staging_rd_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam staging_rd_addr_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i1 (.D(accepted_sequence_sync[1]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i2 (.D(accepted_sequence_sync[2]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i3 (.D(accepted_sequence_sync[3]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i4 (.D(accepted_sequence_sync[4]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i5 (.D(accepted_sequence_sync[5]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i6 (.D(accepted_sequence_sync[6]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i7 (.D(accepted_sequence_sync[7]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i8 (.D(accepted_sequence_sync[8]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i9 (.D(accepted_sequence_sync[9]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i10 (.D(accepted_sequence_sync[10]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i11 (.D(accepted_sequence_sync[11]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i12 (.D(accepted_sequence_sync[12]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i13 (.D(accepted_sequence_sync[13]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i14 (.D(accepted_sequence_sync[14]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i15 (.D(accepted_sequence_sync[15]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i15.GSR = "DISABLED";
    FD1P3AX pending_sequence_i16 (.D(accepted_sequence_sync[16]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i16.GSR = "DISABLED";
    FD1P3AX pending_sequence_i17 (.D(accepted_sequence_sync[17]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i17.GSR = "DISABLED";
    FD1P3AX pending_sequence_i18 (.D(accepted_sequence_sync[18]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i18.GSR = "DISABLED";
    FD1P3AX pending_sequence_i19 (.D(accepted_sequence_sync[19]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i19.GSR = "DISABLED";
    FD1P3AX pending_sequence_i20 (.D(accepted_sequence_sync[20]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i20.GSR = "DISABLED";
    FD1P3AX pending_sequence_i21 (.D(accepted_sequence_sync[21]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i21.GSR = "DISABLED";
    FD1P3AX pending_sequence_i22 (.D(accepted_sequence_sync[22]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i22.GSR = "DISABLED";
    FD1P3AX pending_sequence_i23 (.D(accepted_sequence_sync[23]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i23.GSR = "DISABLED";
    FD1P3AX pending_sequence_i24 (.D(accepted_sequence_sync[24]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i24.GSR = "DISABLED";
    FD1P3AX pending_sequence_i25 (.D(accepted_sequence_sync[25]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i25.GSR = "DISABLED";
    FD1P3AX pending_sequence_i26 (.D(accepted_sequence_sync[26]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i26.GSR = "DISABLED";
    FD1P3AX pending_sequence_i27 (.D(accepted_sequence_sync[27]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i27.GSR = "DISABLED";
    FD1P3AX pending_sequence_i28 (.D(accepted_sequence_sync[28]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i28.GSR = "DISABLED";
    FD1P3AX pending_sequence_i29 (.D(accepted_sequence_sync[29]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i29.GSR = "DISABLED";
    FD1P3AX pending_sequence_i30 (.D(accepted_sequence_sync[30]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i30.GSR = "DISABLED";
    FD1P3AX pending_sequence_i31 (.D(accepted_sequence_sync[31]), .SP(pll_clk_enable_307), 
            .CK(pll_clk), .Q(pending_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam pending_sequence_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i1 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i2 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i3 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i4 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i5 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i6 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i7 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i8 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i9 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i10 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i11 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i12 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i13 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i14 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_l_i0_i15 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_322), 
            .CK(pll_clk), .Q(mic_shift_1_l[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_l_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i2 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i3 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i4 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i5 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i6 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i7 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i8 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i9 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i10 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i11 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i12 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i13 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i14 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0_r__i15 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_0_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_0_r__i15.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i2 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i3 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i4 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i5 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i6 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i7 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i8 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i9 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i10 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i11 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i12 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i13 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i14 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1_r__i15 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_shift_1_r[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_shift_1_r__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1_r[0]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1_r[1]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1_r[2]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1_r[3]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1_r[4]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1_r[5]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1_r[6]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1_r[7]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1_r[8]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1_r[9]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1_r[10]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1_r[11]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1_r[12]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1_r[13]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1_r[14]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_shift_1_l[0]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_1_l[1]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_1_l[2]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_1_l[3]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_1_l[4]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_1_l[5]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_1_l[6]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_1_l[7]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_1_l[8]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_1_l[9]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_1_l[10]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_1_l[11]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_1_l[12]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_1_l[13]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_1_l[14]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_1_l[15]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i32 (.D(mic_data_0_c), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i32.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i33 (.D(mic_shift_0_r[0]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i33.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i34 (.D(mic_shift_0_r[1]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i34.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i35 (.D(mic_shift_0_r[2]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i35.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i36 (.D(mic_shift_0_r[3]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i36.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i37 (.D(mic_shift_0_r[4]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i37.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i38 (.D(mic_shift_0_r[5]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i38.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i39 (.D(mic_shift_0_r[6]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i39.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i40 (.D(mic_shift_0_r[7]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i40.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i41 (.D(mic_shift_0_r[8]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i41.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i42 (.D(mic_shift_0_r[9]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i42.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i43 (.D(mic_shift_0_r[10]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i43.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i44 (.D(mic_shift_0_r[11]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i44.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i45 (.D(mic_shift_0_r[12]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i45.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i46 (.D(mic_shift_0_r[13]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i46.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i47 (.D(mic_shift_0_r[14]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i47.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i48 (.D(mic_shift_0_l[0]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i48.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i49 (.D(mic_shift_0_l[1]), .SP(pll_clk_enable_399), 
            .CK(pll_clk), .Q(mic_latest[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i49.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i50 (.D(mic_shift_0_l[2]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i50.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i51 (.D(mic_shift_0_l[3]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i51.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i52 (.D(mic_shift_0_l[4]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i52.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i53 (.D(mic_shift_0_l[5]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i53.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i54 (.D(mic_shift_0_l[6]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i54.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i55 (.D(mic_shift_0_l[7]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i55.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i56 (.D(mic_shift_0_l[8]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i56.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i57 (.D(mic_shift_0_l[9]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i57.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i58 (.D(mic_shift_0_l[10]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i58.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i59 (.D(mic_shift_0_l[11]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i59.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i60 (.D(mic_shift_0_l[12]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i60.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i61 (.D(mic_shift_0_l[13]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i61.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i62 (.D(mic_shift_0_l[14]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i62.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i63 (.D(mic_shift_0_l[15]), .SP(pll_clk_enable_413), 
            .CK(pll_clk), .Q(mic_latest[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam mic_latest_i0_i63.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i16 (.D(spi_frame_sequence[16]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i17 (.D(spi_frame_sequence[17]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i18 (.D(spi_frame_sequence[18]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i19 (.D(spi_frame_sequence[19]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i20 (.D(spi_frame_sequence[20]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i21 (.D(spi_frame_sequence[21]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i22 (.D(spi_frame_sequence[22]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i23 (.D(spi_frame_sequence[23]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i24 (.D(spi_frame_sequence[24]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i25 (.D(spi_frame_sequence[25]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i26 (.D(spi_frame_sequence[26]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i27 (.D(spi_frame_sequence[27]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i28 (.D(spi_frame_sequence[28]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i29 (.D(spi_frame_sequence[29]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i30 (.D(spi_frame_sequence[30]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i31 (.D(spi_frame_sequence[31]), .SP(spi1_sck_c_enable_317), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam accepted_sequence_spi_i0_i31.GSR = "DISABLED";
    CCU2D expected_next_15__I_0_496_9 (.A0(spi_rx_shift[0]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22005), .COUT(n22006), .S0(expected_next[9]), 
          .S1(expected_next[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_9.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_496_9.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_496_9.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_9.INJECT1_1 = "NO";
    CCU2D fpga_time_1057_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22041), .COUT(n22042), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_29.INJECT1_1 = "NO";
    CCU2D add_499_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21991), .COUT(n21992), .S0(phase_frac_sum[3]), .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_5.INIT0 = 16'h5aaa;
    defparam add_499_5.INIT1 = 16'h5aaa;
    defparam add_499_5.INJECT1_0 = "NO";
    defparam add_499_5.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_496_7 (.A0(expected_next_15__N_1365[7]), .B0(spi_extension_length[7]), 
          .C0(GND_net), .D0(GND_net), .A1(spi1_mosi_c_0), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22004), .COUT(n22005), .S0(expected_next[7]), 
          .S1(expected_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_7.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_496_7.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_496_7.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_7.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_496_5 (.A0(expected_next_15__N_1365[7]), .B0(spi_extension_length[5]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1365[7]), 
          .B1(spi_extension_length[6]), .C1(GND_net), .D1(GND_net), .CIN(n22003), 
          .COUT(n22004), .S0(expected_next[5]), .S1(expected_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_5.INIT0 = 16'ha999;
    defparam expected_next_15__I_0_496_5.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_496_5.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_159 (.A(build_sum[8]), .B(n23529), .C(ev_bit[38]), 
         .D(init_shadow[38]), .Z(n13073)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_159.init = 16'hff80;
    LUT4 i1_3_lut_4_lut_adj_160 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[32]), 
         .D(ev_bit[32]), .Z(ev_wr_data[32])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_160.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_161 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[33]), 
         .D(ev_bit[33]), .Z(ev_wr_data[33])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_161.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_162 (.A(build_sum[8]), .B(n23529), .C(ev_bit[36]), 
         .D(init_shadow[36]), .Z(n13061)) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_3_lut_4_lut_adj_162.init = 16'hff80;
    CCU2D expected_next_15__I_0_496_3 (.A0(expected_next_15__N_1365[7]), .B0(spi_extension_length[3]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1406[3]), 
          .B1(spi_extension_length[4]), .C1(GND_net), .D1(GND_net), .CIN(n22002), 
          .COUT(n22003), .S0(expected_next[3]), .S1(expected_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_3.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_496_3.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_496_3.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_3.INJECT1_1 = "NO";
    LUT4 i3_3_lut_rep_96_4_lut (.A(spi_byte_count[10]), .B(n23541), .C(n23534), 
         .D(n19122), .Z(n23506)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i3_3_lut_rep_96_4_lut.init = 16'hfffe;
    CCU2D expected_next_15__I_0_496_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(expected_next_15__N_1406[3]), .B1(spi_extension_length[2]), 
          .C1(GND_net), .D1(GND_net), .COUT(n22002), .S1(expected_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(272[33] 274[86])
    defparam expected_next_15__I_0_496_1.INIT0 = 16'hF000;
    defparam expected_next_15__I_0_496_1.INIT1 = 16'ha999;
    defparam expected_next_15__I_0_496_1.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_496_1.INJECT1_1 = "NO";
    CCU2D add_135_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_byte_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21978), .S1(spi_byte_count_15__N_1643[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_1.INIT0 = 16'hF000;
    defparam add_135_1.INIT1 = 16'h5555;
    defparam add_135_1.INJECT1_0 = "NO";
    defparam add_135_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_163 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[34]), 
         .D(ev_bit[34]), .Z(ev_wr_data[34])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_163.init = 16'hddd0;
    LUT4 i2_3_lut_rep_93_4_lut (.A(spi_byte_count[10]), .B(n23541), .C(spi_byte_count[9]), 
         .D(n23520), .Z(n23503)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i2_3_lut_rep_93_4_lut.init = 16'hfffe;
    CCU2D add_499_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22001), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_25.INIT0 = 16'h5aaa;
    defparam add_499_25.INIT1 = 16'h0000;
    defparam add_499_25.INJECT1_0 = "NO";
    defparam add_499_25.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_164 (.A(ev_run_hold[1]), .B(us_tx_c_1), .C(init_shadow[1]), 
         .D(swap_now_d3), .Z(n11434)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_164.init = 16'h5a66;
    LUT4 i7_4_lut_adj_165 (.A(ev_run_hold[2]), .B(us_tx_c_2), .C(init_shadow[2]), 
         .D(swap_now_d3), .Z(n11436)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_165.init = 16'h5a66;
    CCU2D fpga_time_1057_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22040), .COUT(n22041), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_27.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_166 (.A(ev_run_hold[3]), .B(us_tx_c_3), .C(init_shadow[3]), 
         .D(swap_now_d3), .Z(n11438)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_166.init = 16'h5a66;
    CCU2D fpga_time_1057_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22039), .COUT(n22040), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_25.INJECT1_1 = "NO";
    CCU2D equal_1855_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n21891), 
          .S0(frame_end_N_2543));
    defparam equal_1855_17.INIT0 = 16'hFFFF;
    defparam equal_1855_17.INIT1 = 16'h0000;
    defparam equal_1855_17.INJECT1_0 = "NO";
    defparam equal_1855_17.INJECT1_1 = "NO";
    CCU2D fpga_time_1057_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22038), .COUT(n22039), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_23.INJECT1_1 = "NO";
    CCU2D equal_1855_11 (.A0(spi_expected_length[15]), .B0(spi_byte_count_15__N_1643[15]), 
          .C0(spi_expected_length[14]), .D0(spi_byte_count_15__N_1643[14]), 
          .A1(spi_expected_length[13]), .B1(spi_byte_count_15__N_1643[13]), 
          .C1(spi_expected_length[12]), .D1(spi_byte_count_15__N_1643[12]), 
          .CIN(n21887), .COUT(n21888));
    defparam equal_1855_11.INIT0 = 16'h9009;
    defparam equal_1855_11.INIT1 = 16'h9009;
    defparam equal_1855_11.INJECT1_0 = "YES";
    defparam equal_1855_11.INJECT1_1 = "YES";
    LUT4 i1_2_lut_adj_167 (.A(active_bank), .B(swap_now_d1), .Z(active_bank_N_859)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:69])
    defparam i1_2_lut_adj_167.init = 16'h6666;
    spi_mic_stream mic_stream_i (.sck_N_2908(sck_N_2908), .spi_mic_cs_n_c(spi_mic_cs_n_c), 
            .mic_latest({mic_latest}), .spi_mic_miso_c(spi_mic_miso_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(528[20] 531[6])
    FD1P3AX status_hold__i2 (.D(accepted_sequence[25]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i2.GSR = "DISABLED";
    LUT4 i3_3_lut_4_lut (.A(spi_byte_count[15]), .B(n23540), .C(spi_byte_count[5]), 
         .D(n23506), .Z(n8_adj_3051)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i3_3_lut_4_lut.init = 16'hffef;
    LUT4 i1_3_lut_4_lut_adj_168 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[35]), 
         .D(ev_bit[35]), .Z(ev_wr_data[35])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_168.init = 16'hddd0;
    CCU2D fpga_time_1057_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22037), .COUT(n22038), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_21.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_169 (.A(ev_run_hold[4]), .B(us_tx_c_4), .C(init_shadow[4]), 
         .D(swap_now_d3), .Z(n11440)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_169.init = 16'h5a66;
    LUT4 run_addr_reg_8__I_0_i2_3_lut (.A(global_phase[1]), .B(ev_rd_slot[1]), 
         .C(n19130), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i2_3_lut.init = 16'hacac;
    LUT4 i1_2_lut_3_lut_4_lut_adj_170 (.A(spi_bit_count[2]), .B(n23542), 
         .C(n23533), .D(n23494), .Z(n4)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_2_lut_3_lut_4_lut_adj_170.init = 16'h0008;
    LUT4 i1_3_lut_4_lut_adj_171 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[36]), 
         .D(ev_bit[36]), .Z(ev_wr_data[36])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_171.init = 16'hddd0;
    CCU2D equal_1855_17_13456 (.A0(spi_expected_length[3]), .B0(spi_byte_count_15__N_1643[3]), 
          .C0(spi_expected_length[2]), .D0(spi_byte_count_15__N_1643[2]), 
          .A1(spi_expected_length[1]), .B1(spi_byte_count_15__N_1643[1]), 
          .C1(spi_expected_length[0]), .D1(spi_byte_count_15__N_1643[0]), 
          .CIN(n21890), .COUT(n21891));
    defparam equal_1855_17_13456.INIT0 = 16'h9009;
    defparam equal_1855_17_13456.INIT1 = 16'h9009;
    defparam equal_1855_17_13456.INJECT1_0 = "YES";
    defparam equal_1855_17_13456.INJECT1_1 = "YES";
    CCU2D global_phase_7__I_0_445_2 (.A0(global_phase[0]), .B0(phase_step_reg), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .COUT(n21969), .S1(next_global_phase[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[37:56])
    defparam global_phase_7__I_0_445_2.INIT0 = 16'h7000;
    defparam global_phase_7__I_0_445_2.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_445_2.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_445_2.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_172 (.A(ev_run_hold[5]), .B(us_tx_c_5), .C(init_shadow[5]), 
         .D(swap_now_d3), .Z(n11442)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_172.init = 16'h5a66;
    LUT4 i2_3_lut_4_lut (.A(spi_bit_count[2]), .B(n23542), .C(spi_byte_count[15]), 
         .D(n23540), .Z(n22660)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i2_3_lut_4_lut.init = 16'h0008;
    LUT4 i1_3_lut_rep_79_4_lut (.A(spi_bit_count[2]), .B(n23542), .C(fpga_cs_n_c), 
         .D(frame_end), .Z(spi1_sck_c_enable_28)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_3_lut_rep_79_4_lut.init = 16'h0800;
    CCU2D fpga_time_1057_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22036), .COUT(n22037), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_19.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_4_lut_adj_173 (.A(ev_ch[3]), .B(n23545), .C(n14801), 
         .D(ev_ch[4]), .Z(n14566)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_173.init = 16'h4000;
    LUT4 i7_4_lut_adj_174 (.A(ev_run_hold[6]), .B(us_tx_c_6), .C(init_shadow[6]), 
         .D(swap_now_d3), .Z(n11444)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_174.init = 16'h5a66;
    LUT4 i1_2_lut_3_lut_4_lut_adj_175 (.A(ev_ch[3]), .B(n23545), .C(n23548), 
         .D(ev_ch[4]), .Z(n14185)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_175.init = 16'h4000;
    CCU2D fpga_time_1057_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22035), .COUT(n22036), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_17.INJECT1_1 = "NO";
    CCU2D add_501_2 (.A0(staging_q[8]), .B0(staging_q[0]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[9]), .B1(staging_q[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n21974), .S1(build_sum_8__N_2005[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_501_2.INIT0 = 16'h7000;
    defparam add_501_2.INIT1 = 16'h5666;
    defparam add_501_2.INJECT1_0 = "NO";
    defparam add_501_2.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_176 (.A(ev_run_hold[7]), .B(us_tx_c_7), .C(init_shadow[7]), 
         .D(swap_now_d3), .Z(n11446)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_176.init = 16'h5a66;
    LUT4 i1_2_lut_3_lut_4_lut_adj_177 (.A(ev_ch[3]), .B(n23545), .C(n23546), 
         .D(ev_ch[4]), .Z(n14624)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_177.init = 16'h0400;
    LUT4 run_addr_reg_8__I_0_i3_3_lut (.A(global_phase[2]), .B(ev_rd_slot[2]), 
         .C(n19130), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i3_3_lut.init = 16'hacac;
    LUT4 i1_2_lut_3_lut_4_lut_adj_178 (.A(ev_ch[3]), .B(n23545), .C(n23546), 
         .D(ev_ch[4]), .Z(n14550)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_178.init = 16'h0004;
    LUT4 i1_2_lut_3_lut_4_lut_adj_179 (.A(ev_ch[3]), .B(n23545), .C(n23548), 
         .D(ev_ch[4]), .Z(n14173)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_179.init = 16'h0040;
    LUT4 i7_4_lut_adj_180 (.A(ev_run_hold[8]), .B(us_tx_c_8), .C(init_shadow[8]), 
         .D(swap_now_d3), .Z(n11448)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_180.init = 16'h5a66;
    LUT4 i1_2_lut_3_lut_4_lut_adj_181 (.A(ev_ch[3]), .B(n23545), .C(n14801), 
         .D(ev_ch[4]), .Z(n14612)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_181.init = 16'h0040;
    LUT4 i7_4_lut_adj_182 (.A(ev_run_hold[9]), .B(us_tx_c_9), .C(init_shadow[9]), 
         .D(swap_now_d3), .Z(n11450)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_182.init = 16'h5a66;
    LUT4 run_addr_reg_8__I_0_i4_3_lut (.A(global_phase[3]), .B(ev_rd_slot[3]), 
         .C(n19130), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i4_3_lut.init = 16'hacac;
    LUT4 i1_2_lut_adj_183 (.A(ev_state[1]), .B(ev_state_3__N_1964[2]), .Z(n28_adj_3041)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i1_2_lut_adj_183.init = 16'h2222;
    LUT4 i1_4_lut_adj_184 (.A(n5_adj_3067), .B(spi1_sck_c_enable_28), .C(spi_command[4]), 
         .D(n22739), .Z(spi1_sck_c_enable_27)) /* synthesis lut_function=(A (B)+!A (B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_4_lut_adj_184.init = 16'hccc8;
    LUT4 run_addr_reg_8__I_0_i5_3_lut (.A(global_phase[4]), .B(ev_rd_slot[4]), 
         .C(n19130), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i5_3_lut.init = 16'hacac;
    CCU2D equal_1855_15 (.A0(spi_expected_length[7]), .B0(spi_byte_count_15__N_1643[7]), 
          .C0(spi_expected_length[6]), .D0(spi_byte_count_15__N_1643[6]), 
          .A1(spi_expected_length[5]), .B1(spi_byte_count_15__N_1643[5]), 
          .C1(spi_expected_length[4]), .D1(spi_byte_count_15__N_1643[4]), 
          .CIN(n21889), .COUT(n21890));
    defparam equal_1855_15.INIT0 = 16'h9009;
    defparam equal_1855_15.INIT1 = 16'h9009;
    defparam equal_1855_15.INJECT1_0 = "YES";
    defparam equal_1855_15.INJECT1_1 = "YES";
    CCU2D equal_1855_13 (.A0(spi_expected_length[11]), .B0(spi_byte_count_15__N_1643[11]), 
          .C0(spi_expected_length[10]), .D0(spi_byte_count_15__N_1643[10]), 
          .A1(spi_expected_length[9]), .B1(spi_byte_count_15__N_1643[9]), 
          .C1(spi_expected_length[8]), .D1(spi_byte_count_15__N_1643[8]), 
          .CIN(n21888), .COUT(n21889));
    defparam equal_1855_13.INIT0 = 16'h9009;
    defparam equal_1855_13.INIT1 = 16'h9009;
    defparam equal_1855_13.INJECT1_0 = "YES";
    defparam equal_1855_13.INJECT1_1 = "YES";
    LUT4 i14676_2_lut_3_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23544), .C(ev_ch[4]), 
         .D(n23551), .Z(n14580)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14676_2_lut_3_lut_3_lut_4_lut.init = 16'h0001;
    LUT4 i14632_2_lut_3_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23544), .C(ev_ch[4]), 
         .D(n23546), .Z(n14390)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14632_2_lut_3_lut_3_lut_4_lut.init = 16'h0001;
    LUT4 i14622_2_lut_3_lut_4_lut_4_lut (.A(n23551), .B(ev_ch[4]), .C(n23544), 
         .D(ev_ch[3]), .Z(n14596)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i14622_2_lut_3_lut_4_lut_4_lut.init = 16'h0004;
    LUT4 i1_2_lut_3_lut_4_lut_adj_185 (.A(ev_ch[3]), .B(n23547), .C(n23548), 
         .D(ev_ch[4]), .Z(n14181)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_185.init = 16'h0080;
    LUT4 i1_2_lut_3_lut_4_lut_adj_186 (.A(ev_ch[3]), .B(n23547), .C(n14801), 
         .D(ev_ch[4]), .Z(n14524)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_186.init = 16'h0080;
    LUT4 i1_2_lut_3_lut_4_lut_adj_187 (.A(ev_ch[3]), .B(n23547), .C(n23546), 
         .D(ev_ch[4]), .Z(n14622)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_187.init = 16'h0008;
    LUT4 i1_2_lut_3_lut_4_lut_adj_188 (.A(ev_ch[3]), .B(n23547), .C(n23548), 
         .D(ev_ch[4]), .Z(n14408)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_188.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_189 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[37]), 
         .D(ev_bit[37]), .Z(ev_wr_data[37])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_189.init = 16'hddd0;
    LUT4 i14670_2_lut_3_lut_4_lut_4_lut (.A(n23546), .B(ev_ch[4]), .C(n23544), 
         .D(ev_ch[3]), .Z(n14387)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i14670_2_lut_3_lut_4_lut_4_lut.init = 16'h0004;
    LUT4 i14667_2_lut_3_lut_4_lut_4_lut (.A(n23528), .B(ev_ch[4]), .C(n23544), 
         .D(ev_ch[3]), .Z(n14345)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i14667_2_lut_3_lut_4_lut_4_lut.init = 16'h0004;
    LUT4 i1_2_lut_3_lut_4_lut_adj_190 (.A(n23545), .B(ev_ch[3]), .C(n23548), 
         .D(ev_ch[4]), .Z(n14178)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_190.init = 16'h0080;
    LUT4 i1_2_lut_3_lut_4_lut_adj_191 (.A(n23545), .B(ev_ch[3]), .C(n14801), 
         .D(ev_ch[4]), .Z(n14552)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_191.init = 16'h0080;
    PFUMX i14741 (.BLUT(n23324), .ALUT(n23323), .C0(ev_state[3]), .Z(ev_state_3__N_645[2]));
    LUT4 i1_2_lut_3_lut_4_lut_adj_192 (.A(n23545), .B(ev_ch[3]), .C(n23546), 
         .D(ev_ch[4]), .Z(n14546)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_192.init = 16'h0008;
    LUT4 i1_2_lut_3_lut_4_lut_adj_193 (.A(n23545), .B(ev_ch[3]), .C(n23548), 
         .D(ev_ch[4]), .Z(n14446)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_193.init = 16'h8000;
    LUT4 i14661_2_lut_3_lut_4_lut_4_lut (.A(n23546), .B(ev_ch[4]), .C(ev_ch[3]), 
         .D(n23544), .Z(n14385)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i14661_2_lut_3_lut_4_lut_4_lut.init = 16'h0010;
    LUT4 i1_2_lut_3_lut_4_lut_adj_194 (.A(ev_ch[3]), .B(n23547), .C(n14801), 
         .D(ev_ch[4]), .Z(n14616)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_194.init = 16'h0040;
    LUT4 i1_2_lut_3_lut_4_lut_adj_195 (.A(ev_ch[3]), .B(n23547), .C(n23548), 
         .D(ev_ch[4]), .Z(n14175)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_195.init = 16'h0040;
    LUT4 i1_3_lut_4_lut_adj_196 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[38]), 
         .D(ev_bit[38]), .Z(ev_wr_data[38])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_196.init = 16'hddd0;
    CCU2D add_499_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22000), .COUT(n22001), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_23.INIT0 = 16'h5aaa;
    defparam add_499_23.INIT1 = 16'h5aaa;
    defparam add_499_23.INJECT1_0 = "NO";
    defparam add_499_23.INJECT1_1 = "NO";
    CCU2D fpga_time_1057_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22034), .COUT(n22035), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_15.INJECT1_1 = "NO";
    LUT4 i2_4_lut_adj_197 (.A(n19458), .B(spi1_sck_c_enable_286), .C(n23534), 
         .D(n4_adj_3064), .Z(n22667)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i2_4_lut_adj_197.init = 16'h0004;
    LUT4 i1_4_lut_adj_198 (.A(spi_command[1]), .B(n22739), .C(spi_command[4]), 
         .D(spi_command[0]), .Z(n22099)) /* synthesis lut_function=(A (B+((D)+!C))+!A (B+!(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_4_lut_adj_198.init = 16'hefdf;
    CCU2D add_499_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21990), .COUT(n21991), .S0(phase_frac_sum[1]), .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_3.INIT0 = 16'h5aaa;
    defparam add_499_3.INIT1 = 16'h5aaa;
    defparam add_499_3.INJECT1_0 = "NO";
    defparam add_499_3.INJECT1_1 = "NO";
    CCU2D add_501_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n21977), 
          .S0(build_sum_8__N_2005[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_501_cout.INIT0 = 16'h0000;
    defparam add_501_cout.INIT1 = 16'h0000;
    defparam add_501_cout.INJECT1_0 = "NO";
    defparam add_501_cout.INJECT1_1 = "NO";
    CCU2D add_499_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21990), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_1.INIT0 = 16'hF000;
    defparam add_499_1.INIT1 = 16'h5555;
    defparam add_499_1.INJECT1_0 = "NO";
    defparam add_499_1.INJECT1_1 = "NO";
    CCU2D add_499_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21999), .COUT(n22000), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_21.INIT0 = 16'h5aaa;
    defparam add_499_21.INIT1 = 16'h5555;
    defparam add_499_21.INJECT1_0 = "NO";
    defparam add_499_21.INJECT1_1 = "NO";
    CCU2D add_242_9 (.A0(ev_clear_addr[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21989), .S0(ev_clear_addr_7__N_2189[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_9.INIT0 = 16'h5aaa;
    defparam add_242_9.INIT1 = 16'h0000;
    defparam add_242_9.INJECT1_0 = "NO";
    defparam add_242_9.INJECT1_1 = "NO";
    CCU2D add_242_7 (.A0(ev_clear_addr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21988), .COUT(n21989), .S0(ev_clear_addr_7__N_2189[5]), 
          .S1(ev_clear_addr_7__N_2189[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_7.INIT0 = 16'h5aaa;
    defparam add_242_7.INIT1 = 16'h5aaa;
    defparam add_242_7.INJECT1_0 = "NO";
    defparam add_242_7.INJECT1_1 = "NO";
    CCU2D add_242_5 (.A0(ev_clear_addr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21987), .COUT(n21988), .S0(ev_clear_addr_7__N_2189[3]), 
          .S1(ev_clear_addr_7__N_2189[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_5.INIT0 = 16'h5aaa;
    defparam add_242_5.INIT1 = 16'h5aaa;
    defparam add_242_5.INJECT1_0 = "NO";
    defparam add_242_5.INJECT1_1 = "NO";
    CCU2D add_499_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21998), .COUT(n21999), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_19.INIT0 = 16'h5aaa;
    defparam add_499_19.INIT1 = 16'h5555;
    defparam add_499_19.INJECT1_0 = "NO";
    defparam add_499_19.INJECT1_1 = "NO";
    CCU2D add_499_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21997), .COUT(n21998), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_17.INIT0 = 16'h5aaa;
    defparam add_499_17.INIT1 = 16'h5aaa;
    defparam add_499_17.INJECT1_0 = "NO";
    defparam add_499_17.INJECT1_1 = "NO";
    CCU2D add_242_3 (.A0(ev_clear_addr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21986), .COUT(n21987), .S0(ev_clear_addr_7__N_2189[1]), 
          .S1(ev_clear_addr_7__N_2189[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_3.INIT0 = 16'h5aaa;
    defparam add_242_3.INIT1 = 16'h5aaa;
    defparam add_242_3.INJECT1_0 = "NO";
    defparam add_242_3.INJECT1_1 = "NO";
    LUT4 i1_2_lut_adj_199 (.A(n24), .B(spi_command[7]), .Z(n22739)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_2_lut_adj_199.init = 16'heeee;
    CCU2D add_242_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(ev_clear_addr[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21986), .S1(ev_clear_addr_7__N_2189[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_1.INIT0 = 16'hF000;
    defparam add_242_1.INIT1 = 16'h5555;
    defparam add_242_1.INJECT1_0 = "NO";
    defparam add_242_1.INJECT1_1 = "NO";
    CCU2D add_135_17 (.A0(spi_byte_count[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21985), .S0(spi_byte_count_15__N_1643[15]), .S1(frame_end_N_2544[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_17.INIT0 = 16'h5aaa;
    defparam add_135_17.INIT1 = 16'h0000;
    defparam add_135_17.INJECT1_0 = "NO";
    defparam add_135_17.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_4_lut_adj_200 (.A(ev_ch[3]), .B(n23547), .C(n23546), 
         .D(ev_ch[4]), .Z(n14568)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_200.init = 16'h0004;
    LUT4 i1_2_lut_3_lut_4_lut_adj_201 (.A(ev_ch[3]), .B(n23547), .C(n23548), 
         .D(ev_ch[4]), .Z(n14404)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_3_lut_4_lut_adj_201.init = 16'h4000;
    LUT4 i14640_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23549), .C(n23551), 
         .D(ev_ch[4]), .Z(n14608)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14640_2_lut_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i14619_2_lut_3_lut_4_lut_4_lut (.A(n23551), .B(ev_ch[4]), .C(n23549), 
         .D(ev_ch[3]), .Z(n14592)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i14619_2_lut_3_lut_4_lut_4_lut.init = 16'h0100;
    LUT4 frame_toggle_spi_I_0_2_lut (.A(frame_toggle_spi), .B(n23491), .Z(frame_toggle_spi_N_2476)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[30] 337[24])
    defparam frame_toggle_spi_I_0_2_lut.init = 16'h6666;
    LUT4 run_addr_reg_8__I_0_i6_3_lut (.A(global_phase[5]), .B(ev_rd_slot[5]), 
         .C(n19130), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i6_3_lut.init = 16'hacac;
    PFUMX i14734 (.BLUT(n23313), .ALUT(n23312), .C0(ev_state[2]), .Z(n23314));
    LUT4 mux_931_i1_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[0]), 
         .D(ev_clear_addr[0]), .Z(ev_wr_addr[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i1_3_lut_rep_95_4_lut (.A(ev_state[1]), .B(n23560), .C(ev_state[0]), 
         .D(n23539), .Z(pll_clk_enable_477)) /* synthesis lut_function=(!(A+(B+!(C+(D))))) */ ;
    defparam i1_3_lut_rep_95_4_lut.init = 16'h1110;
    LUT4 i7_4_lut_adj_202 (.A(ev_run_hold[10]), .B(us_tx_c_10), .C(init_shadow[10]), 
         .D(swap_now_d3), .Z(n11452)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_202.init = 16'h5a66;
    CCU2D equal_1855_0 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(frame_end_N_2544[16]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21887));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(277[48:95])
    defparam equal_1855_0.INIT0 = 16'hF000;
    defparam equal_1855_0.INIT1 = 16'h5555;
    defparam equal_1855_0.INJECT1_0 = "NO";
    defparam equal_1855_0.INJECT1_1 = "YES";
    LUT4 i7266_2_lut_3_lut_4_lut (.A(ev_state[1]), .B(n23560), .C(ev_state[0]), 
         .D(n23539), .Z(n15876)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i7266_2_lut_3_lut_4_lut.init = 16'h0100;
    LUT4 i7_4_lut_adj_203 (.A(ev_run_hold[11]), .B(us_tx_c_11), .C(init_shadow[11]), 
         .D(swap_now_d3), .Z(n11454)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_203.init = 16'h5a66;
    LUT4 mux_931_i2_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[1]), 
         .D(ev_clear_addr[1]), .Z(ev_wr_addr[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i7_4_lut_adj_204 (.A(ev_run_hold[12]), .B(us_tx_c_12), .C(init_shadow[12]), 
         .D(swap_now_d3), .Z(n11456)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_204.init = 16'h5a66;
    LUT4 i14691_2_lut_rep_80_2_lut (.A(n23491), .B(n22099), .Z(n23490)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(332[30] 337[24])
    defparam i14691_2_lut_rep_80_2_lut.init = 16'h4444;
    LUT4 i2_4_lut_rep_81 (.A(n23002), .B(spi_command[4]), .C(n44), .D(spi_command[0]), 
         .Z(n23491)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i2_4_lut_rep_81.init = 16'h0040;
    LUT4 i7_4_lut_adj_205 (.A(ev_run_hold[13]), .B(us_tx_c_13), .C(init_shadow[13]), 
         .D(swap_now_d3), .Z(n11458)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_205.init = 16'h5a66;
    CCU2D add_501_6 (.A0(staging_q[12]), .B0(staging_q[4]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[13]), .B1(staging_q[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n21975), .COUT(n21976), .S0(build_sum_8__N_2005[4]), 
          .S1(build_sum_8__N_2005[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_501_6.INIT0 = 16'h5666;
    defparam add_501_6.INIT1 = 16'h5666;
    defparam add_501_6.INJECT1_0 = "NO";
    defparam add_501_6.INJECT1_1 = "NO";
    CCU2D add_499_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21996), .COUT(n21997), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_15.INIT0 = 16'h5555;
    defparam add_499_15.INIT1 = 16'h5555;
    defparam add_499_15.INJECT1_0 = "NO";
    defparam add_499_15.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_90_3_lut_4_lut (.A(ev_state[1]), .B(n23560), .C(n23539), 
         .D(ev_state[0]), .Z(pll_clk_enable_307)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i1_2_lut_rep_90_3_lut_4_lut.init = 16'h0010;
    LUT4 i14658_2_lut_3_lut_4_lut_4_lut (.A(n23528), .B(ev_ch[4]), .C(n23549), 
         .D(ev_ch[3]), .Z(n14347)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i14658_2_lut_3_lut_4_lut_4_lut.init = 16'h0100;
    LUT4 i1_2_lut_3_lut (.A(spi_channel_field[1]), .B(spi1_sck_c_enable_325), 
         .C(spi_channel_field[0]), .Z(spi1_sck_c_enable_89)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i1_2_lut_3_lut.init = 16'h0404;
    LUT4 i14637_2_lut_3_lut_4_lut_4_lut (.A(n23546), .B(ev_ch[4]), .C(n23549), 
         .D(ev_ch[3]), .Z(n14359)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i14637_2_lut_3_lut_4_lut_4_lut.init = 16'h0100;
    FD1P3IX init_shadow_i4 (.D(n12790), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i4.GSR = "DISABLED";
    CCU2D add_135_15 (.A0(spi_byte_count[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21984), .COUT(n21985), .S0(spi_byte_count_15__N_1643[13]), 
          .S1(spi_byte_count_15__N_1643[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_15.INIT0 = 16'h5aaa;
    defparam add_135_15.INIT1 = 16'h5aaa;
    defparam add_135_15.INJECT1_0 = "NO";
    defparam add_135_15.INJECT1_1 = "NO";
    FD1P3AX status_hold__i3 (.D(accepted_sequence[26]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[27]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[28]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[29]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[30]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[31]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[16]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[17]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[18]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[19]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[20]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[21]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[22]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[23]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(accepted_sequence[8]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(accepted_sequence[9]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(accepted_sequence[10]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(accepted_sequence[11]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(accepted_sequence[12]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(accepted_sequence[13]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(accepted_sequence[14]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(accepted_sequence[15]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(accepted_sequence[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(accepted_sequence[1]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(accepted_sequence[2]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(accepted_sequence[3]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(accepted_sequence[4]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(accepted_sequence[5]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(accepted_sequence[6]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(accepted_sequence[7]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[24]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[25]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[26]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[27]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[28]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[29]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[30]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[31]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[16]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[17]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[18]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[19]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[20]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[21]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[22]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[23]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(fpga_time[8]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(fpga_time[9]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(fpga_time[10]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(fpga_time[11]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(fpga_time[12]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(fpga_time[13]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(fpga_time[14]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(fpga_time[15]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(fpga_time[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(fpga_time[1]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(fpga_time[2]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(fpga_time[3]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(fpga_time[4]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(fpga_time[5]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX status_hold__i63 (.D(fpga_time[6]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i63.GSR = "DISABLED";
    FD1P3AX status_hold__i64 (.D(fpga_time[7]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i64.GSR = "DISABLED";
    FD1P3AX status_hold__i65 (.D(status_flags_wire_15__N_1333[2]), .SP(cs_fall), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i65.GSR = "DISABLED";
    FD1P3AX status_hold__i66 (.D(status_flags_wire_15__N_1349[4]), .SP(cs_fall), 
            .CK(pll_clk), .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i66.GSR = "DISABLED";
    FD1P3AX status_hold__i67 (.D(n23511), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i67.GSR = "DISABLED";
    FD1P3AX status_hold__i68 (.D(fifo_credit_wire[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam status_hold__i68.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i1 (.D(pending_sequence[1]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i2 (.D(pending_sequence[2]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i3 (.D(pending_sequence[3]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i4 (.D(pending_sequence[4]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i5 (.D(pending_sequence[5]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i6 (.D(pending_sequence[6]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i7 (.D(pending_sequence[7]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i8 (.D(pending_sequence[8]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i9 (.D(pending_sequence[9]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i10 (.D(pending_sequence[10]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i11 (.D(pending_sequence[11]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i12 (.D(pending_sequence[12]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i13 (.D(pending_sequence[13]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i14 (.D(pending_sequence[14]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i15 (.D(pending_sequence[15]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i16 (.D(pending_sequence[16]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i17 (.D(pending_sequence[17]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i18 (.D(pending_sequence[18]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i19 (.D(pending_sequence[19]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i20 (.D(pending_sequence[20]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i21 (.D(pending_sequence[21]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i22 (.D(pending_sequence[22]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i23 (.D(pending_sequence[23]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i24 (.D(pending_sequence[24]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i25 (.D(pending_sequence[25]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i26 (.D(pending_sequence[26]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i27 (.D(pending_sequence[27]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i28 (.D(pending_sequence[28]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i29 (.D(pending_sequence[29]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i30 (.D(pending_sequence[30]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i31 (.D(pending_sequence[31]), .SP(pll_clk_enable_445), 
            .CK(pll_clk), .Q(accepted_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam accepted_sequence_i0_i31.GSR = "DISABLED";
    FD1P3IX init_shadow_i3 (.D(n12784), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i3.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_206 (.A(ev_run_hold[14]), .B(us_tx_c_14), .C(init_shadow[14]), 
         .D(swap_now_d3), .Z(n11460)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_206.init = 16'h5a66;
    CCU2D global_phase_7__I_0_445_8 (.A0(global_phase[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[7]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21971), .S0(next_global_phase[6]), 
          .S1(next_global_phase[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[37:56])
    defparam global_phase_7__I_0_445_8.INIT0 = 16'h5aaa;
    defparam global_phase_7__I_0_445_8.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_445_8.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_445_8.INJECT1_1 = "NO";
    LUT4 mux_931_i3_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[2]), 
         .D(ev_clear_addr[2]), .Z(ev_wr_addr[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i14655_2_lut_3_lut_4_lut_4_lut (.A(n23528), .B(ev_ch[4]), .C(ev_ch[3]), 
         .D(n23544), .Z(n14383)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i14655_2_lut_3_lut_4_lut_4_lut.init = 16'h0010;
    LUT4 mux_931_i4_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[3]), 
         .D(ev_clear_addr[3]), .Z(ev_wr_addr[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i4_3_lut_4_lut.init = 16'hf2d0;
    FD1S3IX ev_bit_i21 (.D(n14423), .CK(pll_clk), .CD(n23555), .Q(ev_bit[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i21.GSR = "DISABLED";
    FD1S3IX ev_bit_i25 (.D(n14425), .CK(pll_clk), .CD(n23555), .Q(ev_bit[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i25.GSR = "DISABLED";
    FD1S3IX ev_bit_i60 (.D(n14608), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i60.GSR = "DISABLED";
    FD1S3IX ev_bit_i12 (.D(n14359), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i12.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i0 (.D(ev_clear_addr_7__N_2189[0]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i0.GSR = "DISABLED";
    FD1P3IX init_shadow_i0 (.D(n11417), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i0.GSR = "DISABLED";
    FD1P3AX ev_state_i0 (.D(ev_state_3__N_645[0]), .SP(pll_clk_enable_449), 
            .CK(pll_clk), .Q(ev_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_state_i0.GSR = "DISABLED";
    FD1P3IX init_shadow_i2 (.D(n12778), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i2.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i3 (.D(ev_clear_addr_7__N_2189[3]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i3.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i2 (.D(ev_clear_addr_7__N_2189[2]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i2.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_207 (.A(ev_run_hold[15]), .B(us_tx_c_15), .C(init_shadow[15]), 
         .D(swap_now_d3), .Z(n11462)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_207.init = 16'h5a66;
    LUT4 i14646_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[4]), .B(n23546), .C(n23549), 
         .D(ev_ch[3]), .Z(n14423)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14646_2_lut_2_lut_3_lut_4_lut.init = 16'h0002;
    FD1P3IX ev_clear_addr_i1 (.D(ev_clear_addr_7__N_2189[1]), .SP(pll_clk_enable_477), 
            .CD(n15876), .CK(pll_clk), .Q(ev_clear_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_clear_addr_i1.GSR = "DISABLED";
    FD1P3IX init_shadow_i83 (.D(n13361), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i83.GSR = "DISABLED";
    FD1P3IX init_shadow_i82 (.D(n13355), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i82.GSR = "DISABLED";
    FD1P3IX init_shadow_i81 (.D(n13349), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i81.GSR = "DISABLED";
    FD1P3IX init_shadow_i80 (.D(n13343), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i80.GSR = "DISABLED";
    FD1P3IX init_shadow_i79 (.D(n13337), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i79.GSR = "DISABLED";
    FD1P3IX init_shadow_i78 (.D(n13331), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i78.GSR = "DISABLED";
    FD1P3IX init_shadow_i77 (.D(n13325), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i77.GSR = "DISABLED";
    FD1P3IX init_shadow_i76 (.D(n13319), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i76.GSR = "DISABLED";
    FD1P3IX init_shadow_i75 (.D(n13313), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i75.GSR = "DISABLED";
    FD1P3IX init_shadow_i74 (.D(n13307), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i74.GSR = "DISABLED";
    FD1P3IX init_shadow_i73 (.D(n13301), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i73.GSR = "DISABLED";
    FD1P3IX init_shadow_i72 (.D(n13295), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i72.GSR = "DISABLED";
    FD1P3IX init_shadow_i71 (.D(n13289), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i71.GSR = "DISABLED";
    FD1P3IX init_shadow_i70 (.D(n13283), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i70.GSR = "DISABLED";
    FD1P3IX init_shadow_i69 (.D(n13277), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i69.GSR = "DISABLED";
    FD1P3IX init_shadow_i68 (.D(n13271), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i68.GSR = "DISABLED";
    FD1P3IX init_shadow_i67 (.D(n13265), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i67.GSR = "DISABLED";
    FD1P3IX init_shadow_i66 (.D(n13259), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i66.GSR = "DISABLED";
    FD1P3IX init_shadow_i65 (.D(n13253), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i65.GSR = "DISABLED";
    FD1P3IX init_shadow_i64 (.D(n13247), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i64.GSR = "DISABLED";
    FD1P3IX init_shadow_i63 (.D(n13241), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i63.GSR = "DISABLED";
    FD1P3IX init_shadow_i62 (.D(n13231), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i62.GSR = "DISABLED";
    FD1P3IX init_shadow_i61 (.D(n13223), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i61.GSR = "DISABLED";
    FD1P3IX init_shadow_i60 (.D(n13213), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i60.GSR = "DISABLED";
    FD1P3IX init_shadow_i59 (.D(n13207), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i59.GSR = "DISABLED";
    FD1P3IX init_shadow_i58 (.D(n13201), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i58.GSR = "DISABLED";
    FD1P3IX init_shadow_i57 (.D(n13195), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i57.GSR = "DISABLED";
    FD1P3IX init_shadow_i56 (.D(n13189), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i56.GSR = "DISABLED";
    FD1P3IX init_shadow_i55 (.D(n13183), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i55.GSR = "DISABLED";
    FD1P3IX init_shadow_i54 (.D(n13177), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i54.GSR = "DISABLED";
    FD1P3IX init_shadow_i53 (.D(n13171), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i53.GSR = "DISABLED";
    FD1P3IX init_shadow_i52 (.D(n13165), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i52.GSR = "DISABLED";
    FD1P3IX init_shadow_i51 (.D(n13159), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i51.GSR = "DISABLED";
    FD1P3IX init_shadow_i50 (.D(n13153), .SP(pll_clk_enable_511), .CD(n15793), 
            .CK(pll_clk), .Q(init_shadow[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i50.GSR = "DISABLED";
    FD1P3IX init_shadow_i49 (.D(n13147), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i49.GSR = "DISABLED";
    FD1P3IX init_shadow_i48 (.D(n13141), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i48.GSR = "DISABLED";
    FD1P3IX init_shadow_i47 (.D(n13135), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i47.GSR = "DISABLED";
    FD1P3IX init_shadow_i46 (.D(n13129), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i46.GSR = "DISABLED";
    FD1P3IX init_shadow_i45 (.D(n13121), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i45.GSR = "DISABLED";
    FD1P3IX init_shadow_i44 (.D(n13115), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i44.GSR = "DISABLED";
    FD1P3IX init_shadow_i43 (.D(n13109), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i43.GSR = "DISABLED";
    FD1P3IX init_shadow_i42 (.D(n13103), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i42.GSR = "DISABLED";
    FD1P3IX init_shadow_i12 (.D(n12840), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i12.GSR = "DISABLED";
    LUT4 i14673_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[4]), .B(n23546), .C(n23549), 
         .D(ev_ch[3]), .Z(n14427)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14673_2_lut_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i14643_2_lut_2_lut_3_lut_4_lut (.A(ev_ch[4]), .B(n23546), .C(ev_ch[3]), 
         .D(n23544), .Z(n14425)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i14643_2_lut_2_lut_3_lut_4_lut.init = 16'h0020;
    LUT4 mux_931_i5_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[4]), 
         .D(ev_clear_addr[4]), .Z(ev_wr_addr[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i5_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i11 (.D(n12834), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i11.GSR = "DISABLED";
    FD1P3IX init_shadow_i10 (.D(n12826), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i10.GSR = "DISABLED";
    FD1P3IX init_shadow_i9 (.D(n12820), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i8 (.D(n12814), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i8.GSR = "DISABLED";
    LUT4 mux_931_i6_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[5]), 
         .D(ev_clear_addr[5]), .Z(ev_wr_addr[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i6_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i7_4_lut_adj_208 (.A(ev_run_hold[16]), .B(us_tx_c_16), .C(init_shadow[16]), 
         .D(swap_now_d3), .Z(n11464)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_208.init = 16'h5a66;
    CCU2D add_135_13 (.A0(spi_byte_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21983), .COUT(n21984), .S0(spi_byte_count_15__N_1643[11]), 
          .S1(spi_byte_count_15__N_1643[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_13.INIT0 = 16'h5aaa;
    defparam add_135_13.INIT1 = 16'h5aaa;
    defparam add_135_13.INJECT1_0 = "NO";
    defparam add_135_13.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_209 (.A(ev_run_hold[17]), .B(us_tx_c_17), .C(init_shadow[17]), 
         .D(swap_now_d3), .Z(n11466)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_209.init = 16'h5a66;
    CCU2D fpga_time_1057_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22033), .COUT(n22034), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_13.INJECT1_1 = "NO";
    CCU2D add_135_11 (.A0(spi_byte_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21982), .COUT(n21983), .S0(spi_byte_count_15__N_1643[9]), 
          .S1(spi_byte_count_15__N_1643[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_11.INIT0 = 16'h5aaa;
    defparam add_135_11.INIT1 = 16'h5aaa;
    defparam add_135_11.INJECT1_0 = "NO";
    defparam add_135_11.INJECT1_1 = "NO";
    CCU2D fpga_time_1057_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22032), .COUT(n22033), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_11.INJECT1_1 = "NO";
    LUT4 mux_931_i7_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[6]), 
         .D(ev_clear_addr[6]), .Z(ev_wr_addr[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i7_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i1477_2_lut_3_lut_4_lut (.A(ev_ch[2]), .B(n23553), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_6__N_1984[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1477_2_lut_3_lut_4_lut.init = 16'h78f0;
    CCU2D add_499_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21995), .COUT(n21996), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_499_13.INIT0 = 16'h5555;
    defparam add_499_13.INIT1 = 16'h5555;
    defparam add_499_13.INJECT1_0 = "NO";
    defparam add_499_13.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_210 (.A(ev_run_hold[18]), .B(us_tx_c_18), .C(init_shadow[18]), 
         .D(swap_now_d3), .Z(n11468)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_210.init = 16'h5a66;
    FD1P3AX spi_channel_index_1054__i1 (.D(n39_adj_3045), .SP(spi1_sck_c_enable_323), 
            .CK(spi1_sck_c), .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054__i1.GSR = "ENABLED";
    LUT4 mux_931_i8_3_lut_4_lut (.A(ev_state[0]), .B(n23522), .C(ev_wr_addr_8__N_860[7]), 
         .D(ev_clear_addr[7]), .Z(ev_wr_addr[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam mux_931_i8_3_lut_4_lut.init = 16'hf2d0;
    LUT4 run_addr_reg_8__I_0_i7_3_lut (.A(global_phase[6]), .B(ev_rd_slot[6]), 
         .C(n19130), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i7_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_4_lut_adj_211 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[48]), 
         .D(ev_bit[48]), .Z(ev_wr_data[48])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_211.init = 16'hddd0;
    FD1P3AX spi_channel_index_1054__i2 (.D(n38_adj_3044), .SP(spi1_sck_c_enable_323), 
            .CK(spi1_sck_c), .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054__i2.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1054__i3 (.D(n37), .SP(spi1_sck_c_enable_323), 
            .CK(spi1_sck_c), .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054__i3.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1054__i4 (.D(n36), .SP(spi1_sck_c_enable_323), 
            .CK(spi1_sck_c), .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054__i4.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1054__i5 (.D(n35), .SP(spi1_sck_c_enable_323), 
            .CK(spi1_sck_c), .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054__i5.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1054__i6 (.D(n34), .SP(spi1_sck_c_enable_323), 
            .CK(spi1_sck_c), .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(313[64:88])
    defparam spi_channel_index_1054__i6.GSR = "ENABLED";
    FD1P3AX status_bit_index_1053__i1 (.D(n39_adj_3040), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053__i1.GSR = "ENABLED";
    CCU2D add_135_9 (.A0(spi_byte_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21981), .COUT(n21982), .S0(spi_byte_count_15__N_1643[7]), 
          .S1(spi_byte_count_15__N_1643[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_9.INIT0 = 16'h5aaa;
    defparam add_135_9.INIT1 = 16'h5aaa;
    defparam add_135_9.INJECT1_0 = "NO";
    defparam add_135_9.INJECT1_1 = "NO";
    FD1P3AX status_bit_index_1053__i2 (.D(n38_adj_3039), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053__i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_1053__i3 (.D(n37_adj_3038), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053__i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_1053__i4 (.D(n36_adj_3030), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053__i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_1053__i5 (.D(n35_adj_3029), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053__i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_1053__i6 (.D(n34_adj_3028), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053__i6.GSR = "ENABLED";
    FD1P3AX fpga_time_1057__i1 (.D(n164), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i1.GSR = "DISABLED";
    CCU2D add_501_4 (.A0(staging_q[10]), .B0(staging_q[2]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[11]), .B1(staging_q[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n21974), .COUT(n21975), .S0(build_sum_8__N_2005[2]), 
          .S1(build_sum_8__N_2005[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_501_4.INIT0 = 16'h5666;
    defparam add_501_4.INIT1 = 16'h5666;
    defparam add_501_4.INJECT1_0 = "NO";
    defparam add_501_4.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_4_lut_adj_212 (.A(ev_ch[4]), .B(n23546), .C(n23547), 
         .D(ev_ch[3]), .Z(n14576)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_212.init = 16'h2000;
    LUT4 i1_2_lut_3_lut_4_lut_adj_213 (.A(ev_ch[4]), .B(n23546), .C(n23547), 
         .D(ev_ch[3]), .Z(n14530)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_213.init = 16'h0020;
    PFUMX ev_state_3__I_0_448_Mux_1_i15 (.BLUT(n7), .ALUT(n14_adj_3043), 
          .C0(ev_state[3]), .Z(ev_state_3__N_645[1]));
    CCU2D global_phase_7__I_0_445_6 (.A0(global_phase[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21970), .COUT(n21971), .S0(next_global_phase[4]), 
          .S1(next_global_phase[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[37:56])
    defparam global_phase_7__I_0_445_6.INIT0 = 16'h5aaa;
    defparam global_phase_7__I_0_445_6.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_445_6.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_445_6.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_4_lut_adj_214 (.A(ev_ch[4]), .B(n23546), .C(ev_ch[3]), 
         .D(n23545), .Z(n14528)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_214.init = 16'h2000;
    CCU2D add_135_7 (.A0(spi_byte_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21980), .COUT(n21981), .S0(spi_byte_count_15__N_1643[5]), 
          .S1(spi_byte_count_15__N_1643[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(325[35:57])
    defparam add_135_7.INIT0 = 16'h5aaa;
    defparam add_135_7.INIT1 = 16'h5aaa;
    defparam add_135_7.INJECT1_0 = "NO";
    defparam add_135_7.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_215 (.A(ev_run_hold[19]), .B(us_tx_c_19), .C(init_shadow[19]), 
         .D(swap_now_d3), .Z(n11470)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_215.init = 16'h5a66;
    FD1P3AX fpga_time_1057__i2 (.D(n163), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i2.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i3 (.D(n162), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i3.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i4 (.D(n161), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i4.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i5 (.D(n160), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i5.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i6 (.D(n159), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i6.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i7 (.D(n158), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i7.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i8 (.D(n157), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i8.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i9 (.D(n156), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i9.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i10 (.D(n155), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i10.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i11 (.D(n154), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i11.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i12 (.D(n153), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i12.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i13 (.D(n152), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i13.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i14 (.D(n151), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i14.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i15 (.D(n150), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i15.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i16 (.D(n149), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i16.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i17 (.D(n148), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i17.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i18 (.D(n147), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i18.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i19 (.D(n146), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i19.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i20 (.D(n145), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i20.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i21 (.D(n144), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i21.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i22 (.D(n143), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i22.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i23 (.D(n142), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i23.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i24 (.D(n141), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i24.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i25 (.D(n140), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i25.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i26 (.D(n139), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i26.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i27 (.D(n138), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i27.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i28 (.D(n137), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i28.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i29 (.D(n136), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i29.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i30 (.D(n135), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i30.GSR = "DISABLED";
    FD1P3AX fpga_time_1057__i31 (.D(n134), .SP(pll_clk_enable_555), .CK(pll_clk), 
            .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057__i31.GSR = "DISABLED";
    FD1S3IX mic_divider_1060__i1 (.D(n39), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060__i1.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_216 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[49]), 
         .D(ev_bit[49]), .Z(ev_wr_data[49])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_216.init = 16'hddd0;
    LUT4 i14649_2_lut_3_lut_4_lut_4_lut (.A(n23528), .B(ev_ch[4]), .C(n23544), 
         .D(ev_ch[3]), .Z(n14378)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i14649_2_lut_3_lut_4_lut_4_lut.init = 16'h0001;
    LUT4 i2_3_lut_4_lut_adj_217 (.A(frame_settle[0]), .B(n23537), .C(pll_clk_enable_17), 
         .D(pll_clk_enable_18), .Z(pll_clk_enable_566)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[22:42])
    defparam i2_3_lut_4_lut_adj_217.init = 16'hfffe;
    LUT4 i1456_2_lut (.A(ev_ch[1]), .B(n23836), .Z(ev_ch_6__N_1984[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1456_2_lut.init = 16'h6666;
    LUT4 sub_67_inv_0_i6_1_lut (.A(status_bit_index[5]), .Z(spi1_miso_N_2465[5])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(264[55:80])
    defparam sub_67_inv_0_i6_1_lut.init = 16'h5555;
    LUT4 build_phase_7__I_0_i1_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[0]), 
         .D(build_sum[0]), .Z(ev_rd_slot[0])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i1_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i3_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[2]), 
         .D(build_sum[2]), .Z(ev_rd_slot[2])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i3_3_lut_4_lut.init = 16'hf4b0;
    FD1S3IX ev_bit_i0 (.D(n14390), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i0.GSR = "DISABLED";
    FD1P3IX init_shadow_i1 (.D(n12772), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i1.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_218 (.A(ev_run_hold[20]), .B(us_tx_c_20), .C(init_shadow[20]), 
         .D(swap_now_d3), .Z(n11472)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_218.init = 16'h5a66;
    LUT4 i13526_2_lut (.A(spi_channel_field[1]), .B(spi_channel_field[0]), 
         .Z(n14)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(317[78:102])
    defparam i13526_2_lut.init = 16'h6666;
    FD1P3IX ev_ch_i6 (.D(ev_ch_6__N_1984[6]), .SP(pll_clk_enable_562), .CD(n17517), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i6.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i4_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[3]), 
         .D(build_sum[3]), .Z(ev_rd_slot[3])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i4_3_lut_4_lut.init = 16'hf4b0;
    FD1P3IX ev_ch_i5 (.D(ev_ch_6__N_1984[5]), .SP(pll_clk_enable_562), .CD(n17517), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i5.GSR = "DISABLED";
    LUT4 i1633_1_lut (.A(status_bit_index[3]), .Z(spi1_miso_N_2465[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i1633_1_lut.init = 16'h5555;
    FD1P3IX ev_ch_i4 (.D(ev_ch_6__N_1984[4]), .SP(pll_clk_enable_562), .CD(n17517), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i4.GSR = "DISABLED";
    FD1P3IX ev_ch_i3 (.D(ev_ch_6__N_1984[3]), .SP(pll_clk_enable_562), .CD(n17517), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i3.GSR = "DISABLED";
    FD1P3IX ev_ch_i2 (.D(ev_ch_6__N_1984[2]), .SP(pll_clk_enable_562), .CD(n17517), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i2.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i5_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[4]), 
         .D(build_sum[4]), .Z(ev_rd_slot[4])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i5_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i7_4_lut_adj_219 (.A(ev_run_hold[21]), .B(us_tx_c_21), .C(init_shadow[21]), 
         .D(swap_now_d3), .Z(n11474)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_219.init = 16'h5a66;
    FD1P3IX ev_ch_i1 (.D(ev_ch_6__N_1984[1]), .SP(pll_clk_enable_562), .CD(n17517), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i1.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i2_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[1]), 
         .D(build_sum[1]), .Z(ev_rd_slot[1])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i2_3_lut_4_lut.init = 16'hf4b0;
    PFUMX i14555 (.BLUT(n23065), .ALUT(n23066), .C0(n23556), .Z(n23077));
    LUT4 i7_4_lut_adj_220 (.A(ev_run_hold[22]), .B(us_tx_c_22), .C(init_shadow[22]), 
         .D(swap_now_d3), .Z(n11476)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_220.init = 16'h5a66;
    LUT4 i7_4_lut_adj_221 (.A(ev_run_hold[23]), .B(us_tx_c_23), .C(init_shadow[23]), 
         .D(swap_now_d3), .Z(n11478)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_221.init = 16'h5a66;
    LUT4 i4_4_lut (.A(n7_adj_3066), .B(n22711), .C(spi_channel_field[1]), 
         .D(spi_channel_field[0]), .Z(spi_write)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i4_4_lut.init = 16'h0800;
    LUT4 build_phase_7__I_0_i6_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[5]), 
         .D(build_sum[5]), .Z(ev_rd_slot[5])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i6_3_lut_4_lut.init = 16'hf4b0;
    PFUMX i110235_i1 (.BLUT(n23053), .ALUT(n23087), .C0(spi1_miso_N_2465[5]), 
          .Z(n63));
    LUT4 i1_3_lut_4_lut_adj_222 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[50]), 
         .D(ev_bit[50]), .Z(ev_wr_data[50])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_222.init = 16'hddd0;
    LUT4 i7_4_lut_adj_223 (.A(ev_run_hold[24]), .B(us_tx_c_24), .C(init_shadow[24]), 
         .D(swap_now_d3), .Z(n11480)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_223.init = 16'h5a66;
    LUT4 build_phase_7__I_0_i7_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[6]), 
         .D(build_sum[6]), .Z(ev_rd_slot[6])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i7_3_lut_4_lut.init = 16'hf4b0;
    LUT4 build_phase_7__I_0_i8_3_lut_4_lut (.A(n23530), .B(n23561), .C(build_phase[7]), 
         .D(build_sum[7]), .Z(ev_rd_slot[7])) /* synthesis lut_function=(A (C)+!A (B (D)+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam build_phase_7__I_0_i8_3_lut_4_lut.init = 16'hf4b0;
    LUT4 i14634_3_lut_4_lut (.A(ev_state[3]), .B(pll_clk_enable_17), .C(n14_adj_3053), 
         .D(ev_state[0]), .Z(pll_clk_enable_449)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;
    defparam i14634_3_lut_4_lut.init = 16'hefff;
    LUT4 i14664_2_lut_3_lut_4_lut_4_lut (.A(n23551), .B(ev_ch[4]), .C(ev_ch[3]), 
         .D(n23544), .Z(n14588)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i14664_2_lut_3_lut_4_lut_4_lut.init = 16'h0010;
    PFUMX i14534 (.BLUT(n23054), .ALUT(n23055), .C0(spi1_miso_N_2465[5]), 
          .Z(n23056));
    LUT4 i1_3_lut_4_lut_adj_224 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[51]), 
         .D(ev_bit[51]), .Z(ev_wr_data[51])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_224.init = 16'hddd0;
    FD1P3IX spi_channel_field_1055__i1 (.D(n14), .SP(spi1_sck_c_enable_325), 
            .CD(spi1_sck_c_enable_323), .CK(spi1_sck_c), .Q(spi_channel_field[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(317[78:102])
    defparam spi_channel_field_1055__i1.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut_adj_225 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[52]), 
         .D(ev_bit[52]), .Z(ev_wr_data[52])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_225.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_226 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[53]), 
         .D(ev_bit[53]), .Z(ev_wr_data[53])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_226.init = 16'hddd0;
    LUT4 i7_4_lut_adj_227 (.A(ev_run_hold[25]), .B(us_tx_c_25), .C(init_shadow[25]), 
         .D(swap_now_d3), .Z(n11482)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_227.init = 16'h5a66;
    LUT4 i7_4_lut_adj_228 (.A(ev_run_hold[26]), .B(us_tx_c_26), .C(init_shadow[26]), 
         .D(swap_now_d3), .Z(n11484)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_228.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_229 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[54]), 
         .D(ev_bit[54]), .Z(ev_wr_data[54])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_229.init = 16'hddd0;
    LUT4 i1_2_lut_rep_118 (.A(ev_ch[5]), .B(ev_ch[6]), .Z(n23528)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i1_2_lut_rep_118.init = 16'hbbbb;
    LUT4 i1_3_lut_4_lut_adj_230 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[55]), 
         .D(ev_bit[55]), .Z(ev_wr_data[55])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_230.init = 16'hddd0;
    LUT4 i7_4_lut_adj_231 (.A(ev_run_hold[27]), .B(us_tx_c_27), .C(init_shadow[27]), 
         .D(swap_now_d3), .Z(n11486)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_231.init = 16'h5a66;
    LUT4 i2_4_lut_adj_232 (.A(spi_byte_count[9]), .B(n22660), .C(n8), 
         .D(spi_byte_count[10]), .Z(n7_adj_3066)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i2_4_lut_adj_232.init = 16'h0040;
    LUT4 i7_4_lut_adj_233 (.A(ev_run_hold[28]), .B(us_tx_c_28), .C(init_shadow[28]), 
         .D(swap_now_d3), .Z(n11488)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_233.init = 16'h5a66;
    LUT4 i3_4_lut_adj_234 (.A(n38_adj_3057), .B(n23541), .C(n22974), .D(spi_byte_count[7]), 
         .Z(n8)) /* synthesis lut_function=(!(A (B+(C (D)))+!A (B+(C+!(D))))) */ ;
    defparam i3_4_lut_adj_234.init = 16'h0322;
    L6MUX21 i14529 (.D0(n23047), .D1(n23048), .SD(spi1_miso_N_2465[3]), 
            .Z(n23051));
    LUT4 i1491_3_lut_4_lut (.A(ev_ch[4]), .B(n23504), .C(ev_ch[5]), .D(ev_ch[6]), 
         .Z(ev_ch_6__N_1984[6])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1491_3_lut_4_lut.init = 16'h7f80;
    LUT4 i1_2_lut_rep_119 (.A(ev_state[0]), .B(ev_state[1]), .Z(n23529)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_2_lut_rep_119.init = 16'h2222;
    LUT4 i4754_4_lut (.A(ev_ch[1]), .B(staging_rd_addr[1]), .C(n15663), 
         .D(n22670), .Z(staging_rd_addr_6__N_849[1])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i4754_4_lut.init = 16'hcac0;
    LUT4 i4756_4_lut (.A(ev_ch[2]), .B(staging_rd_addr[2]), .C(n15663), 
         .D(n22670), .Z(staging_rd_addr_6__N_849[2])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i4756_4_lut.init = 16'hcac0;
    L6MUX21 i14530 (.D0(n23049), .D1(n23050), .SD(spi1_miso_N_2465[3]), 
            .Z(n23052));
    LUT4 i4758_4_lut (.A(ev_ch[3]), .B(staging_rd_addr[3]), .C(n15663), 
         .D(n22670), .Z(staging_rd_addr_6__N_849[3])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i4758_4_lut.init = 16'hcac0;
    LUT4 i7_4_lut_adj_235 (.A(ev_run_hold[29]), .B(us_tx_c_29), .C(init_shadow[29]), 
         .D(swap_now_d3), .Z(n11490)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_235.init = 16'h5a66;
    L6MUX21 i14563 (.D0(n23081), .D1(n23082), .SD(spi1_miso_N_2465[3]), 
            .Z(n23085));
    L6MUX21 i14564 (.D0(n23083), .D1(n23084), .SD(spi1_miso_N_2465[3]), 
            .Z(n23086));
    LUT4 i4760_4_lut (.A(ev_ch[4]), .B(staging_rd_addr[4]), .C(n15663), 
         .D(n22670), .Z(staging_rd_addr_6__N_849[4])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i4760_4_lut.init = 16'hcac0;
    LUT4 i1_3_lut_4_lut_adj_236 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[56]), 
         .D(ev_bit[56]), .Z(ev_wr_data[56])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_236.init = 16'hddd0;
    LUT4 ev_state_3__I_0_440_i6_2_lut_rep_120 (.A(ev_state[2]), .B(ev_state[3]), 
         .Z(n23530)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam ev_state_3__I_0_440_i6_2_lut_rep_120.init = 16'hdddd;
    LUT4 i1_2_lut_rep_104_3_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(n23514)) /* synthesis lut_function=((B+(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam i1_2_lut_rep_104_3_lut.init = 16'hfdfd;
    LUT4 i7_4_lut_adj_237 (.A(ev_run_hold[30]), .B(us_tx_c_30), .C(init_shadow[30]), 
         .D(swap_now_d3), .Z(n11492)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_237.init = 16'h5a66;
    L6MUX21 i14525 (.D0(n23039), .D1(n23040), .SD(n23550), .Z(n23047));
    LUT4 i7_4_lut_adj_238 (.A(ev_run_hold[31]), .B(us_tx_c_31), .C(init_shadow[31]), 
         .D(swap_now_d3), .Z(n11494)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_238.init = 16'h5a66;
    L6MUX21 i14526 (.D0(n23041), .D1(n23042), .SD(n23550), .Z(n23048));
    LUT4 i4762_4_lut (.A(ev_ch[5]), .B(staging_rd_addr[5]), .C(n15663), 
         .D(n22670), .Z(staging_rd_addr_6__N_849[5])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i4762_4_lut.init = 16'hcac0;
    LUT4 i1_3_lut_4_lut_adj_239 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[57]), 
         .D(ev_bit[57]), .Z(ev_wr_data[57])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_239.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_then_4_lut (.A(ev_state[3]), .B(ev_state[2]), .C(ev_state[1]), 
         .D(ev_state_3__N_1976[1]), .Z(n23565)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i1_3_lut_4_lut_then_4_lut.init = 16'hfeff;
    LUT4 i1_3_lut_4_lut_adj_240 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[58]), 
         .D(ev_bit[58]), .Z(ev_wr_data[58])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_240.init = 16'hddd0;
    L6MUX21 i14527 (.D0(n23043), .D1(n23044), .SD(n23550), .Z(n23049));
    L6MUX21 i14528 (.D0(n23045), .D1(n23046), .SD(n23550), .Z(n23050));
    L6MUX21 i14559 (.D0(n23073), .D1(n23074), .SD(n23550), .Z(n23081));
    L6MUX21 i14560 (.D0(n23075), .D1(n23076), .SD(n23550), .Z(n23082));
    L6MUX21 i14561 (.D0(n23077), .D1(n23078), .SD(n23550), .Z(n23083));
    L6MUX21 i14562 (.D0(n23079), .D1(n23080), .SD(n23550), .Z(n23084));
    PFUMX i14517 (.BLUT(n23023), .ALUT(n23024), .C0(n23556), .Z(n23039));
    LUT4 i1_2_lut_3_lut_4_lut_adj_241 (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n22721)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(206[33:53])
    defparam i1_2_lut_3_lut_4_lut_adj_241.init = 16'hfffd;
    CCU2D fpga_time_1057_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22031), .COUT(n22032), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_9.INJECT1_1 = "NO";
    LUT4 i10576_2_lut_rep_121 (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .Z(n23531)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10576_2_lut_rep_121.init = 16'h8888;
    LUT4 i2_2_lut_3_lut_4_lut (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .C(n23543), .D(spi_byte_count[2]), .Z(n7_adj_3054)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_2_lut_3_lut_4_lut.init = 16'hfff7;
    LUT4 i1_4_lut_adj_242 (.A(n22709), .B(n16497), .C(ev_state[1]), .D(n23493), 
         .Z(ev_state_3__N_645[3])) /* synthesis lut_function=(A (B+!((D)+!C))+!A (B)) */ ;
    defparam i1_4_lut_adj_242.init = 16'hccec;
    LUT4 i68_3_lut_4_lut (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[3]), .Z(n55)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C+(D)))+!A !(C+(D)))) */ ;
    defparam i68_3_lut_4_lut.init = 16'h7ff0;
    LUT4 i7_4_lut_adj_243 (.A(ev_run_hold[32]), .B(us_tx_c_32), .C(init_shadow[32]), 
         .D(swap_now_d3), .Z(n11496)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_243.init = 16'h5a66;
    PFUMX i14518 (.BLUT(n23025), .ALUT(n23026), .C0(n23556), .Z(n23040));
    LUT4 i10463_4_lut (.A(n13851), .B(fpga_cs_n_c), .C(n63), .D(status_bit_index[6]), 
         .Z(spi1_miso_c)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(264[24:81])
    defparam i10463_4_lut.init = 16'h3022;
    LUT4 i10819_2_lut (.A(n23056), .B(status_bit_index[3]), .Z(n13851)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(264[55:80])
    defparam i10819_2_lut.init = 16'h2222;
    PFUMX i14519 (.BLUT(n23027), .ALUT(n23028), .C0(n23556), .Z(n23041));
    LUT4 i7_4_lut_adj_244 (.A(ev_run_hold[33]), .B(us_tx_c_33), .C(init_shadow[33]), 
         .D(swap_now_d3), .Z(n11498)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_244.init = 16'h5a66;
    PFUMX i14520 (.BLUT(n23029), .ALUT(n23030), .C0(n23556), .Z(n23042));
    LUT4 cs_sync_d_I_0_2_lut (.A(cs_sync_d), .B(cs_sync), .Z(cs_fall)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(244[20:41])
    defparam cs_sync_d_I_0_2_lut.init = 16'h2222;
    LUT4 i1_3_lut_4_lut_adj_245 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[59]), 
         .D(ev_bit[59]), .Z(ev_wr_data[59])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_245.init = 16'hddd0;
    LUT4 i14453_2_lut_3_lut_4_lut (.A(spi_byte_count[3]), .B(spi_byte_count[2]), 
         .C(spi_byte_count[6]), .D(n23533), .Z(n22974)) /* synthesis lut_function=(A (B (C)+!B (C (D)))+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i14453_2_lut_3_lut_4_lut.init = 16'hf080;
    LUT4 i7_4_lut_adj_246 (.A(ev_run_hold[34]), .B(us_tx_c_34), .C(init_shadow[34]), 
         .D(swap_now_d3), .Z(n11500)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_246.init = 16'h5a66;
    LUT4 i7_4_lut_adj_247 (.A(ev_run_hold[35]), .B(us_tx_c_35), .C(init_shadow[35]), 
         .D(swap_now_d3), .Z(n11502)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_247.init = 16'h5a66;
    LUT4 i2_4_lut_4_lut (.A(spi_byte_count[3]), .B(spi_byte_count[2]), .C(spi_byte_count[4]), 
         .D(fpga_cs_n_c), .Z(n8_adj_3055)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A ((D)+!C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i2_4_lut_4_lut.init = 16'h0058;
    LUT4 i1_3_lut_4_lut_adj_248 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[60]), 
         .D(ev_bit[60]), .Z(ev_wr_data[60])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_248.init = 16'hddd0;
    LUT4 n19130_bdd_4_lut (.A(n19130), .B(ev_state[2]), .C(ev_state[1]), 
         .D(ev_state[0]), .Z(n23324)) /* synthesis lut_function=(!(A ((C (D)+!C !(D))+!B)+!A ((C (D))+!B))) */ ;
    defparam n19130_bdd_4_lut.init = 16'h0cc4;
    LUT4 i1_3_lut_4_lut_adj_249 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[61]), 
         .D(ev_bit[61]), .Z(ev_wr_data[61])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_249.init = 16'hddd0;
    LUT4 i7_4_lut_adj_250 (.A(ev_run_hold[36]), .B(us_tx_c_36), .C(init_shadow[36]), 
         .D(swap_now_d3), .Z(n11504)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_250.init = 16'h5a66;
    LUT4 active_bank_I_0_493_1_lut_rep_122 (.A(active_bank), .Z(n23532)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[56:68])
    defparam active_bank_I_0_493_1_lut_rep_122.init = 16'h5555;
    LUT4 i1_2_lut_adj_251 (.A(ev_ch[6]), .B(ev_ch[5]), .Z(n14801)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_251.init = 16'h2222;
    CCU2D fpga_time_1057_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22030), .COUT(n22031), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_7.INJECT1_1 = "NO";
    LUT4 run_addr_reg_8__I_0_i9_3_lut_3_lut (.A(active_bank), .B(n19130), 
         .C(run_addr_reg[8]), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (B (C))+!A ((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[56:68])
    defparam run_addr_reg_8__I_0_i9_3_lut_3_lut.init = 16'hd1d1;
    LUT4 i7_4_lut_adj_252 (.A(ev_run_hold[37]), .B(us_tx_c_37), .C(init_shadow[37]), 
         .D(swap_now_d3), .Z(n11506)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_252.init = 16'h5a66;
    LUT4 i10469_2_lut_rep_123 (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .Z(n23533)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10469_2_lut_rep_123.init = 16'heeee;
    PFUMX i14521 (.BLUT(n23031), .ALUT(n23032), .C0(n23556), .Z(n23043));
    LUT4 i7_4_lut_adj_253 (.A(ev_run_hold[38]), .B(us_tx_c_38), .C(init_shadow[38]), 
         .D(swap_now_d3), .Z(n11508)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_253.init = 16'h5a66;
    LUT4 i7_4_lut_adj_254 (.A(ev_run_hold[39]), .B(us_tx_c_39), .C(init_shadow[39]), 
         .D(swap_now_d3), .Z(n11510)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_254.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_255 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[62]), 
         .D(ev_bit[62]), .Z(ev_wr_data[62])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_255.init = 16'hddd0;
    CCU2D fpga_time_1057_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22029), .COUT(n22030), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_5.INJECT1_1 = "NO";
    FD1P3AX ev_state_i3 (.D(ev_state_3__N_645[3]), .SP(pll_clk_enable_565), 
            .CK(pll_clk), .Q(ev_state[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_state_i3.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_256 (.A(ev_run_hold[40]), .B(us_tx_c_40), .C(init_shadow[40]), 
         .D(swap_now_d3), .Z(n11512)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_256.init = 16'h5a66;
    LUT4 i10823_2_lut_rep_106_3_lut_4_lut (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[3]), .Z(n23516)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i10823_2_lut_rep_106_3_lut_4_lut.init = 16'hfeee;
    PFUMX i14522 (.BLUT(n23033), .ALUT(n23034), .C0(n23556), .Z(n23044));
    LUT4 i7_4_lut_adj_257 (.A(ev_run_hold[41]), .B(us_tx_c_41), .C(init_shadow[41]), 
         .D(swap_now_d3), .Z(n11514)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_257.init = 16'h5a66;
    LUT4 i7_4_lut_adj_258 (.A(ev_run_hold[42]), .B(us_tx_c_42), .C(init_shadow[42]), 
         .D(swap_now_d3), .Z(n11516)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_258.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_259 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[63]), 
         .D(ev_bit[63]), .Z(ev_wr_data[63])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_259.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_260 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[64]), 
         .D(ev_bit[64]), .Z(ev_wr_data[64])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_260.init = 16'hddd0;
    LUT4 i7_4_lut_adj_261 (.A(ev_run_hold[43]), .B(us_tx_c_43), .C(init_shadow[43]), 
         .D(swap_now_d3), .Z(n11518)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_261.init = 16'h5a66;
    PFUMX i14523 (.BLUT(n23035), .ALUT(n23036), .C0(n23556), .Z(n23045));
    LUT4 i1_2_lut_adj_262 (.A(spi_byte_count[0]), .B(n22681), .Z(spi1_sck_c_enable_36)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_262.init = 16'h4444;
    LUT4 i3_4_lut_adj_263 (.A(n5), .B(spi_byte_count[5]), .C(n23503), 
         .D(n19122), .Z(n22681)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i3_4_lut_adj_263.init = 16'h0002;
    LUT4 i1_4_lut_adj_264 (.A(spi1_sck_c_enable_286), .B(n23543), .C(n18996), 
         .D(spi_byte_count[8]), .Z(n5)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i1_4_lut_adj_264.init = 16'h0002;
    PFUMX i14524 (.BLUT(n23037), .ALUT(n23038), .C0(n23556), .Z(n23046));
    PFUMX i14556 (.BLUT(n23067), .ALUT(n23068), .C0(n23556), .Z(n23078));
    LUT4 i1_3_lut_4_lut_adj_265 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[65]), 
         .D(ev_bit[65]), .Z(ev_wr_data[65])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_265.init = 16'hddd0;
    PFUMX i14551 (.BLUT(n23057), .ALUT(n23058), .C0(n23556), .Z(n23073));
    LUT4 i10479_2_lut (.A(spi_byte_count[2]), .B(spi_byte_count[1]), .Z(n18996)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10479_2_lut.init = 16'heeee;
    LUT4 i7_4_lut_adj_266 (.A(ev_run_hold[44]), .B(us_tx_c_44), .C(init_shadow[44]), 
         .D(swap_now_d3), .Z(n11520)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_266.init = 16'h5a66;
    PFUMX i14552 (.BLUT(n23059), .ALUT(n23060), .C0(n23556), .Z(n23074));
    LUT4 i1_3_lut_4_lut_adj_267 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[66]), 
         .D(ev_bit[66]), .Z(ev_wr_data[66])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_267.init = 16'hddd0;
    LUT4 i2_3_lut_4_lut_adj_268 (.A(status_flags_wire_15__N_1349[4]), .B(pll_locked), 
         .C(pll_clk_enable_17), .D(phase_step_d3), .Z(pll_clk_enable_162)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_adj_268.init = 16'hfff7;
    LUT4 i1_2_lut_adj_269 (.A(spi_byte_count[6]), .B(spi_byte_count[7]), 
         .Z(n19122)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_2_lut_adj_269.init = 16'heeee;
    LUT4 n19130_bdd_4_lut_14740 (.A(ev_state_3__N_1964[2]), .B(ev_state[2]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n23323)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam n19130_bdd_4_lut_14740.init = 16'h0020;
    LUT4 i1_2_lut_rep_124 (.A(spi_byte_count[8]), .B(spi_byte_count[9]), 
         .Z(n23534)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_124.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_270 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[67]), 
         .D(ev_bit[67]), .Z(ev_wr_data[67])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_270.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_271 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[68]), 
         .D(ev_bit[68]), .Z(ev_wr_data[68])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_271.init = 16'hddd0;
    LUT4 i7_4_lut_adj_272 (.A(ev_run_hold[45]), .B(us_tx_c_45), .C(init_shadow[45]), 
         .D(swap_now_d3), .Z(n11522)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_272.init = 16'h5a66;
    LUT4 i7_4_lut_adj_273 (.A(ev_run_hold[46]), .B(us_tx_c_46), .C(init_shadow[46]), 
         .D(swap_now_d3), .Z(n11524)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_273.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_274 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[69]), 
         .D(ev_bit[69]), .Z(ev_wr_data[69])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_274.init = 16'hddd0;
    LUT4 i2_3_lut_rep_84_4_lut (.A(spi_byte_count[8]), .B(spi_byte_count[9]), 
         .C(n19122), .D(n19458), .Z(n23494)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_rep_84_4_lut.init = 16'hfffe;
    LUT4 i7_4_lut_adj_275 (.A(ev_run_hold[47]), .B(us_tx_c_47), .C(init_shadow[47]), 
         .D(swap_now_d3), .Z(n11526)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_275.init = 16'h5a66;
    LUT4 i1_2_lut_adj_276 (.A(spi_byte_count[0]), .B(n22681), .Z(spi1_sck_c_enable_43)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_276.init = 16'h8888;
    LUT4 i1_2_lut_rep_125 (.A(spi_byte_count[1]), .B(spi_byte_count[3]), 
         .Z(n23535)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_rep_125.init = 16'h2222;
    LUT4 i7_4_lut_adj_277 (.A(ev_run_hold[48]), .B(us_tx_c_48), .C(init_shadow[48]), 
         .D(swap_now_d3), .Z(n11528)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_277.init = 16'h5a66;
    LUT4 i2_2_lut_rep_107_3_lut (.A(spi_byte_count[1]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[2]), .Z(n23517)) /* synthesis lut_function=(!((B+(C))+!A)) */ ;
    defparam i2_2_lut_rep_107_3_lut.init = 16'h0202;
    LUT4 i81_3_lut_3_lut (.A(spi_byte_count[1]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[2]), .Z(n60)) /* synthesis lut_function=(!(A (B+!(C))+!A (C))) */ ;
    defparam i81_3_lut_3_lut.init = 16'h2525;
    LUT4 stop_toggle_sync_I_0_2_lut_rep_126 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(pll_clk_enable_17)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[23:61])
    defparam stop_toggle_sync_I_0_2_lut_rep_126.init = 16'h6666;
    LUT4 i576_2_lut_2_lut_3_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(swap_now_d1), .Z(pll_clk_enable_445)) /* synthesis lut_function=(A (B (C))+!A !(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[23:61])
    defparam i576_2_lut_2_lut_3_lut.init = 16'h9090;
    LUT4 i7_4_lut_adj_278 (.A(ev_run_hold[49]), .B(us_tx_c_49), .C(init_shadow[49]), 
         .D(swap_now_d3), .Z(n11530)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_278.init = 16'h5a66;
    LUT4 i7_4_lut_adj_279 (.A(ev_run_hold[50]), .B(us_tx_c_50), .C(init_shadow[50]), 
         .D(swap_now_d3), .Z(n11532)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_279.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_280 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[71]), 
         .D(ev_bit[71]), .Z(ev_wr_data[71])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_280.init = 16'hddd0;
    LUT4 i2_3_lut_4_lut_adj_281 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(swap_now_d1), .D(swap_pending_N_2519), .Z(pll_clk_enable_7)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[23:61])
    defparam i2_3_lut_4_lut_adj_281.init = 16'hfff6;
    LUT4 i7_4_lut_adj_282 (.A(ev_run_hold[51]), .B(us_tx_c_51), .C(init_shadow[51]), 
         .D(swap_now_d3), .Z(n11534)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_282.init = 16'h5a66;
    LUT4 i7_4_lut_adj_283 (.A(ev_run_hold[52]), .B(us_tx_c_52), .C(init_shadow[52]), 
         .D(swap_now_d3), .Z(n11536)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_283.init = 16'h5a66;
    LUT4 i7_4_lut_adj_284 (.A(ev_run_hold[53]), .B(us_tx_c_53), .C(init_shadow[53]), 
         .D(swap_now_d3), .Z(n11538)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_284.init = 16'h5a66;
    LUT4 i1_2_lut_3_lut_4_lut_adj_285 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1349[4]), .Z(n9326)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[23:61])
    defparam i1_2_lut_3_lut_4_lut_adj_285.init = 16'h6fff;
    LUT4 i7_4_lut_adj_286 (.A(ev_run_hold[54]), .B(us_tx_c_54), .C(init_shadow[54]), 
         .D(swap_now_d3), .Z(n11540)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_286.init = 16'h5a66;
    LUT4 i7_4_lut_adj_287 (.A(ev_run_hold[55]), .B(us_tx_c_55), .C(init_shadow[55]), 
         .D(swap_now_d3), .Z(n11542)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_287.init = 16'h5a66;
    LUT4 i14698_2_lut_3_lut_4_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(n23314), .D(ev_state[3]), .Z(pll_clk_enable_565)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[23:61])
    defparam i14698_2_lut_3_lut_4_lut.init = 16'hfff6;
    LUT4 i7_4_lut_adj_288 (.A(ev_run_hold[56]), .B(us_tx_c_56), .C(init_shadow[56]), 
         .D(swap_now_d3), .Z(n11544)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_288.init = 16'h5a66;
    LUT4 i7_4_lut_adj_289 (.A(ev_run_hold[57]), .B(us_tx_c_57), .C(init_shadow[57]), 
         .D(swap_now_d3), .Z(n11546)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_289.init = 16'h5a66;
    PFUMX i14557 (.BLUT(n23069), .ALUT(n23070), .C0(n23556), .Z(n23079));
    LUT4 i18_1_lut_rep_114_2_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(pll_clk_enable_19)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[23:61])
    defparam i18_1_lut_rep_114_2_lut.init = 16'h9999;
    LUT4 i2_3_lut_rep_127 (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .Z(n23537)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[22:42])
    defparam i2_3_lut_rep_127.init = 16'hfefe;
    LUT4 i1_2_lut_rep_115_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(pll_clk_enable_79)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[22:42])
    defparam i1_2_lut_rep_115_4_lut.init = 16'hfffe;
    LUT4 i6416_2_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n15027)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[22:42])
    defparam i6416_2_lut_4_lut.init = 16'h00fe;
    LUT4 i7_4_lut_adj_290 (.A(ev_run_hold[58]), .B(us_tx_c_58), .C(init_shadow[58]), 
         .D(swap_now_d3), .Z(n11548)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_290.init = 16'h5a66;
    LUT4 i7_4_lut_adj_291 (.A(ev_run_hold[59]), .B(us_tx_c_59), .C(init_shadow[59]), 
         .D(swap_now_d3), .Z(n11550)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_291.init = 16'h5a66;
    LUT4 i1_2_lut_adj_292 (.A(spi_byte_count[5]), .B(n14781), .Z(n4_adj_3064)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_292.init = 16'heeee;
    LUT4 frame_toggle_sync_I_0_2_lut_rep_128 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(pll_clk_enable_18)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[13:51])
    defparam frame_toggle_sync_I_0_2_lut_rep_128.init = 16'h6666;
    LUT4 i2619_2_lut_3_lut_4_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(stop_toggle_seen), .D(stop_toggle_sync), .Z(n11144)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[13:51])
    defparam i2619_2_lut_3_lut_4_lut.init = 16'h6ff6;
    LUT4 i1_3_lut_4_lut_adj_293 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n23537), .D(frame_settle[0]), .Z(pll_clk_enable_77)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(383[13:51])
    defparam i1_3_lut_4_lut_adj_293.init = 16'h0900;
    LUT4 i1_2_lut_rep_129 (.A(swap_pending), .B(frame_req), .Z(n23539)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(192[55:68])
    defparam i1_2_lut_rep_129.init = 16'h4444;
    LUT4 i14707_3_lut (.A(n22614), .B(spi_byte_count[5]), .C(spi_byte_count[4]), 
         .Z(spi1_sck_c_enable_44)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i14707_3_lut.init = 16'h1010;
    LUT4 i10502_4_lut (.A(frame_settle[3]), .B(pll_clk_enable_18), .C(frame_settle[2]), 
         .D(n23554), .Z(frame_settle_3__N_1924[3])) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[18] 392[12])
    defparam i10502_4_lut.init = 16'heeed;
    LUT4 i5_4_lut_adj_294 (.A(spi_byte_count[7]), .B(n7_adj_3056), .C(n22667), 
         .D(n8_adj_3055), .Z(spi1_sck_c_enable_271)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut_adj_294.init = 16'h8000;
    LUT4 i13524_1_lut (.A(spi_channel_field[0]), .Z(n15)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(317[78:102])
    defparam i13524_1_lut.init = 16'h5555;
    CCU2D fpga_time_1057_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22028), .COUT(n22029), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_1057_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_1057_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_3.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_295 (.A(ev_run_hold[60]), .B(us_tx_c_60), .C(init_shadow[60]), 
         .D(swap_now_d3), .Z(n11552)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_295.init = 16'h5a66;
    LUT4 i7_4_lut_adj_296 (.A(ev_run_hold[61]), .B(us_tx_c_61), .C(init_shadow[61]), 
         .D(swap_now_d3), .Z(n11554)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_296.init = 16'h5a66;
    LUT4 i1_2_lut_adj_297 (.A(expected_next_15__N_1406[3]), .B(spi_byte_count[6]), 
         .Z(n7_adj_3056)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_297.init = 16'h8888;
    CCU2D fpga_time_1057_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22028), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(367[29:46])
    defparam fpga_time_1057_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_1057_add_4_1.INIT1 = 16'h0555;
    defparam fpga_time_1057_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_1057_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_298 (.A(ev_run_hold[62]), .B(us_tx_c_62), .C(init_shadow[62]), 
         .D(swap_now_d3), .Z(n11556)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_298.init = 16'h5a66;
    LUT4 i7_4_lut_adj_299 (.A(ev_run_hold[63]), .B(us_tx_c_63), .C(init_shadow[63]), 
         .D(swap_now_d3), .Z(n11558)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_299.init = 16'h5a66;
    LUT4 i7_4_lut_adj_300 (.A(ev_run_hold[64]), .B(us_tx_c_64), .C(init_shadow[64]), 
         .D(swap_now_d3), .Z(n11560)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_300.init = 16'h5a66;
    LUT4 i13468_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam i13468_2_lut.init = 16'h6666;
    LUT4 i13490_2_lut (.A(mic_sample_count[1]), .B(mic_sample_count[0]), 
         .Z(n29)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam i13490_2_lut.init = 16'h6666;
    LUT4 i7_4_lut_adj_301 (.A(ev_run_hold[65]), .B(us_tx_c_65), .C(init_shadow[65]), 
         .D(swap_now_d3), .Z(n11562)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_301.init = 16'h5a66;
    LUT4 i1_2_lut_rep_108_3_lut (.A(swap_pending), .B(frame_req), .C(ev_state[0]), 
         .Z(n23518)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(192[55:68])
    defparam i1_2_lut_rep_108_3_lut.init = 16'h0404;
    LUT4 i46_3_lut_rep_103_4_lut (.A(swap_pending), .B(frame_req), .C(ev_state[0]), 
         .D(ev_state_3__N_1976[1]), .Z(n23513)) /* synthesis lut_function=(A (C (D))+!A (B ((D)+!C)+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(192[55:68])
    defparam i46_3_lut_rep_103_4_lut.init = 16'hf404;
    LUT4 i1_2_lut_rep_130 (.A(spi_byte_count[14]), .B(spi_byte_count[13]), 
         .Z(n23540)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_2_lut_rep_130.init = 16'heeee;
    LUT4 i1_2_lut_rep_110_3_lut (.A(spi_byte_count[14]), .B(spi_byte_count[13]), 
         .C(spi_byte_count[15]), .Z(n23520)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i1_2_lut_rep_110_3_lut.init = 16'hfefe;
    LUT4 i7_4_lut_adj_302 (.A(ev_run_hold[66]), .B(us_tx_c_66), .C(init_shadow[66]), 
         .D(swap_now_d3), .Z(n11564)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_302.init = 16'h5a66;
    LUT4 i14243_2_lut_rep_131 (.A(spi_byte_count[11]), .B(spi_byte_count[12]), 
         .Z(n23541)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14243_2_lut_rep_131.init = 16'heeee;
    LUT4 i7_4_lut_adj_303 (.A(ev_run_hold[67]), .B(us_tx_c_67), .C(init_shadow[67]), 
         .D(swap_now_d3), .Z(n11566)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_303.init = 16'h5a66;
    LUT4 i2_2_lut_rep_109_3_lut (.A(spi_byte_count[11]), .B(spi_byte_count[12]), 
         .C(spi_byte_count[10]), .Z(n23519)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_2_lut_rep_109_3_lut.init = 16'hfefe;
    LUT4 i13471_2_lut_rep_132 (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .Z(n23542)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam i13471_2_lut_rep_132.init = 16'h8888;
    LUT4 i13475_2_lut_3_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .Z(n18)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam i13475_2_lut_3_lut.init = 16'h7878;
    LUT4 i7_4_lut_adj_304 (.A(ev_run_hold[68]), .B(us_tx_c_68), .C(init_shadow[68]), 
         .D(swap_now_d3), .Z(n11568)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_304.init = 16'h5a66;
    LUT4 i2_2_lut_rep_111_3_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .Z(spi1_sck_c_enable_286)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam i2_2_lut_rep_111_3_lut.init = 16'h8080;
    LUT4 i7_4_lut_adj_305 (.A(ev_run_hold[69]), .B(us_tx_c_69), .C(init_shadow[69]), 
         .D(swap_now_d3), .Z(n11570)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_305.init = 16'h5a66;
    LUT4 i1_2_lut_rep_133 (.A(spi_byte_count[4]), .B(spi_byte_count[3]), 
         .Z(n23543)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_133.init = 16'heeee;
    LUT4 i14679_4_lut (.A(n23491), .B(fpga_cs_n_c), .C(spi1_sck_c_enable_286), 
         .D(frame_end), .Z(spi1_sck_c_enable_326)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i14679_4_lut.init = 16'h1000;
    LUT4 i1_2_lut_rep_105_3_lut (.A(spi_byte_count[4]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[2]), .Z(n23515)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_105_3_lut.init = 16'hfefe;
    LUT4 i1296_2_lut_rep_91_3_lut_4_lut (.A(spi_byte_count[4]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[5]), .D(spi_byte_count[2]), .Z(n23501)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C (D)))) */ ;
    defparam i1296_2_lut_rep_91_3_lut_4_lut.init = 16'hf0e0;
    LUT4 i7_4_lut_adj_306 (.A(ev_run_hold[70]), .B(us_tx_c_70), .C(init_shadow[70]), 
         .D(swap_now_d3), .Z(n11572)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_306.init = 16'h5a66;
    LUT4 i2118_2_lut_rep_134 (.A(ev_ch[1]), .B(ev_ch[2]), .Z(n23544)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i2118_2_lut_rep_134.init = 16'heeee;
    LUT4 i1_2_lut_adj_307 (.A(stop_toggle_spi), .B(n22099), .Z(stop_toggle_spi_N_2487)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_307.init = 16'h9999;
    LUT4 i7_4_lut_adj_308 (.A(ev_run_hold[71]), .B(us_tx_c_71), .C(init_shadow[71]), 
         .D(swap_now_d3), .Z(n11574)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_308.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_309 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[72]), 
         .D(ev_bit[72]), .Z(ev_wr_data[72])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_309.init = 16'hddd0;
    LUT4 i7_4_lut_adj_310 (.A(ev_run_hold[72]), .B(us_tx_c_72), .C(init_shadow[72]), 
         .D(swap_now_d3), .Z(n11576)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_310.init = 16'h5a66;
    LUT4 i7_4_lut_adj_311 (.A(ev_run_hold[73]), .B(us_tx_c_73), .C(init_shadow[73]), 
         .D(swap_now_d3), .Z(n11578)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_311.init = 16'h5a66;
    LUT4 i2114_2_lut_rep_135 (.A(ev_ch[1]), .B(ev_ch[2]), .Z(n23545)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i2114_2_lut_rep_135.init = 16'h2222;
    LUT4 i7_4_lut_adj_312 (.A(ev_run_hold[74]), .B(us_tx_c_74), .C(init_shadow[74]), 
         .D(swap_now_d3), .Z(n11580)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_312.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_313 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[73]), 
         .D(ev_bit[73]), .Z(ev_wr_data[73])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_313.init = 16'hddd0;
    LUT4 i1_2_lut_rep_136 (.A(ev_ch[5]), .B(ev_ch[6]), .Z(n23546)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i1_2_lut_rep_136.init = 16'heeee;
    LUT4 i7_4_lut_adj_314 (.A(ev_run_hold[0]), .B(us_tx_c_0), .C(init_shadow[0]), 
         .D(swap_now_d3), .Z(n10486)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_314.init = 16'h5a66;
    LUT4 i3_4_lut_adj_315 (.A(spi_byte_count[14]), .B(spi_byte_count[13]), 
         .C(spi_byte_count[15]), .D(n23519), .Z(n19458)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_315.init = 16'hfffe;
    CCU2D status_bit_index_1053_add_4_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22026), .S0(n35_adj_3029), 
          .S1(n34_adj_3028));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053_add_4_7.INIT0 = 16'hfaaa;
    defparam status_bit_index_1053_add_4_7.INIT1 = 16'hfaaa;
    defparam status_bit_index_1053_add_4_7.INJECT1_0 = "NO";
    defparam status_bit_index_1053_add_4_7.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_316 (.A(ev_run_hold[75]), .B(us_tx_c_75), .C(init_shadow[75]), 
         .D(swap_now_d3), .Z(n11582)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_316.init = 16'h5a66;
    LUT4 i7_4_lut_adj_317 (.A(ev_run_hold[76]), .B(us_tx_c_76), .C(init_shadow[76]), 
         .D(swap_now_d3), .Z(n11584)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_317.init = 16'h5a66;
    LUT4 i1_2_lut_rep_137 (.A(ev_ch[1]), .B(ev_ch[2]), .Z(n23547)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_rep_137.init = 16'h8888;
    LUT4 i7_4_lut_adj_318 (.A(ev_run_hold[77]), .B(us_tx_c_77), .C(init_shadow[77]), 
         .D(swap_now_d3), .Z(n11586)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_318.init = 16'h5a66;
    LUT4 i7_4_lut_adj_319 (.A(ev_run_hold[78]), .B(us_tx_c_78), .C(init_shadow[78]), 
         .D(swap_now_d3), .Z(n11588)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_319.init = 16'h5a66;
    LUT4 i13532_2_lut (.A(global_phase[0]), .B(phase_step_reg), .Z(next_global_phase[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13532_2_lut.init = 16'h6666;
    LUT4 i14704_3_lut (.A(n22614), .B(spi_byte_count[5]), .C(spi_byte_count[4]), 
         .Z(spi1_sck_c_enable_51)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i14704_3_lut.init = 16'h0404;
    LUT4 i1_3_lut_4_lut_adj_320 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[74]), 
         .D(ev_bit[74]), .Z(ev_wr_data[74])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_320.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_321 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[77]), 
         .D(ev_bit[77]), .Z(ev_wr_data[77])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_321.init = 16'hddd0;
    FD1S3IX ev_bit_i56 (.D(n14604), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i56.GSR = "DISABLED";
    FD1P3AX ev_state_i2 (.D(ev_state_3__N_645[2]), .SP(pll_clk_enable_565), 
            .CK(pll_clk), .Q(ev_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_state_i2.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_322 (.A(ev_run_hold[79]), .B(us_tx_c_79), .C(init_shadow[79]), 
         .D(swap_now_d3), .Z(n11590)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_322.init = 16'h5a66;
    FD1S3IX ev_bit_i52 (.D(n14600), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i52.GSR = "DISABLED";
    FD1P3AX ev_state_i1 (.D(ev_state_3__N_645[1]), .SP(pll_clk_enable_565), 
            .CK(pll_clk), .Q(ev_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_state_i1.GSR = "DISABLED";
    FD1S3IX ev_bit_i48 (.D(n14596), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i48.GSR = "DISABLED";
    FD1P3IX frame_settle__i3 (.D(frame_settle_3__N_1924[3]), .SP(pll_clk_enable_566), 
            .CD(pll_clk_enable_17), .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_138 (.A(ev_ch[5]), .B(ev_ch[6]), .Z(n23548)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_rep_138.init = 16'h2222;
    FD1S3IX ev_bit_i44 (.D(n14592), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i44.GSR = "DISABLED";
    FD1P3IX spi_channel_field_1055__i0 (.D(n15), .SP(spi1_sck_c_enable_325), 
            .CD(spi1_sck_c_enable_323), .CK(spi1_sck_c), .Q(spi_channel_field[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(317[78:102])
    defparam spi_channel_field_1055__i0.GSR = "ENABLED";
    LUT4 i1_2_lut_rep_139 (.A(ev_ch[1]), .B(ev_ch[2]), .Z(n23549)) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i1_2_lut_rep_139.init = 16'hbbbb;
    FD1S3IX ev_bit_i4 (.D(n15009), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i4.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_323 (.A(ev_run_hold[80]), .B(us_tx_c_80), .C(init_shadow[80]), 
         .D(swap_now_d3), .Z(n11592)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_323.init = 16'h5a66;
    LUT4 i1634_1_lut_rep_140 (.A(status_bit_index[2]), .Z(n23550)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i1634_1_lut_rep_140.init = 16'h5555;
    FD1S3IX ev_bit_i42 (.D(n14178), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i42.GSR = "DISABLED";
    CCU2D time_divider_1058_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22050), .S0(n35_adj_3036), 
          .S1(n34_adj_3037));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_1058_add_4_7.INIT1 = 16'hfaaa;
    defparam time_divider_1058_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_1058_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_141 (.A(ev_ch[6]), .B(ev_ch[5]), .Z(n23551)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam i1_2_lut_rep_141.init = 16'hbbbb;
    LUT4 i7_4_lut_adj_324 (.A(ev_run_hold[81]), .B(us_tx_c_81), .C(init_shadow[81]), 
         .D(swap_now_d3), .Z(n11594)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_324.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_325 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[76]), 
         .D(ev_bit[76]), .Z(ev_wr_data[76])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_325.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_326 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[78]), 
         .D(ev_bit[78]), .Z(ev_wr_data[78])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_326.init = 16'hddd0;
    LUT4 i7_4_lut_adj_327 (.A(ev_run_hold[82]), .B(us_tx_c_82), .C(init_shadow[82]), 
         .D(swap_now_d3), .Z(n11596)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_327.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_else_4_lut (.A(ev_state[3]), .B(ev_state[2]), .C(ev_state[1]), 
         .Z(n23564)) /* synthesis lut_function=(A+(B+!(C))) */ ;
    defparam i1_3_lut_4_lut_else_4_lut.init = 16'hefef;
    LUT4 i2_3_lut_4_lut_adj_328 (.A(ev_state[3]), .B(ev_state[2]), .C(n5_adj_3047), 
         .D(ev_state[0]), .Z(swap_pending_N_2519)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam i2_3_lut_4_lut_adj_328.init = 16'h0200;
    LUT4 i14565_3_lut_3_lut (.A(status_bit_index[4]), .B(n23086), .C(n23085), 
         .Z(n23087)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14565_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14599_3_lut_4_lut_4_lut (.A(status_bit_index[4]), .B(n23569), 
         .C(n23557), .D(status_hold[88]), .Z(n23054)) /* synthesis lut_function=(A (B)+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14599_3_lut_4_lut_4_lut.init = 16'hd888;
    LUT4 i14533_3_lut_3_lut (.A(status_bit_index[4]), .B(n23557), .C(status_hold[104]), 
         .Z(n23055)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14533_3_lut_3_lut.init = 16'hc4c4;
    LUT4 i14531_3_lut_3_lut (.A(status_bit_index[4]), .B(n23052), .C(n23051), 
         .Z(n23053)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14531_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_329 (.A(ev_run_hold[83]), .B(us_tx_c_83), .C(init_shadow[83]), 
         .D(swap_now_d3), .Z(n11598)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_329.init = 16'h5a66;
    LUT4 i1458_2_lut_rep_143 (.A(ev_ch[1]), .B(n23836), .Z(n23553)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1458_2_lut_rep_143.init = 16'h8888;
    CCU2D time_divider_1058_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22049), .COUT(n22050), .S0(n37_adj_3034), 
          .S1(n36_adj_3035));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_1058_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_1058_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_1058_add_4_5.INJECT1_1 = "NO";
    CCU2D status_bit_index_1053_add_4_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22025), .COUT(n22026), .S0(n37_adj_3038), 
          .S1(n36_adj_3030));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053_add_4_5.INIT0 = 16'hfaaa;
    defparam status_bit_index_1053_add_4_5.INIT1 = 16'hfaaa;
    defparam status_bit_index_1053_add_4_5.INJECT1_0 = "NO";
    defparam status_bit_index_1053_add_4_5.INJECT1_1 = "NO";
    LUT4 i1465_2_lut_rep_113_3_lut (.A(ev_ch[1]), .B(n23836), .C(ev_ch[2]), 
         .Z(n23523)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1465_2_lut_rep_113_3_lut.init = 16'h8080;
    LUT4 i1463_2_lut_3_lut (.A(ev_ch[1]), .B(n23836), .C(ev_ch[2]), .Z(ev_ch_6__N_1984[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1463_2_lut_3_lut.init = 16'h7878;
    LUT4 i1472_2_lut_rep_94_3_lut_4_lut (.A(ev_ch[1]), .B(n23836), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(n23504)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1472_2_lut_rep_94_3_lut_4_lut.init = 16'h8000;
    LUT4 i1470_2_lut_3_lut_4_lut (.A(ev_ch[1]), .B(n23836), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(ev_ch_6__N_1984[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1470_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i10558_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[0]), .C(n5_adj_3047), 
         .D(n28_adj_3041), .Z(n14_adj_3043)) /* synthesis lut_function=(!(A+!(B (C)+!B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(203[33:53])
    defparam i10558_4_lut_4_lut.init = 16'h5140;
    LUT4 i1_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[0]), 
         .D(n23497), .Z(pll_clk_enable_562)) /* synthesis lut_function=(!(A+!(B (C)+!B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(203[33:53])
    defparam i1_4_lut_4_lut.init = 16'h5140;
    LUT4 i1_3_lut_4_lut_adj_330 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[79]), 
         .D(ev_bit[79]), .Z(ev_wr_data[79])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_330.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_331 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[8]), 
         .D(ev_bit[8]), .Z(ev_wr_data[8])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_331.init = 16'hddd0;
    LUT4 i1519_2_lut_rep_144 (.A(frame_settle[1]), .B(frame_settle[0]), 
         .Z(n23554)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(387[29:48])
    defparam i1519_2_lut_rep_144.init = 16'heeee;
    LUT4 i1_2_lut_3_lut_adj_332 (.A(frame_settle[1]), .B(frame_settle[0]), 
         .C(frame_settle[2]), .Z(n14396)) /* synthesis lut_function=(A (C)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(387[29:48])
    defparam i1_2_lut_3_lut_adj_332.init = 16'he1e1;
    LUT4 i507_1_lut_rep_145 (.A(n23836), .Z(n23555)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i507_1_lut_rep_145.init = 16'h5555;
    LUT4 i4_4_lut_4_lut (.A(n23836), .B(ev_ch[1]), .C(n23528), .D(ev_ch[2]), 
         .Z(n10)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(481[19:33])
    defparam i4_4_lut_4_lut.init = 16'hfff7;
    FD1S3IX mic_divider_1060__i2 (.D(n38), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060__i2.GSR = "DISABLED";
    FD1S3IX mic_divider_1060__i3 (.D(n37_adj_3021), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060__i3.GSR = "DISABLED";
    FD1S3IX mic_divider_1060__i4 (.D(n36_adj_3022), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060__i4.GSR = "DISABLED";
    FD1S3IX mic_divider_1060__i5 (.D(n35_adj_3023), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060__i5.GSR = "DISABLED";
    FD1S3IX mic_divider_1060__i6 (.D(n34_adj_3024), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(517[28:46])
    defparam mic_divider_1060__i6.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1056__i1 (.D(n19), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam spi_bit_count_1056__i1.GSR = "ENABLED";
    FD1S3AX spi_bit_count_1056__i2 (.D(n18), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(341[34:54])
    defparam spi_bit_count_1056__i2.GSR = "ENABLED";
    FD1P3AX mic_sample_count_1059__i1 (.D(n29), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam mic_sample_count_1059__i1.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1059__i2 (.D(n28), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam mic_sample_count_1059__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1059__i3 (.D(n27), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam mic_sample_count_1059__i3.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1059__i4 (.D(n26), .SP(pll_clk_enable_574), 
            .CK(pll_clk), .Q(mic_sample_count[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(509[37:60])
    defparam mic_sample_count_1059__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1058__i1 (.D(n39_adj_3032), .CK(pll_clk), .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058__i1.GSR = "DISABLED";
    FD1S3AX time_divider_1058__i2 (.D(n38_adj_3033), .CK(pll_clk), .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058__i2.GSR = "DISABLED";
    FD1S3AX time_divider_1058__i3 (.D(n37_adj_3034), .CK(pll_clk), .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058__i3.GSR = "DISABLED";
    FD1S3AX time_divider_1058__i4 (.D(n36_adj_3035), .CK(pll_clk), .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1058__i5 (.D(n35_adj_3036), .CK(pll_clk), .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058__i5.GSR = "DISABLED";
    FD1S3AX time_divider_1058__i6 (.D(n34_adj_3037), .CK(pll_clk), .Q(time_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058__i6.GSR = "DISABLED";
    FD1S3IX ev_bit_i36 (.D(n15010), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i36.GSR = "DISABLED";
    FD1P3AX stop_toggle_spi_381 (.D(stop_toggle_spi_N_2487), .SP(spi1_sck_c_enable_326), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam stop_toggle_spi_381.GSR = "DISABLED";
    FD1S3IX ev_bit_i32 (.D(n14580), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i32.GSR = "DISABLED";
    FD1S3IX ev_bit_i28 (.D(n14427), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i28.GSR = "DISABLED";
    FD1S3IX ev_bit_i16 (.D(n14387), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i16.GSR = "DISABLED";
    FD1S3IX ev_bit_i80 (.D(n14345), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i80.GSR = "DISABLED";
    FD1S3IX ev_bit_i41 (.D(n14588), .CK(pll_clk), .CD(n23555), .Q(ev_bit[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i41.GSR = "DISABLED";
    CCU2D status_bit_index_1053_add_4_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22024), .COUT(n22025), .S0(n39_adj_3040), 
          .S1(n38_adj_3039));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[66:89])
    defparam status_bit_index_1053_add_4_3.INIT0 = 16'hfaaa;
    defparam status_bit_index_1053_add_4_3.INIT1 = 16'hfaaa;
    defparam status_bit_index_1053_add_4_3.INJECT1_0 = "NO";
    defparam status_bit_index_1053_add_4_3.INJECT1_1 = "NO";
    FD1S3IX ev_bit_i8 (.D(n14385), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i8.GSR = "DISABLED";
    FD1P3IX init_shadow_i41 (.D(n13095), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i41.GSR = "DISABLED";
    FD1P3IX init_shadow_i40 (.D(n13085), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i40.GSR = "DISABLED";
    FD1P3IX init_shadow_i39 (.D(n13079), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i39.GSR = "DISABLED";
    FD1P3IX init_shadow_i38 (.D(n13073), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i38.GSR = "DISABLED";
    LUT4 i1635_1_lut_rep_146 (.A(status_bit_index[1]), .Z(n23556)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i1635_1_lut_rep_146.init = 16'h5555;
    LUT4 i14549_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[61]), 
         .C(status_hold[60]), .Z(n23071)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14549_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14550_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[63]), 
         .C(status_hold[62]), .Z(n23072)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14550_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14545_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[53]), 
         .C(status_hold[52]), .Z(n23067)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14545_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14548_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[59]), 
         .C(status_hold[58]), .Z(n23070)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14548_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14547_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[57]), 
         .C(status_hold[56]), .Z(n23069)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14547_3_lut_3_lut.init = 16'he4e4;
    FD1P3IX init_shadow_i37 (.D(n13067), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i37.GSR = "DISABLED";
    PFUMX i14558 (.BLUT(n23071), .ALUT(n23072), .C0(n23556), .Z(n23080));
    LUT4 i5_3_lut (.A(ev_ch[3]), .B(n10), .C(ev_ch[4]), .Z(n5_adj_3047)) /* synthesis lut_function=(A+(B+!(C))) */ ;
    defparam i5_3_lut.init = 16'hefef;
    FD1S3IX ev_bit_i76 (.D(n14347), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i76.GSR = "DISABLED";
    LUT4 i14544_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[51]), 
         .C(status_hold[50]), .Z(n23066)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14544_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14543_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[49]), 
         .C(status_hold[48]), .Z(n23065)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14543_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14546_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[55]), 
         .C(status_hold[54]), .Z(n23068)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14546_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14540_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[43]), 
         .C(status_hold[42]), .Z(n23062)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14540_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14539_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[41]), 
         .C(status_hold[40]), .Z(n23061)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14539_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14538_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[39]), 
         .C(status_hold[38]), .Z(n23060)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14538_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14537_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[37]), 
         .C(status_hold[36]), .Z(n23059)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14537_3_lut_3_lut.init = 16'he4e4;
    CCU2D time_divider_1058_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n22048), .COUT(n22049), .S0(n39_adj_3032), 
          .S1(n38_adj_3033));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_1058_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_1058_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_1058_add_4_3.INJECT1_1 = "NO";
    LUT4 i14542_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[47]), 
         .C(status_hold[46]), .Z(n23064)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14542_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14536_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[35]), 
         .C(status_hold[34]), .Z(n23058)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14536_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14535_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[33]), 
         .C(status_hold[32]), .Z(n23057)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14535_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14516_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[31]), 
         .C(status_hold[30]), .Z(n23038)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14516_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14515_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[29]), 
         .C(status_hold[28]), .Z(n23037)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14515_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14514_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[27]), 
         .C(status_hold[26]), .Z(n23036)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14514_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14513_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[25]), 
         .C(status_hold[24]), .Z(n23035)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14513_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14512_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[23]), 
         .C(status_hold[22]), .Z(n23034)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14512_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14541_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[45]), 
         .C(status_hold[44]), .Z(n23063)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14541_3_lut_3_lut.init = 16'he4e4;
    LUT4 i3_4_lut_adj_333 (.A(spi_channel_index[0]), .B(spi_channel_index[1]), 
         .C(spi_channel_index[4]), .D(spi_channel_index[6]), .Z(n22069)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut_adj_333.init = 16'h8000;
    LUT4 i14511_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[21]), 
         .C(status_hold[20]), .Z(n23033)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14511_3_lut_3_lut.init = 16'he4e4;
    LUT4 run_addr_reg_8__I_0_i8_3_lut (.A(global_phase[7]), .B(ev_rd_slot[7]), 
         .C(n19130), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(210[33:75])
    defparam run_addr_reg_8__I_0_i8_3_lut.init = 16'hacac;
    LUT4 i3_4_lut_adj_334 (.A(spi1_sck_c_enable_286), .B(n98), .C(n19396), 
         .D(n22711), .Z(spi1_sck_c_enable_325)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i3_4_lut_adj_334.init = 16'h0800;
    LUT4 i14510_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[19]), 
         .C(status_hold[18]), .Z(n23032)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14510_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_2_lut_adj_335 (.A(spi_channel_index[3]), .B(spi_channel_index[5]), 
         .Z(n5_adj_3065)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[43:69])
    defparam i1_2_lut_adj_335.init = 16'heeee;
    LUT4 i14509_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[17]), 
         .C(status_hold[16]), .Z(n23031)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14509_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14508_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[15]), 
         .C(status_hold[14]), .Z(n23030)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14508_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14507_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[13]), 
         .C(status_hold[12]), .Z(n23029)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14507_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14506_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[11]), 
         .C(status_hold[10]), .Z(n23028)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14506_3_lut_3_lut.init = 16'he4e4;
    LUT4 i2_4_lut_4_lut_adj_336 (.A(spi_byte_count[2]), .B(spi_byte_count[1]), 
         .C(n4), .D(spi_byte_count[3]), .Z(n1)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i2_4_lut_4_lut_adj_336.init = 16'h1080;
    LUT4 i10507_3_lut (.A(n23836), .B(ev_state[3]), .C(ev_state[0]), .Z(ev_ch_6__N_657[0])) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i10507_3_lut.init = 16'h4848;
    LUT4 i14505_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[9]), 
         .C(status_hold[8]), .Z(n23027)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14505_3_lut_3_lut.init = 16'he4e4;
    LUT4 i2_4_lut_4_lut_adj_337 (.A(ev_state[0]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(ev_state[3]), .Z(ev_we)) /* synthesis lut_function=(!(A (B+(C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29] 213[49])
    defparam i2_4_lut_4_lut_adj_337.init = 16'h0242;
    LUT4 i1_3_lut_4_lut_adj_338 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[13]), 
         .D(ev_bit[13]), .Z(ev_wr_data[13])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_338.init = 16'hddd0;
    LUT4 i14504_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[7]), 
         .C(status_hold[6]), .Z(n23026)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14504_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_4_lut_4_lut_adj_339 (.A(ev_state[1]), .B(ev_state[0]), .C(ev_state[3]), 
         .D(ev_state[2]), .Z(pll_clk_enable_276)) /* synthesis lut_function=(!(A+(B (C+!(D))+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_4_lut_4_lut_adj_339.init = 16'h0410;
    LUT4 i14503_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[5]), 
         .C(status_hold[4]), .Z(n23025)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14503_3_lut_3_lut.init = 16'he4e4;
    LUT4 i119_4_lut (.A(spi_byte_count[7]), .B(n23516), .C(spi_byte_count[6]), 
         .D(n23501), .Z(n98)) /* synthesis lut_function=(!(A (B (C))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i119_4_lut.init = 16'h7f7a;
    LUT4 i14502_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[3]), 
         .C(status_hold[2]), .Z(n23024)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14502_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14501_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[1]), 
         .C(status_hold[0]), .Z(n23023)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i14501_3_lut_3_lut.init = 16'he4e4;
    LUT4 i4764_4_lut (.A(ev_ch[6]), .B(staging_rd_addr[6]), .C(n15663), 
         .D(n22670), .Z(staging_rd_addr_6__N_849[6])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i4764_4_lut.init = 16'hcac0;
    LUT4 i2_3_lut_rep_147 (.A(status_bit_index[2]), .B(status_bit_index[0]), 
         .C(status_bit_index[1]), .Z(n23557)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(264[55:80])
    defparam i2_3_lut_rep_147.init = 16'h8080;
    LUT4 i1_3_lut_4_lut_adj_340 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[14]), 
         .D(ev_bit[14]), .Z(ev_wr_data[14])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_340.init = 16'hddd0;
    LUT4 i1_2_lut_rep_150 (.A(ev_state[3]), .B(ev_state[2]), .Z(n23560)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_150.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_341 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[39]), 
         .D(ev_bit[39]), .Z(ev_wr_data[39])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_341.init = 16'hddd0;
    LUT4 i2_2_lut_3_lut_4_lut_adj_342 (.A(ev_state[3]), .B(ev_state[2]), 
         .C(n23513), .D(ev_state[1]), .Z(n17517)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i2_2_lut_3_lut_4_lut_adj_342.init = 16'h0010;
    LUT4 i1_4_lut_adj_343 (.A(n60), .B(n23494), .C(n14778), .D(n23533), 
         .Z(n14781)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(292[17] 324[24])
    defparam i1_4_lut_adj_343.init = 16'h3032;
    LUT4 i7_4_lut_adj_344 (.A(ev_clear_addr[3]), .B(n14_adj_3048), .C(n10_adj_3049), 
         .D(ev_clear_addr[6]), .Z(ev_state_3__N_1976[1])) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut_adj_344.init = 16'h8000;
    LUT4 i1_2_lut_rep_112_3_lut (.A(ev_state[3]), .B(ev_state[2]), .C(ev_state[1]), 
         .Z(n23522)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_112_3_lut.init = 16'hfefe;
    LUT4 i14361_2_lut_rep_151 (.A(ev_state[0]), .B(ev_state[1]), .Z(n23561)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14361_2_lut_rep_151.init = 16'h8888;
    LUT4 i5_4_lut_rep_159 (.A(spi_byte_count[7]), .B(n7_adj_3056), .C(n22667), 
         .D(n8_adj_3055), .Z(spi1_sck_c_enable_225)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut_rep_159.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_345 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[80]), 
         .D(ev_bit[80]), .Z(ev_wr_data[80])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_345.init = 16'hddd0;
    LUT4 i9_4_lut_adj_346 (.A(n17), .B(phase_step_reg), .C(n16), .D(global_phase[4]), 
         .Z(swap_now)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i9_4_lut_adj_346.init = 16'h8000;
    LUT4 i2_3_lut_4_lut_rep_161 (.A(status_flags_wire_15__N_1349[4]), .B(pll_locked), 
         .C(pll_clk_enable_17), .D(phase_step_d3), .Z(pll_clk_enable_128)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_rep_161.init = 16'hfff7;
    LUT4 i2_2_lut_rep_116_3_lut_4_lut (.A(ev_state[0]), .B(ev_state[1]), 
         .C(ev_state[3]), .D(ev_state[2]), .Z(n23526)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;
    defparam i2_2_lut_rep_116_3_lut_4_lut.init = 16'hf7ff;
    LUT4 i1_3_lut_4_lut_adj_347 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[81]), 
         .D(ev_bit[81]), .Z(ev_wr_data[81])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_347.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_348 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[40]), 
         .D(ev_bit[40]), .Z(ev_wr_data[40])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_348.init = 16'hddd0;
    LUT4 i1_2_lut_3_lut_adj_349 (.A(spi_channel_field[1]), .B(spi1_sck_c_enable_325), 
         .C(spi_channel_field[0]), .Z(spi1_sck_c_enable_323)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_349.init = 16'h4040;
    CCU2D time_divider_1058_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n22048), .S1(n40_adj_3031));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(369[29:48])
    defparam time_divider_1058_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_1058_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_1058_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_1058_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_350 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[41]), 
         .D(ev_bit[41]), .Z(ev_wr_data[41])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_350.init = 16'hddd0;
    LUT4 i2_2_lut_rep_152 (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .Z(n23562)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_rep_152.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_351 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[42]), 
         .D(ev_bit[42]), .Z(ev_wr_data[42])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_351.init = 16'hddd0;
    LUT4 i13504_2_lut_3_lut_4_lut (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .C(mic_sample_count[3]), .D(mic_sample_count[2]), .Z(n27)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;
    defparam i13504_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i6_4_lut_adj_352 (.A(ev_clear_addr[2]), .B(ev_clear_addr[1]), .C(ev_clear_addr[4]), 
         .D(ev_clear_addr[7]), .Z(n14_adj_3048)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_352.init = 16'h8000;
    LUT4 i5_3_lut_4_lut (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .C(mic_tick), .D(mic_sample_count[2]), .Z(n12_adj_3025)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_3_lut_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_353 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[43]), 
         .D(ev_bit[43]), .Z(ev_wr_data[43])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_353.init = 16'hddd0;
    LUT4 i13497_2_lut_3_lut (.A(mic_sample_count[0]), .B(mic_sample_count[1]), 
         .C(mic_sample_count[2]), .Z(n28)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;
    defparam i13497_2_lut_3_lut.init = 16'h7878;
    LUT4 i14693_2_lut_4_lut (.A(ev_state[2]), .B(ev_state[1]), .C(ev_state[0]), 
         .D(ev_state[3]), .Z(pll_clk_enable_193)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i14693_2_lut_4_lut.init = 16'h0400;
    LUT4 i1_3_lut_4_lut_adj_354 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[44]), 
         .D(ev_bit[44]), .Z(ev_wr_data[44])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_354.init = 16'hddd0;
    LUT4 mux_1430_i14_3_lut (.A(n9935), .B(n9936), .C(n9908), .Z(rd_data_15__N_2579[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i14_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_355 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[45]), 
         .D(ev_bit[45]), .Z(ev_wr_data[45])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_355.init = 16'hddd0;
    LUT4 i1_4_lut_4_lut_rep_163 (.A(ev_state[1]), .B(ev_state[0]), .C(ev_state[3]), 
         .D(ev_state[2]), .Z(pll_clk_enable_242)) /* synthesis lut_function=(!(A+(B (C+!(D))+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i1_4_lut_4_lut_rep_163.init = 16'h0410;
    LUT4 i2_2_lut_adj_356 (.A(ev_clear_addr[0]), .B(ev_clear_addr[5]), .Z(n10_adj_3049)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut_adj_356.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_357 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[82]), 
         .D(ev_bit[82]), .Z(ev_wr_data[82])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_357.init = 16'hddd0;
    LUT4 i6_4_lut_rep_165 (.A(mic_sample_count[3]), .B(n12_adj_3025), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_399)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_rep_165.init = 16'h0800;
    LUT4 i1_3_lut_4_lut_adj_358 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[83]), 
         .D(ev_bit[83]), .Z(ev_wr_data[83])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_358.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_359 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[46]), 
         .D(ev_bit[46]), .Z(ev_wr_data[46])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_359.init = 16'hddd0;
    LUT4 i14712_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_58)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i14712_2_lut_3_lut.init = 16'h4040;
    LUT4 i2_3_lut_adj_360 (.A(swap_now_d1), .B(swap_now), .C(active_bank), 
         .Z(run_bank)) /* synthesis lut_function=(A (B (C)+!B !(C))+!A !(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:69])
    defparam i2_3_lut_adj_360.init = 16'h9696;
    LUT4 i1_3_lut_4_lut_adj_361 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[47]), 
         .D(ev_bit[47]), .Z(ev_wr_data[47])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_361.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_362 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[0]), 
         .D(ev_bit[0]), .Z(ev_wr_data[0])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_362.init = 16'hddd0;
    LUT4 i7_4_lut_adj_363 (.A(swap_pending), .B(global_phase[5]), .C(global_phase[7]), 
         .D(global_phase[0]), .Z(n17)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut_adj_363.init = 16'h8000;
    LUT4 i7259_2_lut_4_lut_rep_168 (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n23518), .Z(n23838)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i7259_2_lut_4_lut_rep_168.init = 16'h0100;
    LUT4 i1_3_lut_4_lut_adj_364 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[1]), 
         .D(ev_bit[1]), .Z(ev_wr_data[1])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_364.init = 16'hddd0;
    LUT4 i14710_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_74)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i14710_2_lut_3_lut.init = 16'h0404;
    LUT4 i6_4_lut_adj_365 (.A(global_phase[2]), .B(global_phase[6]), .C(global_phase[3]), 
         .D(global_phase[1]), .Z(n16)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_365.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_4_lut_rep_170 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1349[4]), .Z(n23840)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(238[23:61])
    defparam i1_2_lut_3_lut_4_lut_rep_170.init = 16'h6fff;
    LUT4 mux_1430_i2_3_lut (.A(n9911), .B(n9912), .C(n9908), .Z(rd_data_15__N_2579[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i2_3_lut.init = 16'hcaca;
    LUT4 i590_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_322)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(516[18] 518[12])
    defparam i590_2_lut.init = 16'h4444;
    LUT4 mux_1430_i3_3_lut (.A(n9913), .B(n9914), .C(n9908), .Z(rd_data_15__N_2579[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_adj_366 (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_66)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i1_2_lut_3_lut_adj_366.init = 16'h8080;
    LUT4 mux_1430_i8_3_lut (.A(n9923), .B(n9924), .C(n9908), .Z(rd_data_15__N_2579[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i8_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_367 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[2]), 
         .D(ev_bit[2]), .Z(ev_wr_data[2])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_367.init = 16'hddd0;
    LUT4 mux_1430_i6_3_lut (.A(n9919), .B(n9920), .C(n9908), .Z(rd_data_15__N_2579[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i6_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_368 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[3]), 
         .D(ev_bit[3]), .Z(ev_wr_data[3])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_368.init = 16'hddd0;
    LUT4 i13533_2_lut (.A(staging_q[8]), .B(staging_q[0]), .Z(build_sum_8__N_2005[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13533_2_lut.init = 16'h6666;
    LUT4 mux_1430_i7_3_lut (.A(n9921), .B(n9922), .C(n9908), .Z(rd_data_15__N_2579[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i7_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_4_lut_then_3_lut (.A(status_bit_index[1]), .B(status_hold[74]), 
         .C(status_bit_index[0]), .Z(n23568)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(268[18:44])
    defparam i1_4_lut_4_lut_then_3_lut.init = 16'h4040;
    LUT4 i85_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2502)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(288[18] 343[12])
    defparam i85_1_lut.init = 16'h5555;
    LUT4 mux_1430_i12_3_lut (.A(n9931), .B(n9932), .C(n9908), .Z(rd_data_15__N_2579[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i12_3_lut.init = 16'hcaca;
    LUT4 mux_1430_i4_3_lut (.A(n9915), .B(n9916), .C(n9908), .Z(rd_data_15__N_2579[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i4_3_lut.init = 16'hcaca;
    LUT4 mux_1430_i13_3_lut (.A(n9933), .B(n9934), .C(n9908), .Z(rd_data_15__N_2579[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i13_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_369 (.A(ev_state[0]), .B(n23522), .C(ev_rd_hold[11]), 
         .D(ev_bit[11]), .Z(ev_wr_data[11])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(212[29:51])
    defparam i1_3_lut_4_lut_adj_369.init = 16'hddd0;
    INV i14934 (.A(spi1_sck_c), .Z(spi1_sck_N_457));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:32])
    LUT4 mux_1430_i5_3_lut (.A(n9917), .B(n9918), .C(n9908), .Z(rd_data_15__N_2579[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i5_3_lut.init = 16'hcaca;
    VLO i1 (.Z(GND_net));
    LUT4 mux_1430_i10_3_lut (.A(n9927), .B(n9928), .C(n9908), .Z(rd_data_15__N_2579[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i10_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_rep_92 (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n23518), .Z(pll_clk_enable_511)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;
    defparam i1_4_lut_rep_92.init = 16'h5150;
    TSALL TSALL_INST (.TSALL(GND_net));
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    LUT4 i7259_2_lut_4_lut (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n23518), .Z(n15793)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i7259_2_lut_4_lut.init = 16'h0100;
    LUT4 i1_2_lut_adj_370 (.A(ev_state[0]), .B(ev_state[3]), .Z(n22693)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut_adj_370.init = 16'h4444;
    LUT4 mux_1430_i9_3_lut (.A(n9925), .B(n9926), .C(n9908), .Z(rd_data_15__N_2579[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i9_3_lut.init = 16'hcaca;
    ws2812_stream ws2812_i (.\rgb_hold[93] (rgb_hold[93]), .\rgb_hold[94] (rgb_hold[94]), 
            .\rgb_hold[95] (rgb_hold[95]), .\rgb_hold[80] (rgb_hold[80]), 
            .\rgb_hold[81] (rgb_hold[81]), .\rgb_hold[82] (rgb_hold[82]), 
            .\rgb_hold[83] (rgb_hold[83]), .\rgb_hold[84] (rgb_hold[84]), 
            .\rgb_hold[73] (rgb_hold[73]), .\rgb_hold[74] (rgb_hold[74]), 
            .\rgb_hold[75] (rgb_hold[75]), .\rgb_hold[85] (rgb_hold[85]), 
            .\rgb_hold[76] (rgb_hold[76]), .\rgb_hold[86] (rgb_hold[86]), 
            .\rgb_hold[87] (rgb_hold[87]), .\rgb_hold[77] (rgb_hold[77]), 
            .\rgb_hold[78] (rgb_hold[78]), .\rgb_hold[79] (rgb_hold[79]), 
            .\rgb_hold[88] (rgb_hold[88]), .\rgb_hold[89] (rgb_hold[89]), 
            .\rgb_hold[90] (rgb_hold[90]), .\rgb_hold[91] (rgb_hold[91]), 
            .\rgb_hold[92] (rgb_hold[92]), .GND_net(GND_net), .rgb_data_c(rgb_data_c), 
            .pll_clk(pll_clk), .\rgb_hold[72] (rgb_hold[72])) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(523[19] 527[6])
    LUT4 i10877_2_lut_4_lut (.A(n23520), .B(spi_byte_count[9]), .C(n23519), 
         .D(n14781), .Z(n19396)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i10877_2_lut_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut_3_lut_adj_371 (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_82)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_371.init = 16'h0808;
    FD1P3AX ev_ch_i0_rep_166 (.D(ev_ch_6__N_657[0]), .SP(pll_clk_enable_580), 
            .CK(pll_clk), .Q(n23836)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_ch_i0_rep_166.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_372 (.A(ev_state[0]), .B(ev_state[3]), .Z(n22709)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam i1_2_lut_adj_372.init = 16'h2222;
    FD1P3IX init_shadow_i36 (.D(n13061), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i36.GSR = "DISABLED";
    FD1P3IX init_shadow_i35 (.D(n13055), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i35.GSR = "DISABLED";
    FD1P3IX init_shadow_i34 (.D(n13049), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i34.GSR = "DISABLED";
    FD1P3IX init_shadow_i33 (.D(n13043), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i33.GSR = "DISABLED";
    FD1P3IX init_shadow_i32 (.D(n13037), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i32.GSR = "DISABLED";
    FD1P3IX init_shadow_i31 (.D(n13025), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i31.GSR = "DISABLED";
    FD1P3IX init_shadow_i30 (.D(n13004), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i30.GSR = "DISABLED";
    FD1P3IX init_shadow_i29 (.D(n12990), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i29.GSR = "DISABLED";
    LUT4 i2894_4_lut (.A(n23836), .B(staging_rd_addr[0]), .C(n15663), 
         .D(n22670), .Z(staging_rd_addr_6__N_849[0])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(408[9] 479[16])
    defparam i2894_4_lut.init = 16'hcac0;
    FD1P3IX init_shadow_i28 (.D(n12976), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i28.GSR = "DISABLED";
    FD1P3IX init_shadow_i27 (.D(n12960), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i27.GSR = "DISABLED";
    FD1P3IX init_shadow_i26 (.D(n12945), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i26.GSR = "DISABLED";
    FD1P3IX init_shadow_i25 (.D(n12924), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i25.GSR = "DISABLED";
    FD1P3IX init_shadow_i24 (.D(n12912), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i24.GSR = "DISABLED";
    FD1P3IX init_shadow_i23 (.D(n12906), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i23.GSR = "DISABLED";
    FD1P3IX init_shadow_i22 (.D(n12900), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i22.GSR = "DISABLED";
    FD1P3IX init_shadow_i21 (.D(n12894), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i21.GSR = "DISABLED";
    LUT4 i10471_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_574)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10471_2_lut.init = 16'h8888;
    FD1P3IX init_shadow_i20 (.D(n12888), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i20.GSR = "DISABLED";
    FD1P3IX init_shadow_i19 (.D(n12882), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i19.GSR = "DISABLED";
    FD1P3IX init_shadow_i18 (.D(n12876), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i18.GSR = "DISABLED";
    FD1P3IX init_shadow_i17 (.D(n12870), .SP(pll_clk_enable_600), .CD(n23838), 
            .CK(pll_clk), .Q(init_shadow[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam init_shadow_i17.GSR = "DISABLED";
    LUT4 i14685_4_lut (.A(mic_divider[6]), .B(mic_divider[5]), .C(mic_divider[4]), 
         .D(n22938), .Z(mic_tick_N_2507)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(496[21:43])
    defparam i14685_4_lut.init = 16'h0100;
    LUT4 i14417_4_lut (.A(mic_divider[1]), .B(mic_divider[2]), .C(mic_divider[0]), 
         .D(mic_divider[3]), .Z(n22938)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14417_4_lut.init = 16'h8000;
    LUT4 mic_clk_I_0_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(mic_clk_N_2472)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(516[18] 518[12])
    defparam mic_clk_I_0_2_lut.init = 16'h6666;
    GSR GSR_INST (.GSR(fpga_cs_n_N_2502));
    FD1S3IX ev_bit_i72 (.D(n14383), .CK(pll_clk), .CD(ev_ch[0]), .Q(ev_bit[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(349[12] 519[8])
    defparam ev_bit_i72.GSR = "DISABLED";
    LUT4 i6_4_lut_adj_373 (.A(mic_sample_count[3]), .B(n12_adj_3025), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_413)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_adj_373.init = 16'h0800;
    LUT4 m1_lut (.Z(n23820)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    LUT4 i1_2_lut_adj_374 (.A(ev_state[1]), .B(ev_state[0]), .Z(n22670)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_374.init = 16'h2222;
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .\ev_wr_addr[0] (ev_wr_addr[0]), .event_rd_addr({event_rd_addr}), 
            .\ev_wr_addr[1] (ev_wr_addr[1]), .\ev_wr_addr[2] (ev_wr_addr[2]), 
            .\ev_wr_addr[3] (ev_wr_addr[3]), .\ev_wr_addr[4] (ev_wr_addr[4]), 
            .\ev_wr_addr[5] (ev_wr_addr[5]), .\ev_wr_addr[6] (ev_wr_addr[6]), 
            .\ev_wr_addr[7] (ev_wr_addr[7]), .n23532(n23532), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(220[22] 223[6])
    PFUMX i14819 (.BLUT(n23567), .ALUT(n23568), .C0(status_bit_index[2]), 
          .Z(n23569));
    LUT4 mux_1430_i15_3_lut (.A(n9937), .B(n9938), .C(n9908), .Z(rd_data_15__N_2579[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1430_i15_3_lut.init = 16'hcaca;
    umh_channel_ram18 staging_ram (.staging_q({staging_q}), .pll_clk(pll_clk), 
            .rd_data_15__N_2579({rd_data_15__N_2579}), .n9894(n9894), .spi1_sck_c(spi1_sck_c), 
            .spi_channel_index({spi_channel_index}), .n9892(n9892), .spi_write(spi_write), 
            .VCC_net(VCC_net), .GND_net(GND_net), .staging_rd_addr_6__N_849({staging_rd_addr_6__N_849}), 
            .spi1_mosi_c_0(spi1_mosi_c_0), .\spi_rx_shift[0] (spi_rx_shift[0]), 
            .\spi_rx_shift[1] (spi_rx_shift[1]), .\spi_rx_shift[2] (spi_rx_shift[2]), 
            .\spi_rx_shift[3] (spi_rx_shift[3]), .\spi_rx_shift[4] (spi_rx_shift[4]), 
            .\spi_rx_shift[5] (spi_rx_shift[5]), .\spi_rx_shift[6] (spi_rx_shift[6]), 
            .spi_phase_pending({spi_phase_pending}), .n9909(n9909), .n9911(n9911), 
            .n9913(n9913), .n9915(n9915), .n9917(n9917), .n9919(n9919), 
            .n9921(n9921), .n9923(n9923), .n9925(n9925), .n9927(n9927), 
            .n9929(n9929), .n9931(n9931), .n9933(n9933), .n9935(n9935), 
            .n9937(n9937), .n9939(n9939), .n9896(n9896), .n9898(n9898), 
            .n9900(n9900), .n9902(n9902), .n9904(n9904)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(194[23] 197[6])
    LUT4 i14481_4_lut (.A(spi_version[6]), .B(n22964), .C(n22854), .D(spi_rx_shift[0]), 
         .Z(n23002)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14481_4_lut.init = 16'hfffe;
    
endmodule
//
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (sck_N_2908, spi_mic_cs_n_c, mic_latest, spi_mic_miso_c) /* synthesis syn_module_defined=1 */ ;
    input sck_N_2908;
    input spi_mic_cs_n_c;
    input [63:0]mic_latest;
    output spi_mic_miso_c;
    
    wire sck_N_2908 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [95:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
    wire sck_N_2908_enable_101, n15951;
    wire [6:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(12[11:20])
    
    wire n23527, n13, n15911, n15905, n15913;
    wire [6:0]bit_count_6__N_2909;
    
    wire n15909, n15907;
    wire [95:0]shift_register_95__N_2811;
    
    wire n15933, n15969, n15967, n23563, n15949, n23496, n15965, 
        n15947, n15963, n15945, n15943, n15941, n15961, n15959, 
        n15931, n15929, n15939, n15927, n15925, n15937, n15923, 
        n12, n15921, n15919, n15935, n15917, n15915, n15957, n15955, 
        n15953, n23512;
    
    FD1P3DX shift_register_i70 (.D(n15951), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[70])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i70.GSR = "DISABLED";
    LUT4 i10527_3_lut_4_lut (.A(bit_count[3]), .B(n23527), .C(n13), .D(bit_count[4]), 
         .Z(n15911)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10527_3_lut_4_lut.init = 16'h7080;
    LUT4 i10514_3_lut (.A(bit_count[1]), .B(n13), .C(bit_count[0]), .Z(n15905)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10514_3_lut.init = 16'h4848;
    FD1P3DX bit_count_i5 (.D(n15913), .SP(sck_N_2908_enable_101), .CK(sck_N_2908), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i5.GSR = "DISABLED";
    FD1P3DX bit_count_i4 (.D(n15911), .SP(sck_N_2908_enable_101), .CK(sck_N_2908), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i4.GSR = "DISABLED";
    FD1S3DX bit_count_i0 (.D(bit_count_6__N_2909[0]), .CK(sck_N_2908), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i0.GSR = "DISABLED";
    FD1P3DX bit_count_i3 (.D(n15909), .SP(sck_N_2908_enable_101), .CK(sck_N_2908), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i3.GSR = "DISABLED";
    FD1P3DX bit_count_i2 (.D(n15907), .SP(sck_N_2908_enable_101), .CK(sck_N_2908), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i2.GSR = "DISABLED";
    FD1P3DX bit_count_i1 (.D(n15905), .SP(sck_N_2908_enable_101), .CK(sck_N_2908), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 i10520_2_lut (.A(mic_latest[0]), .B(n13), .Z(shift_register_95__N_2811[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10520_2_lut.init = 16'h2222;
    LUT4 shift_register_95__I_0_19_i3_3_lut (.A(mic_latest[1]), .B(shift_register[1]), 
         .C(n13), .Z(shift_register_95__N_2811[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i3_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i4_3_lut (.A(mic_latest[2]), .B(shift_register[2]), 
         .C(n13), .Z(shift_register_95__N_2811[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i4_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i5_3_lut (.A(mic_latest[3]), .B(shift_register[3]), 
         .C(n13), .Z(shift_register_95__N_2811[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i5_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i6_3_lut (.A(mic_latest[4]), .B(shift_register[4]), 
         .C(n13), .Z(shift_register_95__N_2811[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i6_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i7_3_lut (.A(mic_latest[5]), .B(shift_register[5]), 
         .C(n13), .Z(shift_register_95__N_2811[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i7_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i8_3_lut (.A(mic_latest[6]), .B(shift_register[6]), 
         .C(n13), .Z(shift_register_95__N_2811[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i8_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i9_3_lut (.A(mic_latest[7]), .B(shift_register[7]), 
         .C(n13), .Z(shift_register_95__N_2811[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i9_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i10_3_lut (.A(mic_latest[8]), .B(shift_register[8]), 
         .C(n13), .Z(shift_register_95__N_2811[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i10_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i11_3_lut (.A(mic_latest[9]), .B(shift_register[9]), 
         .C(n13), .Z(shift_register_95__N_2811[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i11_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i12_3_lut (.A(mic_latest[10]), .B(shift_register[10]), 
         .C(n13), .Z(shift_register_95__N_2811[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i12_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i13_3_lut (.A(mic_latest[11]), .B(shift_register[11]), 
         .C(n13), .Z(shift_register_95__N_2811[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i13_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i14_3_lut (.A(mic_latest[12]), .B(shift_register[12]), 
         .C(n13), .Z(shift_register_95__N_2811[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i14_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i15_3_lut (.A(mic_latest[13]), .B(shift_register[13]), 
         .C(n13), .Z(shift_register_95__N_2811[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i15_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i16_3_lut (.A(mic_latest[14]), .B(shift_register[14]), 
         .C(n13), .Z(shift_register_95__N_2811[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i16_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i17_3_lut (.A(mic_latest[15]), .B(shift_register[15]), 
         .C(n13), .Z(shift_register_95__N_2811[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i17_3_lut.init = 16'hcaca;
    LUT4 i10521_2_lut (.A(shift_register[16]), .B(n13), .Z(shift_register_95__N_2811[17])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10521_2_lut.init = 16'hbbbb;
    LUT4 i1_3_lut (.A(n13), .B(shift_register[95]), .C(spi_mic_cs_n_c), 
         .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[15] 26[68])
    defparam i1_3_lut.init = 16'h0808;
    LUT4 i10522_2_lut (.A(shift_register[17]), .B(n13), .Z(shift_register_95__N_2811[18])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10522_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i26_3_lut (.A(mic_latest[16]), .B(shift_register[24]), 
         .C(n13), .Z(shift_register_95__N_2811[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i26_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i27_3_lut (.A(mic_latest[17]), .B(shift_register[25]), 
         .C(n13), .Z(shift_register_95__N_2811[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i27_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i28_3_lut (.A(mic_latest[18]), .B(shift_register[26]), 
         .C(n13), .Z(shift_register_95__N_2811[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i28_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i29_3_lut (.A(mic_latest[19]), .B(shift_register[27]), 
         .C(n13), .Z(shift_register_95__N_2811[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i29_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i30_3_lut (.A(mic_latest[20]), .B(shift_register[28]), 
         .C(n13), .Z(shift_register_95__N_2811[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i30_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i31_3_lut (.A(mic_latest[21]), .B(shift_register[29]), 
         .C(n13), .Z(shift_register_95__N_2811[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i31_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i32_3_lut (.A(mic_latest[22]), .B(shift_register[30]), 
         .C(n13), .Z(shift_register_95__N_2811[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i32_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i33_3_lut (.A(mic_latest[23]), .B(shift_register[31]), 
         .C(n13), .Z(shift_register_95__N_2811[32])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i33_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i34_3_lut (.A(mic_latest[24]), .B(shift_register[32]), 
         .C(n13), .Z(shift_register_95__N_2811[33])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i34_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i35_3_lut (.A(mic_latest[25]), .B(shift_register[33]), 
         .C(n13), .Z(shift_register_95__N_2811[34])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i35_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i36_3_lut (.A(mic_latest[26]), .B(shift_register[34]), 
         .C(n13), .Z(shift_register_95__N_2811[35])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i36_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i37_3_lut (.A(mic_latest[27]), .B(shift_register[35]), 
         .C(n13), .Z(shift_register_95__N_2811[36])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i37_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i38_3_lut (.A(mic_latest[28]), .B(shift_register[36]), 
         .C(n13), .Z(shift_register_95__N_2811[37])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i38_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i39_3_lut (.A(mic_latest[29]), .B(shift_register[37]), 
         .C(n13), .Z(shift_register_95__N_2811[38])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i39_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i40_3_lut (.A(mic_latest[30]), .B(shift_register[38]), 
         .C(n13), .Z(shift_register_95__N_2811[39])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i40_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i41_3_lut (.A(mic_latest[31]), .B(shift_register[39]), 
         .C(n13), .Z(shift_register_95__N_2811[40])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i41_3_lut.init = 16'hcaca;
    LUT4 i10523_2_lut (.A(shift_register[41]), .B(n13), .Z(shift_register_95__N_2811[42])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10523_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i50_3_lut (.A(mic_latest[32]), .B(shift_register[48]), 
         .C(n13), .Z(shift_register_95__N_2811[49])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i50_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i51_3_lut (.A(mic_latest[33]), .B(shift_register[49]), 
         .C(n13), .Z(shift_register_95__N_2811[50])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i51_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i52_3_lut (.A(mic_latest[34]), .B(shift_register[50]), 
         .C(n13), .Z(shift_register_95__N_2811[51])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i52_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i53_3_lut (.A(mic_latest[35]), .B(shift_register[51]), 
         .C(n13), .Z(shift_register_95__N_2811[52])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i53_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i54_3_lut (.A(mic_latest[36]), .B(shift_register[52]), 
         .C(n13), .Z(shift_register_95__N_2811[53])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i54_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i55_3_lut (.A(mic_latest[37]), .B(shift_register[53]), 
         .C(n13), .Z(shift_register_95__N_2811[54])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i55_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i56_3_lut (.A(mic_latest[38]), .B(shift_register[54]), 
         .C(n13), .Z(shift_register_95__N_2811[55])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i56_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i57_3_lut (.A(mic_latest[39]), .B(shift_register[55]), 
         .C(n13), .Z(shift_register_95__N_2811[56])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i57_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i58_3_lut (.A(mic_latest[40]), .B(shift_register[56]), 
         .C(n13), .Z(shift_register_95__N_2811[57])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i58_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i59_3_lut (.A(mic_latest[41]), .B(shift_register[57]), 
         .C(n13), .Z(shift_register_95__N_2811[58])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i59_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i60_3_lut (.A(mic_latest[42]), .B(shift_register[58]), 
         .C(n13), .Z(shift_register_95__N_2811[59])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i60_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i61_3_lut (.A(mic_latest[43]), .B(shift_register[59]), 
         .C(n13), .Z(shift_register_95__N_2811[60])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i61_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i62_3_lut (.A(mic_latest[44]), .B(shift_register[60]), 
         .C(n13), .Z(shift_register_95__N_2811[61])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i62_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i63_3_lut (.A(mic_latest[45]), .B(shift_register[61]), 
         .C(n13), .Z(shift_register_95__N_2811[62])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i63_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i64_3_lut (.A(mic_latest[46]), .B(shift_register[62]), 
         .C(n13), .Z(shift_register_95__N_2811[63])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i64_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i65_3_lut (.A(mic_latest[47]), .B(shift_register[63]), 
         .C(n13), .Z(shift_register_95__N_2811[64])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i65_3_lut.init = 16'hcaca;
    LUT4 i10524_2_lut (.A(shift_register[64]), .B(n13), .Z(shift_register_95__N_2811[65])) /* synthesis lut_function=(A+!(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam i10524_2_lut.init = 16'hbbbb;
    LUT4 shift_register_95__I_0_19_i74_3_lut (.A(mic_latest[48]), .B(shift_register[72]), 
         .C(n13), .Z(shift_register_95__N_2811[73])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i74_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i75_3_lut (.A(mic_latest[49]), .B(shift_register[73]), 
         .C(n13), .Z(shift_register_95__N_2811[74])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i75_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i76_3_lut (.A(mic_latest[50]), .B(shift_register[74]), 
         .C(n13), .Z(shift_register_95__N_2811[75])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i76_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i77_3_lut (.A(mic_latest[51]), .B(shift_register[75]), 
         .C(n13), .Z(shift_register_95__N_2811[76])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i77_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i78_3_lut (.A(mic_latest[52]), .B(shift_register[76]), 
         .C(n13), .Z(shift_register_95__N_2811[77])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i78_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i79_3_lut (.A(mic_latest[53]), .B(shift_register[77]), 
         .C(n13), .Z(shift_register_95__N_2811[78])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i79_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i80_3_lut (.A(mic_latest[54]), .B(shift_register[78]), 
         .C(n13), .Z(shift_register_95__N_2811[79])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i80_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i81_3_lut (.A(mic_latest[55]), .B(shift_register[79]), 
         .C(n13), .Z(shift_register_95__N_2811[80])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i81_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i82_3_lut (.A(mic_latest[56]), .B(shift_register[80]), 
         .C(n13), .Z(shift_register_95__N_2811[81])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i82_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i83_3_lut (.A(mic_latest[57]), .B(shift_register[81]), 
         .C(n13), .Z(shift_register_95__N_2811[82])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i83_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i84_3_lut (.A(mic_latest[58]), .B(shift_register[82]), 
         .C(n13), .Z(shift_register_95__N_2811[83])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i84_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i85_3_lut (.A(mic_latest[59]), .B(shift_register[83]), 
         .C(n13), .Z(shift_register_95__N_2811[84])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i85_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i86_3_lut (.A(mic_latest[60]), .B(shift_register[84]), 
         .C(n13), .Z(shift_register_95__N_2811[85])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i86_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i87_3_lut (.A(mic_latest[61]), .B(shift_register[85]), 
         .C(n13), .Z(shift_register_95__N_2811[86])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i87_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i88_3_lut (.A(mic_latest[62]), .B(shift_register[86]), 
         .C(n13), .Z(shift_register_95__N_2811[87])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i88_3_lut.init = 16'hcaca;
    LUT4 shift_register_95__I_0_19_i89_3_lut (.A(mic_latest[63]), .B(shift_register[87]), 
         .C(n13), .Z(shift_register_95__N_2811[88])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(35[14] 38[8])
    defparam shift_register_95__I_0_19_i89_3_lut.init = 16'hcaca;
    FD1P3DX shift_register_i1 (.D(shift_register_95__N_2811[1]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3DX shift_register_i2 (.D(shift_register_95__N_2811[2]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3DX shift_register_i3 (.D(shift_register_95__N_2811[3]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3DX shift_register_i4 (.D(shift_register_95__N_2811[4]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3DX shift_register_i5 (.D(shift_register_95__N_2811[5]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3DX shift_register_i6 (.D(shift_register_95__N_2811[6]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i7 (.D(shift_register_95__N_2811[7]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3DX shift_register_i8 (.D(shift_register_95__N_2811[8]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3DX shift_register_i9 (.D(shift_register_95__N_2811[9]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3DX shift_register_i10 (.D(shift_register_95__N_2811[10]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3DX shift_register_i11 (.D(shift_register_95__N_2811[11]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3DX shift_register_i12 (.D(shift_register_95__N_2811[12]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3DX shift_register_i13 (.D(shift_register_95__N_2811[13]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3DX shift_register_i14 (.D(shift_register_95__N_2811[14]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3DX shift_register_i15 (.D(shift_register_95__N_2811[15]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3DX shift_register_i16 (.D(shift_register_95__N_2811[16]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[16])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3DX shift_register_i17 (.D(shift_register_95__N_2811[17]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[17])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3DX shift_register_i18 (.D(shift_register_95__N_2811[18]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[18])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3DX shift_register_i25 (.D(shift_register_95__N_2811[25]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i25.GSR = "DISABLED";
    FD1P3DX shift_register_i26 (.D(shift_register_95__N_2811[26]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i26.GSR = "DISABLED";
    FD1P3DX shift_register_i27 (.D(shift_register_95__N_2811[27]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i27.GSR = "DISABLED";
    FD1P3DX shift_register_i28 (.D(shift_register_95__N_2811[28]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i28.GSR = "DISABLED";
    FD1P3DX shift_register_i29 (.D(shift_register_95__N_2811[29]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i29.GSR = "DISABLED";
    FD1P3DX shift_register_i30 (.D(shift_register_95__N_2811[30]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i30.GSR = "DISABLED";
    FD1P3DX shift_register_i31 (.D(shift_register_95__N_2811[31]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i31.GSR = "DISABLED";
    FD1P3DX shift_register_i32 (.D(shift_register_95__N_2811[32]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[32])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i32.GSR = "DISABLED";
    FD1P3DX shift_register_i33 (.D(shift_register_95__N_2811[33]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[33])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i33.GSR = "DISABLED";
    FD1P3DX shift_register_i34 (.D(shift_register_95__N_2811[34]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[34])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i34.GSR = "DISABLED";
    FD1P3DX shift_register_i35 (.D(shift_register_95__N_2811[35]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[35])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i35.GSR = "DISABLED";
    FD1P3DX shift_register_i36 (.D(shift_register_95__N_2811[36]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[36])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i36.GSR = "DISABLED";
    FD1P3DX shift_register_i37 (.D(shift_register_95__N_2811[37]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[37])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i37.GSR = "DISABLED";
    FD1P3DX shift_register_i38 (.D(shift_register_95__N_2811[38]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[38])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i38.GSR = "DISABLED";
    FD1P3DX shift_register_i39 (.D(shift_register_95__N_2811[39]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[39])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i39.GSR = "DISABLED";
    FD1P3DX shift_register_i40 (.D(shift_register_95__N_2811[40]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[40])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i40.GSR = "DISABLED";
    FD1P3DX shift_register_i42 (.D(shift_register_95__N_2811[42]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[42])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i42.GSR = "DISABLED";
    FD1P3DX shift_register_i49 (.D(shift_register_95__N_2811[49]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[49])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i49.GSR = "DISABLED";
    FD1P3DX shift_register_i50 (.D(shift_register_95__N_2811[50]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[50])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i50.GSR = "DISABLED";
    FD1P3DX shift_register_i51 (.D(shift_register_95__N_2811[51]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[51])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i51.GSR = "DISABLED";
    FD1P3DX shift_register_i52 (.D(shift_register_95__N_2811[52]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[52])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i52.GSR = "DISABLED";
    FD1P3DX shift_register_i53 (.D(shift_register_95__N_2811[53]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[53])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i53.GSR = "DISABLED";
    FD1P3DX shift_register_i54 (.D(shift_register_95__N_2811[54]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[54])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i54.GSR = "DISABLED";
    FD1P3DX shift_register_i55 (.D(shift_register_95__N_2811[55]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[55])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i55.GSR = "DISABLED";
    FD1P3DX shift_register_i56 (.D(shift_register_95__N_2811[56]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[56])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i56.GSR = "DISABLED";
    FD1P3DX shift_register_i57 (.D(shift_register_95__N_2811[57]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[57])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i57.GSR = "DISABLED";
    FD1P3DX shift_register_i58 (.D(shift_register_95__N_2811[58]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[58])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i58.GSR = "DISABLED";
    FD1P3DX shift_register_i59 (.D(shift_register_95__N_2811[59]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[59])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i59.GSR = "DISABLED";
    FD1P3DX shift_register_i60 (.D(shift_register_95__N_2811[60]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[60])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i60.GSR = "DISABLED";
    FD1P3DX shift_register_i61 (.D(shift_register_95__N_2811[61]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[61])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i61.GSR = "DISABLED";
    FD1P3DX shift_register_i62 (.D(shift_register_95__N_2811[62]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[62])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i62.GSR = "DISABLED";
    FD1P3DX shift_register_i63 (.D(shift_register_95__N_2811[63]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[63])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i63.GSR = "DISABLED";
    FD1P3DX shift_register_i64 (.D(shift_register_95__N_2811[64]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[64])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i64.GSR = "DISABLED";
    FD1P3DX shift_register_i65 (.D(shift_register_95__N_2811[65]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[65])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i65.GSR = "DISABLED";
    FD1P3DX shift_register_i73 (.D(shift_register_95__N_2811[73]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[73])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i73.GSR = "DISABLED";
    FD1P3DX shift_register_i74 (.D(shift_register_95__N_2811[74]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[74])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i74.GSR = "DISABLED";
    FD1P3DX shift_register_i75 (.D(shift_register_95__N_2811[75]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[75])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i75.GSR = "DISABLED";
    FD1P3DX shift_register_i76 (.D(shift_register_95__N_2811[76]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[76])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i76.GSR = "DISABLED";
    FD1P3DX shift_register_i77 (.D(shift_register_95__N_2811[77]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[77])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i77.GSR = "DISABLED";
    FD1P3DX shift_register_i78 (.D(shift_register_95__N_2811[78]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[78])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i78.GSR = "DISABLED";
    FD1P3DX shift_register_i79 (.D(shift_register_95__N_2811[79]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[79])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i79.GSR = "DISABLED";
    FD1P3DX shift_register_i80 (.D(shift_register_95__N_2811[80]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[80])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i80.GSR = "DISABLED";
    FD1P3DX shift_register_i81 (.D(shift_register_95__N_2811[81]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[81])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i81.GSR = "DISABLED";
    FD1P3DX shift_register_i82 (.D(shift_register_95__N_2811[82]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[82])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i82.GSR = "DISABLED";
    FD1P3DX shift_register_i83 (.D(shift_register_95__N_2811[83]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[83])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i83.GSR = "DISABLED";
    FD1P3DX shift_register_i84 (.D(shift_register_95__N_2811[84]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[84])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i84.GSR = "DISABLED";
    FD1P3DX shift_register_i85 (.D(shift_register_95__N_2811[85]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[85])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i85.GSR = "DISABLED";
    FD1P3DX shift_register_i86 (.D(shift_register_95__N_2811[86]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[86])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i86.GSR = "DISABLED";
    FD1P3DX shift_register_i87 (.D(shift_register_95__N_2811[87]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[87])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i87.GSR = "DISABLED";
    FD1P3DX shift_register_i88 (.D(shift_register_95__N_2811[88]), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[88])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i88.GSR = "DISABLED";
    LUT4 i10538_2_lut (.A(shift_register[43]), .B(n13), .Z(n15933)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10538_2_lut.init = 16'h8888;
    LUT4 i10556_2_lut (.A(shift_register[94]), .B(n13), .Z(n15969)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10556_2_lut.init = 16'h8888;
    LUT4 i10555_2_lut (.A(shift_register[93]), .B(n13), .Z(n15967)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10555_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i44 (.D(n15933), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[44])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i44.GSR = "DISABLED";
    FD1P3DX shift_register_i95 (.D(n15969), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[95])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i95.GSR = "DISABLED";
    LUT4 i10526_3_lut_4_lut (.A(bit_count[2]), .B(n23563), .C(n13), .D(bit_count[3]), 
         .Z(n15909)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10526_3_lut_4_lut.init = 16'h7080;
    LUT4 i10546_2_lut (.A(shift_register[68]), .B(n13), .Z(n15949)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10546_2_lut.init = 16'h8888;
    LUT4 i1604_2_lut_rep_86_3_lut_4_lut (.A(bit_count[2]), .B(n23563), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n23496)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1604_2_lut_rep_86_3_lut_4_lut.init = 16'h8000;
    FD1P3DX shift_register_i94 (.D(n15967), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[94])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i94.GSR = "DISABLED";
    LUT4 i10554_2_lut (.A(shift_register[92]), .B(n13), .Z(n15965)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10554_2_lut.init = 16'h8888;
    LUT4 i10545_2_lut (.A(shift_register[67]), .B(n13), .Z(n15947)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10545_2_lut.init = 16'h8888;
    LUT4 i10553_2_lut (.A(shift_register[91]), .B(n13), .Z(n15963)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10553_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i69 (.D(n15949), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[69])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i69.GSR = "DISABLED";
    LUT4 i14688_3_lut_4_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .D(bit_count[0]), .Z(bit_count_6__N_2909[0])) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D))) */ ;
    defparam i14688_3_lut_4_lut.init = 16'h8f7f;
    LUT4 i10544_2_lut (.A(shift_register[66]), .B(n13), .Z(n15945)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10544_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i93 (.D(n15965), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[93])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i93.GSR = "DISABLED";
    LUT4 i14612_2_lut_2_lut_3_lut (.A(bit_count[5]), .B(bit_count[6]), .C(n13), 
         .Z(sck_N_2908_enable_101)) /* synthesis lut_function=(!(A (B (C)))) */ ;
    defparam i14612_2_lut_2_lut_3_lut.init = 16'h7f7f;
    LUT4 i10543_2_lut (.A(shift_register[65]), .B(n13), .Z(n15943)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10543_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i68 (.D(n15947), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[68])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i68.GSR = "DISABLED";
    LUT4 i10542_2_lut (.A(shift_register[47]), .B(n13), .Z(n15941)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10542_2_lut.init = 16'h8888;
    LUT4 i10552_2_lut (.A(shift_register[90]), .B(n13), .Z(n15961)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10552_2_lut.init = 16'h8888;
    LUT4 i10551_2_lut (.A(shift_register[89]), .B(n13), .Z(n15959)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10551_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i92 (.D(n15963), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[92])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i92.GSR = "DISABLED";
    LUT4 i10537_2_lut (.A(shift_register[42]), .B(n13), .Z(n15931)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10537_2_lut.init = 16'h8888;
    LUT4 i10536_2_lut (.A(shift_register[40]), .B(n13), .Z(n15929)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10536_2_lut.init = 16'h8888;
    LUT4 i10541_2_lut (.A(shift_register[46]), .B(n13), .Z(n15939)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10541_2_lut.init = 16'h8888;
    LUT4 i10535_2_lut (.A(shift_register[23]), .B(n13), .Z(n15927)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10535_2_lut.init = 16'h8888;
    LUT4 i10534_2_lut (.A(shift_register[22]), .B(n13), .Z(n15925)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10534_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i67 (.D(n15945), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[67])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i67.GSR = "DISABLED";
    FD1P3DX shift_register_i66 (.D(n15943), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[66])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i66.GSR = "DISABLED";
    FD1P3DX shift_register_i48 (.D(n15941), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[48])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i48.GSR = "DISABLED";
    FD1P3DX shift_register_i91 (.D(n15961), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[91])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i91.GSR = "DISABLED";
    LUT4 i10540_2_lut (.A(shift_register[45]), .B(n13), .Z(n15937)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10540_2_lut.init = 16'h8888;
    LUT4 i10533_2_lut (.A(shift_register[21]), .B(n13), .Z(n15923)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10533_2_lut.init = 16'h8888;
    LUT4 i10547_2_lut (.A(shift_register[69]), .B(n13), .Z(n15951)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10547_2_lut.init = 16'h8888;
    LUT4 i6_4_lut (.A(bit_count[2]), .B(n12), .C(bit_count[6]), .D(bit_count[1]), 
         .Z(n13)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i6_4_lut.init = 16'hfffe;
    FD1P3DX shift_register_i90 (.D(n15959), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[90])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i90.GSR = "DISABLED";
    LUT4 i10532_2_lut (.A(shift_register[20]), .B(n13), .Z(n15921)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10532_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i43 (.D(n15931), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[43])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i43.GSR = "DISABLED";
    LUT4 i10531_2_lut (.A(shift_register[19]), .B(n13), .Z(n15919)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10531_2_lut.init = 16'h8888;
    LUT4 i5_4_lut (.A(bit_count[0]), .B(bit_count[5]), .C(bit_count[4]), 
         .D(bit_count[3]), .Z(n12)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(26[16:33])
    defparam i5_4_lut.init = 16'hfffe;
    FD1P3DX shift_register_i41 (.D(n15929), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[41])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i41.GSR = "DISABLED";
    FD1P3DX shift_register_i47 (.D(n15939), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[47])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i47.GSR = "DISABLED";
    FD1P3DX shift_register_i24 (.D(n15927), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i24.GSR = "DISABLED";
    FD1P3DX shift_register_i23 (.D(n15925), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[23])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i23.GSR = "DISABLED";
    LUT4 i10539_2_lut (.A(shift_register[44]), .B(n13), .Z(n15935)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10539_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i46 (.D(n15937), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[46])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i46.GSR = "DISABLED";
    FD1P3DX shift_register_i22 (.D(n15923), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[22])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3DX shift_register_i21 (.D(n15921), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[21])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3DX shift_register_i20 (.D(n15919), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[20])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3DX shift_register_i19 (.D(n15917), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[19])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i19.GSR = "DISABLED";
    FD1P3DX bit_count_i6 (.D(n15915), .SP(sck_N_2908_enable_101), .CK(sck_N_2908), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam bit_count_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i89 (.D(n15957), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[89])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i89.GSR = "DISABLED";
    FD1P3DX shift_register_i72 (.D(n15955), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[72])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i72.GSR = "DISABLED";
    LUT4 i10530_2_lut (.A(shift_register[18]), .B(n13), .Z(n15917)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10530_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i71 (.D(n15953), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[71])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i71.GSR = "DISABLED";
    LUT4 i1583_2_lut_rep_153 (.A(bit_count[1]), .B(bit_count[0]), .Z(n23563)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1583_2_lut_rep_153.init = 16'h8888;
    LUT4 i1590_2_lut_rep_117_3_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[2]), .Z(n23527)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1590_2_lut_rep_117_3_lut.init = 16'h8080;
    LUT4 i10525_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), .C(n13), 
         .D(bit_count[2]), .Z(n15907)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10525_3_lut_4_lut.init = 16'h7080;
    LUT4 i1597_2_lut_rep_102_3_lut_4_lut (.A(bit_count[1]), .B(bit_count[0]), 
         .C(bit_count[3]), .D(bit_count[2]), .Z(n23512)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i1597_2_lut_rep_102_3_lut_4_lut.init = 16'h8000;
    LUT4 i10529_4_lut (.A(bit_count[6]), .B(n13), .C(bit_count[5]), .D(n23496), 
         .Z(n15915)) /* synthesis lut_function=(!(A ((C (D))+!B)+!A !(B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10529_4_lut.init = 16'h4888;
    LUT4 i10528_3_lut_4_lut (.A(bit_count[4]), .B(n23512), .C(n13), .D(bit_count[5]), 
         .Z(n15913)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(37[22:38])
    defparam i10528_3_lut_4_lut.init = 16'h7080;
    LUT4 i10550_2_lut (.A(shift_register[88]), .B(n13), .Z(n15957)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10550_2_lut.init = 16'h8888;
    LUT4 i10549_2_lut (.A(shift_register[71]), .B(n13), .Z(n15955)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10549_2_lut.init = 16'h8888;
    LUT4 i10548_2_lut (.A(shift_register[70]), .B(n13), .Z(n15953)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam i10548_2_lut.init = 16'h8888;
    FD1P3DX shift_register_i45 (.D(n15935), .SP(sck_N_2908_enable_101), 
            .CK(sck_N_2908), .CD(spi_mic_cs_n_c), .Q(shift_register[45])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=528, LSE_RLINE=531 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(32[14] 38[8])
    defparam shift_register_i45.GSR = "DISABLED";
    
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

module ws2812_stream (\rgb_hold[93] , \rgb_hold[94] , \rgb_hold[95] , 
            \rgb_hold[80] , \rgb_hold[81] , \rgb_hold[82] , \rgb_hold[83] , 
            \rgb_hold[84] , \rgb_hold[73] , \rgb_hold[74] , \rgb_hold[75] , 
            \rgb_hold[85] , \rgb_hold[76] , \rgb_hold[86] , \rgb_hold[87] , 
            \rgb_hold[77] , \rgb_hold[78] , \rgb_hold[79] , \rgb_hold[88] , 
            \rgb_hold[89] , \rgb_hold[90] , \rgb_hold[91] , \rgb_hold[92] , 
            GND_net, rgb_data_c, pll_clk, \rgb_hold[72] ) /* synthesis syn_module_defined=1 */ ;
    input \rgb_hold[93] ;
    input \rgb_hold[94] ;
    input \rgb_hold[95] ;
    input \rgb_hold[80] ;
    input \rgb_hold[81] ;
    input \rgb_hold[82] ;
    input \rgb_hold[83] ;
    input \rgb_hold[84] ;
    input \rgb_hold[73] ;
    input \rgb_hold[74] ;
    input \rgb_hold[75] ;
    input \rgb_hold[85] ;
    input \rgb_hold[76] ;
    input \rgb_hold[86] ;
    input \rgb_hold[87] ;
    input \rgb_hold[77] ;
    input \rgb_hold[78] ;
    input \rgb_hold[79] ;
    input \rgb_hold[88] ;
    input \rgb_hold[89] ;
    input \rgb_hold[90] ;
    input \rgb_hold[91] ;
    input \rgb_hold[92] ;
    input GND_net;
    output rgb_data_c;
    input pll_clk;
    input \rgb_hold[72] ;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[10:17])
    wire [23:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(15[16:30])
    wire [1:0]state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(11[15:20])
    wire [23:0]shift_register_23__N_2707;
    wire [7:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(13[15:29])
    
    wire n16901, n22906, n15;
    wire [4:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(14[15:25])
    
    wire n23508;
    wire [4:0]bit_number_4__N_2793;
    
    wire pll_clk_enable_569, data_out_N_2808, pll_clk_enable_568, n23492, 
        n15889, n19;
    wire [12:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(12[16:27])
    
    wire n18, n12, n22631, n22630;
    wire [7:0]n69;
    wire [7:0]bit_cell_count_7__N_2694;
    
    wire n23509, n22638, n23507, pll_clk_enable_476, n23014, n23559, 
        n23495, n23558, n22760, n22930, n22143, n6, pll_clk_enable_570;
    wire [1:0]state_1__N_2679;
    
    wire n22010;
    wire [12:0]reset_count_12__N_2780;
    
    wire n22011, pll_clk_enable_23, data_out_N_2798, n22009, n22632, 
        n19384, n22019, n22018, n22017, n22016, n22014, n22908, 
        n22688, n7, n8, n22689, n22013, n31, n22012, n33, n22904;
    
    LUT4 state_1__I_0_49_Mux_13_i3_3_lut (.A(\rgb_hold[93] ), .B(shift_register[12]), 
         .C(state[1]), .Z(shift_register_23__N_2707[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_13_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_14_i3_3_lut (.A(\rgb_hold[94] ), .B(shift_register[13]), 
         .C(state[1]), .Z(shift_register_23__N_2707[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_14_i3_3_lut.init = 16'hcaca;
    LUT4 i3_4_lut (.A(bit_cell_count[6]), .B(bit_cell_count[2]), .C(n16901), 
         .D(n22906), .Z(n15)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;
    defparam i3_4_lut.init = 16'hfbff;
    LUT4 i1552_2_lut_3_lut_4_lut (.A(bit_number[0]), .B(n23508), .C(bit_number[2]), 
         .D(bit_number[1]), .Z(bit_number_4__N_2793[2])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(41[30] 44[24])
    defparam i1552_2_lut_3_lut_4_lut.init = 16'hd2f0;
    LUT4 i14385_2_lut (.A(bit_cell_count[3]), .B(bit_cell_count[0]), .Z(n22906)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14385_2_lut.init = 16'h8888;
    LUT4 state_1__I_0_49_Mux_15_i3_3_lut (.A(\rgb_hold[95] ), .B(shift_register[14]), 
         .C(state[1]), .Z(shift_register_23__N_2707[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_15_i3_3_lut.init = 16'hcaca;
    LUT4 i3_4_lut_adj_23 (.A(bit_cell_count[5]), .B(bit_cell_count[4]), 
         .C(bit_cell_count[7]), .D(bit_cell_count[1]), .Z(n16901)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i3_4_lut_adj_23.init = 16'hbfff;
    LUT4 state_0__bdd_3_lut (.A(state[0]), .B(n15), .C(state[1]), .Z(pll_clk_enable_569)) /* synthesis lut_function=(!(A (C)+!A (B+!(C)))) */ ;
    defparam state_0__bdd_3_lut.init = 16'h1a1a;
    LUT4 i14701_3_lut_4_lut (.A(n15), .B(data_out_N_2808), .C(state[1]), 
         .D(state[0]), .Z(pll_clk_enable_568)) /* synthesis lut_function=(!(A (C+(D))+!A (B (C+(D))+!B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(37[25:44])
    defparam i14701_3_lut_4_lut.init = 16'h001f;
    LUT4 state_1__I_0_49_Mux_16_i3_3_lut (.A(\rgb_hold[80] ), .B(shift_register[15]), 
         .C(state[1]), .Z(shift_register_23__N_2707[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_16_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_17_i3_3_lut (.A(\rgb_hold[81] ), .B(shift_register[16]), 
         .C(state[1]), .Z(shift_register_23__N_2707[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_17_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_18_i3_3_lut (.A(\rgb_hold[82] ), .B(shift_register[17]), 
         .C(state[1]), .Z(shift_register_23__N_2707[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_18_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_19_i3_3_lut (.A(\rgb_hold[83] ), .B(shift_register[18]), 
         .C(state[1]), .Z(shift_register_23__N_2707[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_19_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_20_i3_3_lut (.A(\rgb_hold[84] ), .B(shift_register[19]), 
         .C(state[1]), .Z(shift_register_23__N_2707[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_20_i3_3_lut.init = 16'hcaca;
    LUT4 i1566_3_lut_4_lut (.A(bit_number[2]), .B(n23492), .C(bit_number[3]), 
         .D(bit_number[4]), .Z(bit_number_4__N_2793[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(41[30] 44[24])
    defparam i1566_3_lut_4_lut.init = 16'h7f80;
    LUT4 i1_2_lut_3_lut_4_lut (.A(n15), .B(data_out_N_2808), .C(state[1]), 
         .D(state[0]), .Z(n15889)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(37[25:44])
    defparam i1_2_lut_3_lut_4_lut.init = 16'h0010;
    LUT4 state_1__I_0_49_Mux_1_i3_3_lut (.A(\rgb_hold[73] ), .B(shift_register[0]), 
         .C(state[1]), .Z(shift_register_23__N_2707[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_1_i3_3_lut.init = 16'hcaca;
    LUT4 i10_4_lut (.A(n19), .B(reset_count[1]), .C(n18), .D(n12), .Z(n22631)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i10_4_lut.init = 16'h8000;
    LUT4 i7_4_lut (.A(n22630), .B(state[1]), .C(reset_count[9]), .D(reset_count[10]), 
         .Z(n18)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam i7_4_lut.init = 16'h0200;
    LUT4 state_1__I_0_49_Mux_2_i3_3_lut (.A(\rgb_hold[74] ), .B(shift_register[1]), 
         .C(state[1]), .Z(shift_register_23__N_2707[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_2_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut (.A(state[1]), .B(n15), .C(n69[4]), .Z(bit_cell_count_7__N_2694[4])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i1_2_lut_3_lut.init = 16'h8080;
    LUT4 state_1__I_0_49_Mux_3_i3_3_lut (.A(\rgb_hold[75] ), .B(shift_register[2]), 
         .C(state[1]), .Z(shift_register_23__N_2707[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_3_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_21_i3_4_lut (.A(\rgb_hold[85] ), .B(shift_register[20]), 
         .C(state[1]), .D(n23509), .Z(shift_register_23__N_2707[21])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_21_i3_4_lut.init = 16'h0aca;
    LUT4 i1_2_lut (.A(state[0]), .B(reset_count[8]), .Z(n12)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i1_2_lut.init = 16'h4444;
    LUT4 state_1__I_0_49_Mux_4_i3_3_lut (.A(\rgb_hold[76] ), .B(shift_register[3]), 
         .C(state[1]), .Z(shift_register_23__N_2707[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_4_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_22_i3_4_lut (.A(\rgb_hold[86] ), .B(shift_register[21]), 
         .C(state[1]), .D(n23509), .Z(shift_register_23__N_2707[22])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_22_i3_4_lut.init = 16'h0aca;
    LUT4 i600_2_lut_rep_98 (.A(data_out_N_2808), .B(n15), .Z(n23508)) /* synthesis lut_function=((B)+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[26] 51[20])
    defparam i600_2_lut_rep_98.init = 16'hdddd;
    LUT4 i1547_2_lut_rep_82_3_lut_4_lut (.A(data_out_N_2808), .B(n15), .C(bit_number[1]), 
         .D(bit_number[0]), .Z(n23492)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[26] 51[20])
    defparam i1547_2_lut_rep_82_3_lut_4_lut.init = 16'h2000;
    LUT4 state_1__I_0_49_Mux_23_i3_4_lut (.A(\rgb_hold[87] ), .B(shift_register[22]), 
         .C(state[1]), .D(n23509), .Z(shift_register_23__N_2707[23])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_23_i3_4_lut.init = 16'h0aca;
    LUT4 i7104_4_lut (.A(state[0]), .B(n22638), .C(n23507), .D(state[1]), 
         .Z(pll_clk_enable_476)) /* synthesis lut_function=(A+!(B (C+!(D))+!B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i7104_4_lut.init = 16'hafbb;
    LUT4 i7_4_lut_adj_24 (.A(reset_count[9]), .B(n23014), .C(n23559), 
         .D(reset_count[10]), .Z(n22638)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i7_4_lut_adj_24.init = 16'hbfff;
    LUT4 i1_2_lut_3_lut_adj_25 (.A(data_out_N_2808), .B(n15), .C(bit_number[0]), 
         .Z(bit_number_4__N_2793[0])) /* synthesis lut_function=(A (B (C)+!B !(C))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[26] 51[20])
    defparam i1_2_lut_3_lut_adj_25.init = 16'hd2d2;
    LUT4 i1539_2_lut_rep_85_3_lut (.A(data_out_N_2808), .B(n15), .C(bit_number[0]), 
         .Z(n23495)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[26] 51[20])
    defparam i1539_2_lut_rep_85_3_lut.init = 16'h2020;
    LUT4 i1545_2_lut_3_lut_4_lut (.A(data_out_N_2808), .B(n15), .C(bit_number[1]), 
         .D(bit_number[0]), .Z(bit_number_4__N_2793[1])) /* synthesis lut_function=(A (B (C)+!B !(C (D)+!C !(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(45[26] 51[20])
    defparam i1545_2_lut_3_lut_4_lut.init = 16'hd2f0;
    LUT4 i5151_2_lut_rep_99 (.A(n15), .B(state[0]), .Z(n23509)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam i5151_2_lut_rep_99.init = 16'heeee;
    LUT4 i14493_4_lut (.A(n23558), .B(reset_count[1]), .C(n22760), .D(n22930), 
         .Z(n23014)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14493_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_adj_26 (.A(n15), .B(state[0]), .C(state[1]), .Z(n22143)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam i1_2_lut_3_lut_adj_26.init = 16'h1010;
    LUT4 i14239_2_lut (.A(reset_count[4]), .B(reset_count[11]), .Z(n22760)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14239_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_adj_27 (.A(state[1]), .B(n15), .C(n69[5]), .Z(bit_cell_count_7__N_2694[5])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i1_2_lut_3_lut_adj_27.init = 16'h8080;
    LUT4 i14409_4_lut (.A(reset_count[0]), .B(reset_count[7]), .C(reset_count[8]), 
         .D(reset_count[6]), .Z(n22930)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14409_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_3_lut_adj_28 (.A(state[1]), .B(n15), .C(n69[6]), .Z(bit_cell_count_7__N_2694[6])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i1_2_lut_3_lut_adj_28.init = 16'h8080;
    LUT4 i1_2_lut_3_lut_adj_29 (.A(state[1]), .B(n15), .C(n69[7]), .Z(bit_cell_count_7__N_2694[7])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i1_2_lut_3_lut_adj_29.init = 16'h8080;
    LUT4 state_1__I_0_49_Mux_5_i3_3_lut (.A(\rgb_hold[77] ), .B(shift_register[4]), 
         .C(state[1]), .Z(shift_register_23__N_2707[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_5_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_adj_30 (.A(state[1]), .B(n15), .C(n69[3]), .Z(bit_cell_count_7__N_2694[3])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i1_2_lut_3_lut_adj_30.init = 16'h8080;
    LUT4 state_1__I_0_49_Mux_6_i3_3_lut (.A(\rgb_hold[78] ), .B(shift_register[5]), 
         .C(state[1]), .Z(shift_register_23__N_2707[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_6_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_adj_31 (.A(state[1]), .B(n15), .C(n69[2]), .Z(bit_cell_count_7__N_2694[2])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i1_2_lut_3_lut_adj_31.init = 16'h8080;
    LUT4 i4_4_lut (.A(reset_count[6]), .B(reset_count[11]), .C(reset_count[4]), 
         .D(n6), .Z(n22630)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut.init = 16'h8000;
    LUT4 i10515_2_lut_3_lut (.A(state[1]), .B(n15), .C(n69[1]), .Z(bit_cell_count_7__N_2694[1])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i10515_2_lut_3_lut.init = 16'h8080;
    LUT4 state_1__I_0_49_Mux_7_i3_3_lut (.A(\rgb_hold[79] ), .B(shift_register[6]), 
         .C(state[1]), .Z(shift_register_23__N_2707[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_7_i3_3_lut.init = 16'hcaca;
    LUT4 i10497_2_lut_3_lut (.A(state[1]), .B(n15), .C(n69[0]), .Z(bit_cell_count_7__N_2694[0])) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i10497_2_lut_3_lut.init = 16'h8080;
    LUT4 state_1__I_0_49_Mux_8_i3_3_lut (.A(\rgb_hold[88] ), .B(shift_register[7]), 
         .C(state[1]), .Z(shift_register_23__N_2707[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_8_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_9_i3_3_lut (.A(\rgb_hold[89] ), .B(shift_register[8]), 
         .C(state[1]), .Z(shift_register_23__N_2707[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_9_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_adj_32 (.A(reset_count[0]), .B(reset_count[7]), .Z(n6)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_32.init = 16'h8888;
    LUT4 state_1__I_0_49_Mux_10_i3_3_lut (.A(\rgb_hold[90] ), .B(shift_register[9]), 
         .C(state[1]), .Z(shift_register_23__N_2707[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_10_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_97 (.A(n15), .B(data_out_N_2808), .Z(n23507)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(37[25:44])
    defparam i1_2_lut_rep_97.init = 16'heeee;
    LUT4 i15_2_lut (.A(state[1]), .B(state[0]), .Z(pll_clk_enable_570)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i15_2_lut.init = 16'h6666;
    LUT4 i7286_2_lut (.A(state[0]), .B(state[1]), .Z(state_1__N_2679[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i7286_2_lut.init = 16'h2222;
    LUT4 state_1__I_0_49_Mux_11_i3_3_lut (.A(\rgb_hold[91] ), .B(shift_register[10]), 
         .C(state[1]), .Z(shift_register_23__N_2707[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_11_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_49_Mux_12_i3_3_lut (.A(\rgb_hold[92] ), .B(shift_register[11]), 
         .C(state[1]), .Z(shift_register_23__N_2707[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_49_Mux_12_i3_3_lut.init = 16'hcaca;
    CCU2D add_878_5 (.A0(reset_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22010), .COUT(n22011), .S0(reset_count_12__N_2780[3]), 
          .S1(reset_count_12__N_2780[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[22:57])
    defparam add_878_5.INIT0 = 16'h5aaa;
    defparam add_878_5.INIT1 = 16'h5aaa;
    defparam add_878_5.INJECT1_0 = "NO";
    defparam add_878_5.INJECT1_1 = "NO";
    FD1P3AX data_out_reg_45 (.D(data_out_N_2798), .SP(pll_clk_enable_23), 
            .CK(pll_clk), .Q(rgb_data_c)) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam data_out_reg_45.GSR = "DISABLED";
    FD1S3AX state_i0 (.D(n22631), .CK(pll_clk), .Q(state[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam state_i0.GSR = "DISABLED";
    FD1P3IX bit_number_i4 (.D(bit_number_4__N_2793[4]), .SP(pll_clk_enable_570), 
            .CD(state_1__N_2679[1]), .CK(pll_clk), .Q(bit_number[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_number_i4.GSR = "DISABLED";
    FD1P3IX bit_number_i3 (.D(bit_number_4__N_2793[3]), .SP(pll_clk_enable_570), 
            .CD(state_1__N_2679[1]), .CK(pll_clk), .Q(bit_number[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_number_i3.GSR = "DISABLED";
    FD1P3IX bit_number_i2 (.D(bit_number_4__N_2793[2]), .SP(pll_clk_enable_570), 
            .CD(state_1__N_2679[1]), .CK(pll_clk), .Q(bit_number[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_number_i2.GSR = "DISABLED";
    FD1P3IX bit_number_i1 (.D(bit_number_4__N_2793[1]), .SP(pll_clk_enable_570), 
            .CD(state_1__N_2679[1]), .CK(pll_clk), .Q(bit_number[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_number_i1.GSR = "DISABLED";
    FD1P3IX reset_count_i12 (.D(reset_count_12__N_2780[12]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i12.GSR = "DISABLED";
    FD1P3IX reset_count_i11 (.D(reset_count_12__N_2780[11]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i11.GSR = "DISABLED";
    FD1P3IX reset_count_i10 (.D(reset_count_12__N_2780[10]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i10.GSR = "DISABLED";
    FD1P3IX reset_count_i9 (.D(reset_count_12__N_2780[9]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i9.GSR = "DISABLED";
    FD1P3IX reset_count_i8 (.D(reset_count_12__N_2780[8]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i8.GSR = "DISABLED";
    FD1P3IX reset_count_i7 (.D(reset_count_12__N_2780[7]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i7.GSR = "DISABLED";
    FD1P3IX reset_count_i6 (.D(reset_count_12__N_2780[6]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i6.GSR = "DISABLED";
    FD1P3IX reset_count_i5 (.D(reset_count_12__N_2780[5]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i5.GSR = "DISABLED";
    FD1P3IX reset_count_i4 (.D(reset_count_12__N_2780[4]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i4.GSR = "DISABLED";
    FD1P3IX reset_count_i3 (.D(reset_count_12__N_2780[3]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i3.GSR = "DISABLED";
    FD1P3IX reset_count_i2 (.D(reset_count_12__N_2780[2]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i2.GSR = "DISABLED";
    FD1P3IX reset_count_i1 (.D(reset_count_12__N_2780[1]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i1.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i7 (.D(bit_cell_count_7__N_2694[7]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i7.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i6 (.D(bit_cell_count_7__N_2694[6]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i6.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i5 (.D(bit_cell_count_7__N_2694[5]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i5.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i4 (.D(bit_cell_count_7__N_2694[4]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i4.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i3 (.D(bit_cell_count_7__N_2694[3]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i3.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i2 (.D(bit_cell_count_7__N_2694[2]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i2.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i1 (.D(bit_cell_count_7__N_2694[1]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i1.GSR = "DISABLED";
    CCU2D add_878_3 (.A0(reset_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22009), .COUT(n22010), .S0(reset_count_12__N_2780[1]), 
          .S1(reset_count_12__N_2780[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[22:57])
    defparam add_878_3.INIT0 = 16'h5aaa;
    defparam add_878_3.INIT1 = 16'h5aaa;
    defparam add_878_3.INJECT1_0 = "NO";
    defparam add_878_3.INJECT1_1 = "NO";
    LUT4 i1559_2_lut_3_lut_4_lut (.A(bit_number[1]), .B(n23495), .C(bit_number[3]), 
         .D(bit_number[2]), .Z(bit_number_4__N_2793[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(41[30] 44[24])
    defparam i1559_2_lut_3_lut_4_lut.init = 16'h78f0;
    CCU2D add_878_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[0]), .B1(n22632), .C1(n19384), .D1(reset_count[9]), 
          .COUT(n22009), .S1(reset_count_12__N_2780[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[22:57])
    defparam add_878_1.INIT0 = 16'hF000;
    defparam add_878_1.INIT1 = 16'h5595;
    defparam add_878_1.INJECT1_0 = "NO";
    defparam add_878_1.INJECT1_1 = "NO";
    CCU2D add_14_9 (.A0(bit_cell_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n22019), .S0(n69[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[39:60])
    defparam add_14_9.INIT0 = 16'h5aaa;
    defparam add_14_9.INIT1 = 16'h0000;
    defparam add_14_9.INJECT1_0 = "NO";
    defparam add_14_9.INJECT1_1 = "NO";
    CCU2D add_14_7 (.A0(bit_cell_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22018), .COUT(n22019), .S0(n69[5]), .S1(n69[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[39:60])
    defparam add_14_7.INIT0 = 16'h5aaa;
    defparam add_14_7.INIT1 = 16'h5aaa;
    defparam add_14_7.INJECT1_0 = "NO";
    defparam add_14_7.INJECT1_1 = "NO";
    CCU2D add_14_5 (.A0(bit_cell_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22017), .COUT(n22018), .S0(n69[3]), .S1(n69[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[39:60])
    defparam add_14_5.INIT0 = 16'h5aaa;
    defparam add_14_5.INIT1 = 16'h5aaa;
    defparam add_14_5.INJECT1_0 = "NO";
    defparam add_14_5.INJECT1_1 = "NO";
    CCU2D add_14_3 (.A0(bit_cell_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22016), .COUT(n22017), .S0(n69[1]), .S1(n69[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[39:60])
    defparam add_14_3.INIT0 = 16'h5aaa;
    defparam add_14_3.INIT1 = 16'h5aaa;
    defparam add_14_3.INJECT1_0 = "NO";
    defparam add_14_3.INJECT1_1 = "NO";
    CCU2D add_14_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n22016), .S1(n69[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[39:60])
    defparam add_14_1.INIT0 = 16'hF000;
    defparam add_14_1.INIT1 = 16'h5555;
    defparam add_14_1.INJECT1_0 = "NO";
    defparam add_14_1.INJECT1_1 = "NO";
    CCU2D add_878_13 (.A0(reset_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22014), .S0(reset_count_12__N_2780[11]), 
          .S1(reset_count_12__N_2780[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[22:57])
    defparam add_878_13.INIT0 = 16'h5aaa;
    defparam add_878_13.INIT1 = 16'h5aaa;
    defparam add_878_13.INJECT1_0 = "NO";
    defparam add_878_13.INJECT1_1 = "NO";
    LUT4 i3_4_lut_adj_33 (.A(bit_cell_count[5]), .B(shift_register[23]), 
         .C(bit_cell_count[4]), .D(n22908), .Z(n22688)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i3_4_lut_adj_33.init = 16'h0002;
    LUT4 i14387_2_lut (.A(bit_cell_count[1]), .B(bit_cell_count[7]), .Z(n22908)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i14387_2_lut.init = 16'heeee;
    LUT4 i1_2_lut_adj_34 (.A(bit_cell_count[4]), .B(bit_cell_count[5]), 
         .Z(n7)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_34.init = 16'h2222;
    LUT4 i2_2_lut (.A(shift_register[23]), .B(bit_cell_count[2]), .Z(n8)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i2_2_lut.init = 16'h2222;
    LUT4 i5_4_lut (.A(bit_cell_count[0]), .B(n7), .C(n22908), .D(n8), 
         .Z(n22689)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i5_4_lut.init = 16'h0800;
    CCU2D add_878_11 (.A0(reset_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22013), .COUT(n22014), .S0(reset_count_12__N_2780[9]), 
          .S1(reset_count_12__N_2780[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[22:57])
    defparam add_878_11.INIT0 = 16'h5aaa;
    defparam add_878_11.INIT1 = 16'h5aaa;
    defparam add_878_11.INJECT1_0 = "NO";
    defparam add_878_11.INJECT1_1 = "NO";
    LUT4 i1_4_lut (.A(bit_cell_count[2]), .B(n22688), .C(n16901), .D(bit_cell_count[0]), 
         .Z(n31)) /* synthesis lut_function=(!((B (C (D))+!B (C+!(D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam i1_4_lut.init = 16'h0a88;
    FD1P3AX shift_register_i1 (.D(shift_register_23__N_2707[1]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3AX shift_register_i2 (.D(shift_register_23__N_2707[2]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3AX shift_register_i3 (.D(shift_register_23__N_2707[3]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i4 (.D(shift_register_23__N_2707[4]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3AX shift_register_i5 (.D(shift_register_23__N_2707[5]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3AX shift_register_i6 (.D(shift_register_23__N_2707[6]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3AX shift_register_i7 (.D(shift_register_23__N_2707[7]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3AX shift_register_i8 (.D(shift_register_23__N_2707[8]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3AX shift_register_i9 (.D(shift_register_23__N_2707[9]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3AX shift_register_i10 (.D(shift_register_23__N_2707[10]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3AX shift_register_i11 (.D(shift_register_23__N_2707[11]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3AX shift_register_i12 (.D(shift_register_23__N_2707[12]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3AX shift_register_i13 (.D(shift_register_23__N_2707[13]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3AX shift_register_i14 (.D(shift_register_23__N_2707[14]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3AX shift_register_i15 (.D(shift_register_23__N_2707[15]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[15])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3AX shift_register_i16 (.D(shift_register_23__N_2707[16]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[16])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3AX shift_register_i17 (.D(shift_register_23__N_2707[17]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[17])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i18 (.D(shift_register_23__N_2707[18]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[18])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i19 (.D(shift_register_23__N_2707[19]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[19])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i20 (.D(shift_register_23__N_2707[20]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[20])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i21 (.D(shift_register_23__N_2707[21]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[21])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i22 (.D(shift_register_23__N_2707[22]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[22])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i23 (.D(shift_register_23__N_2707[23]), .SP(pll_clk_enable_569), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i23.GSR = "DISABLED";
    FD1P3AX state_i1 (.D(state_1__N_2679[1]), .SP(pll_clk_enable_476), .CK(pll_clk), 
            .Q(state[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam state_i1.GSR = "DISABLED";
    CCU2D add_878_9 (.A0(reset_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22012), .COUT(n22013), .S0(reset_count_12__N_2780[7]), 
          .S1(reset_count_12__N_2780[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[22:57])
    defparam add_878_9.INIT0 = 16'h5aaa;
    defparam add_878_9.INIT1 = 16'h5aaa;
    defparam add_878_9.INJECT1_0 = "NO";
    defparam add_878_9.INJECT1_1 = "NO";
    PFUMX i41 (.BLUT(n31), .ALUT(n22689), .C0(bit_cell_count[6]), .Z(n33));
    CCU2D add_878_7 (.A0(reset_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n22011), .COUT(n22012), .S0(reset_count_12__N_2780[5]), 
          .S1(reset_count_12__N_2780[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[22:57])
    defparam add_878_7.INIT0 = 16'h5aaa;
    defparam add_878_7.INIT1 = 16'h5aaa;
    defparam add_878_7.INJECT1_0 = "NO";
    defparam add_878_7.INJECT1_1 = "NO";
    FD1P3IX bit_number_i0 (.D(bit_number_4__N_2793[0]), .SP(pll_clk_enable_570), 
            .CD(state_1__N_2679[1]), .CK(pll_clk), .Q(bit_number[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_number_i0.GSR = "DISABLED";
    FD1P3IX reset_count_i0 (.D(reset_count_12__N_2780[0]), .SP(pll_clk_enable_568), 
            .CD(n15889), .CK(pll_clk), .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam reset_count_i0.GSR = "DISABLED";
    FD1P3IX shift_register_i0 (.D(\rgb_hold[72] ), .SP(pll_clk_enable_569), 
            .CD(n22143), .CK(pll_clk), .Q(shift_register[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam shift_register_i0.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i0 (.D(bit_cell_count_7__N_2694[0]), .SP(pll_clk_enable_570), 
            .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=523, LSE_RLINE=527 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[12] 55[8])
    defparam bit_cell_count_i0.GSR = "DISABLED";
    LUT4 i3_4_lut_adj_35 (.A(reset_count[10]), .B(reset_count[8]), .C(reset_count[5]), 
         .D(reset_count[12]), .Z(n19384)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut_adj_35.init = 16'h8000;
    LUT4 i14247_2_lut_rep_148 (.A(reset_count[3]), .B(reset_count[2]), .Z(n23558)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14247_2_lut_rep_148.init = 16'h8888;
    LUT4 i3_3_lut_4_lut (.A(reset_count[3]), .B(reset_count[2]), .C(n22630), 
         .D(reset_count[1]), .Z(n22632)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_3_lut_4_lut.init = 16'h8000;
    LUT4 i4_2_lut_rep_149 (.A(reset_count[5]), .B(reset_count[12]), .Z(n23559)) /* synthesis lut_function=(A (B)) */ ;
    defparam i4_2_lut_rep_149.init = 16'h8888;
    LUT4 i8_2_lut_3_lut_4_lut (.A(reset_count[5]), .B(reset_count[12]), 
         .C(reset_count[2]), .D(reset_count[3]), .Z(n19)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i8_2_lut_3_lut_4_lut.init = 16'h8000;
    LUT4 i1_4_lut_adj_36 (.A(state[1]), .B(n33), .C(state[0]), .D(bit_cell_count[3]), 
         .Z(pll_clk_enable_23)) /* synthesis lut_function=(!(A ((C+!(D))+!B))) */ ;
    defparam i1_4_lut_adj_36.init = 16'h5d55;
    LUT4 state_1__I_0_i3_4_lut (.A(state[0]), .B(data_out_N_2808), .C(state[1]), 
         .D(n15), .Z(data_out_N_2798)) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[9] 54[16])
    defparam state_1__I_0_i3_4_lut.init = 16'h0aca;
    LUT4 i4_4_lut_adj_37 (.A(bit_number[3]), .B(bit_number[0]), .C(bit_number[1]), 
         .D(n22904), .Z(data_out_N_2808)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i4_4_lut_adj_37.init = 16'hbfff;
    LUT4 i14383_2_lut (.A(bit_number[2]), .B(bit_number[4]), .Z(n22904)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14383_2_lut.init = 16'h8888;
    
endmodule
//
// Verilog Description of module umh_toggle_ram84
//

module umh_toggle_ram84 (pll_clk, ev_we, VCC_net, GND_net, \ev_wr_addr[0] , 
            event_rd_addr, \ev_wr_addr[1] , \ev_wr_addr[2] , \ev_wr_addr[3] , 
            \ev_wr_addr[4] , \ev_wr_addr[5] , \ev_wr_addr[6] , \ev_wr_addr[7] , 
            n23532, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
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
    input n23532;
    input [83:0]ev_wr_data;
    output [83:0]ev_rd_data;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[10:17])
    
    PDPW8KC mem1 (.DI0(ev_wr_data[48]), .DI1(ev_wr_data[49]), .DI2(ev_wr_data[50]), 
            .DI3(ev_wr_data[51]), .DI4(ev_wr_data[52]), .DI5(ev_wr_data[53]), 
            .DI6(ev_wr_data[54]), .DI7(ev_wr_data[55]), .DI8(ev_wr_data[56]), 
            .DI9(ev_wr_data[57]), .DI10(ev_wr_data[58]), .DI11(ev_wr_data[59]), 
            .DI12(ev_wr_data[60]), .DI13(ev_wr_data[61]), .DI14(ev_wr_data[62]), 
            .DI15(ev_wr_data[63]), .DI16(ev_wr_data[64]), .DI17(ev_wr_data[65]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23532), 
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
    PDPW8KC mem0 (.DI0(ev_wr_data[66]), .DI1(ev_wr_data[67]), .DI2(ev_wr_data[68]), 
            .DI3(ev_wr_data[69]), .DI4(ev_wr_data[70]), .DI5(ev_wr_data[71]), 
            .DI6(ev_wr_data[72]), .DI7(ev_wr_data[73]), .DI8(ev_wr_data[74]), 
            .DI9(ev_wr_data[75]), .DI10(ev_wr_data[76]), .DI11(ev_wr_data[77]), 
            .DI12(ev_wr_data[78]), .DI13(ev_wr_data[79]), .DI14(ev_wr_data[80]), 
            .DI15(ev_wr_data[81]), .DI16(ev_wr_data[82]), .DI17(ev_wr_data[83]), 
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23532), 
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
            .DI16(GND_net), .DI17(GND_net), .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), 
            .ADW2(\ev_wr_addr[2] ), .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), 
            .ADW5(\ev_wr_addr[5] ), .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), 
            .ADW8(n23532), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
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
            .ADW0(\ev_wr_addr[0] ), .ADW1(\ev_wr_addr[1] ), .ADW2(\ev_wr_addr[2] ), 
            .ADW3(\ev_wr_addr[3] ), .ADW4(\ev_wr_addr[4] ), .ADW5(\ev_wr_addr[5] ), 
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23532), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23532), 
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
    
endmodule
//
// Verilog Description of module umh_channel_ram18
//

module umh_channel_ram18 (staging_q, pll_clk, rd_data_15__N_2579, n9894, 
            spi1_sck_c, spi_channel_index, n9892, spi_write, VCC_net, 
            GND_net, staging_rd_addr_6__N_849, spi1_mosi_c_0, \spi_rx_shift[0] , 
            \spi_rx_shift[1] , \spi_rx_shift[2] , \spi_rx_shift[3] , \spi_rx_shift[4] , 
            \spi_rx_shift[5] , \spi_rx_shift[6] , spi_phase_pending, n9909, 
            n9911, n9913, n9915, n9917, n9919, n9921, n9923, n9925, 
            n9927, n9929, n9931, n9933, n9935, n9937, n9939, n9896, 
            n9898, n9900, n9902, n9904) /* synthesis syn_module_defined=1 */ ;
    output [15:0]staging_q;
    input pll_clk;
    input [15:0]rd_data_15__N_2579;
    output n9894;
    input spi1_sck_c;
    input [6:0]spi_channel_index;
    output n9892;
    input spi_write;
    input VCC_net;
    input GND_net;
    input [6:0]staging_rd_addr_6__N_849;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    input [7:0]spi_phase_pending;
    output n9909;
    output n9911;
    output n9913;
    output n9915;
    output n9917;
    output n9919;
    output n9921;
    output n9923;
    output n9925;
    output n9927;
    output n9929;
    output n9931;
    output n9933;
    output n9935;
    output n9937;
    output n9939;
    output n9896;
    output n9898;
    output n9900;
    output n9902;
    output n9904;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[10:17])
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:32])
    
    FD1S3AX rd_data_i0 (.D(rd_data_15__N_2579[0]), .CK(pll_clk), .Q(staging_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    FD1S3AX mem_1401 (.D(spi_channel_index[1]), .CK(spi1_sck_c), .Q(n9894));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1401.GSR = "DISABLED";
    FD1S3AX mem_1399 (.D(spi_channel_index[0]), .CK(spi1_sck_c), .Q(n9892));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1399.GSR = "DISABLED";
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
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(staging_rd_addr_6__N_849[0]), 
            .ADR5(staging_rd_addr_6__N_849[1]), .ADR6(staging_rd_addr_6__N_849[2]), 
            .ADR7(staging_rd_addr_6__N_849[3]), .ADR8(staging_rd_addr_6__N_849[4]), 
            .ADR9(staging_rd_addr_6__N_849[5]), .ADR10(staging_rd_addr_6__N_849[6]), 
            .ADR11(GND_net), .ADR12(GND_net), .CER(VCC_net), .OCER(VCC_net), 
            .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), .CSR2(GND_net), 
            .RST(GND_net), .DO0(n9927), .DO1(n9929), .DO2(n9931), .DO3(n9933), 
            .DO4(n9935), .DO5(n9937), .DO6(n9939), .DO9(n9909), .DO10(n9911), 
            .DO11(n9913), .DO12(n9915), .DO13(n9917), .DO14(n9919), 
            .DO15(n9921), .DO16(n9923), .DO17(n9925));
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
    FD1S3AX mem_1403 (.D(spi_channel_index[2]), .CK(spi1_sck_c), .Q(n9896));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1403.GSR = "DISABLED";
    FD1S3AX mem_1405 (.D(spi_channel_index[3]), .CK(spi1_sck_c), .Q(n9898));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1405.GSR = "DISABLED";
    FD1S3AX mem_1407 (.D(spi_channel_index[4]), .CK(spi1_sck_c), .Q(n9900));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1407.GSR = "DISABLED";
    FD1S3AX mem_1409 (.D(spi_channel_index[5]), .CK(spi1_sck_c), .Q(n9902));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1409.GSR = "DISABLED";
    FD1S3AX mem_1411 (.D(spi_channel_index[6]), .CK(spi1_sck_c), .Q(n9904));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1411.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_15__N_2579[1]), .CK(pll_clk), .Q(staging_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_15__N_2579[2]), .CK(pll_clk), .Q(staging_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_15__N_2579[3]), .CK(pll_clk), .Q(staging_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_15__N_2579[4]), .CK(pll_clk), .Q(staging_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_15__N_2579[5]), .CK(pll_clk), .Q(staging_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_15__N_2579[6]), .CK(pll_clk), .Q(staging_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_15__N_2579[7]), .CK(pll_clk), .Q(staging_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    FD1S3AX rd_data_i8 (.D(rd_data_15__N_2579[8]), .CK(pll_clk), .Q(staging_q[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i8.GSR = "DISABLED";
    FD1S3AX rd_data_i9 (.D(rd_data_15__N_2579[9]), .CK(pll_clk), .Q(staging_q[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i9.GSR = "DISABLED";
    FD1S3AX rd_data_i10 (.D(rd_data_15__N_2579[10]), .CK(pll_clk), .Q(staging_q[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i10.GSR = "DISABLED";
    FD1S3AX rd_data_i11 (.D(rd_data_15__N_2579[11]), .CK(pll_clk), .Q(staging_q[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i11.GSR = "DISABLED";
    FD1S3AX rd_data_i12 (.D(rd_data_15__N_2579[12]), .CK(pll_clk), .Q(staging_q[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i12.GSR = "DISABLED";
    FD1S3AX rd_data_i13 (.D(rd_data_15__N_2579[13]), .CK(pll_clk), .Q(staging_q[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i13.GSR = "DISABLED";
    FD1S3AX rd_data_i14 (.D(rd_data_15__N_2579[14]), .CK(pll_clk), .Q(staging_q[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i14.GSR = "DISABLED";
    FD1S3AX rd_data_i15 (.D(rd_data_15__N_2579[15]), .CK(pll_clk), .Q(staging_q[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=194, LSE_RLINE=197 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i15.GSR = "DISABLED";
    
endmodule
