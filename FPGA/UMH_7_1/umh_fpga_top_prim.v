// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Thu Sep 10 19:01:16 2026
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
    wire sck_N_2937 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
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
    
    wire n22534, n10287, spi1_sck_c_enable_29;
    wire [15:0]spi_extension_length;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(119[51:71])
    
    wire n12;
    wire [31:0]spi_frame_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(120[17:35])
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
    wire [7:0]next_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[17:27])
    wire [6:0]mic_divider;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(161[17:28])
    
    wire mic_tick;
    wire [15:0]mic_shift_0;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(164[17:28])
    wire [15:0]mic_shift_1;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(164[30:41])
    wire [4:0]mic_sample_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(165[17:33])
    wire [31:0]mic_latest;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(166[17:27])
    wire [3:0]ev_state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(180[17:25])
    wire [7:0]ev_clear_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(181[17:30])
    wire [6:0]ev_ch;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(182[17:22])
    wire [83:0]ev_bit;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(183[17:23])
    wire [83:0]init_shadow;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(183[25:36])
    wire [7:0]build_phase;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(184[17:28])
    wire [8:0]build_sum;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(185[17:26])
    wire [6:0]staging_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(187[17:32])
    
    wire frame_req, swap_pending, n12306, active_bank;
    wire [15:0]staging_q;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(192[17:26])
    wire [7:0]ev_rd_slot;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[17:27])
    
    wire swap_now, run_bank;
    wire [8:0]event_rd_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[17:30])
    
    wire ev_we;
    wire [8:0]ev_wr_addr;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(213[17:27])
    wire [83:0]ev_rd_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(216[17:27])
    wire [83:0]ev_rd_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(217[17:27])
    wire [83:0]ev_wr_data;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(218[17:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen, invalid_frame_meta;
    wire [31:0]accepted_sequence_meta;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[17:39])
    wire [31:0]accepted_sequence_sync;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(233[41:63])
    wire [31:0]pending_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[17:33])
    wire [31:0]accepted_sequence;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(234[35:52])
    wire [3:0]frame_settle;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(235[17:29])
    wire [95:0]rgb_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(236[17:25])
    
    wire cs_meta, cs_sync, cs_sync_d, n12360;
    wire [127:0]status_hold;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(242[18:29])
    
    wire n12366, n12372, n12378, n12270, n12384, cs_fall, n12282, 
        n12276;
    wire [15:0]fifo_credit_wire;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(250[17:33])
    
    wire n12288, n12324, n12318;
    wire [15:0]expected_next;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[17:30])
    
    wire frame_end, fpga_cs_n_N_2458;
    wire [8:0]ev_wr_addr_8__N_796;
    
    wire n12300;
    wire [15:0]status_flags_wire_15__N_1269;
    
    wire n12294, n12264, n12312;
    wire [15:0]status_flags_wire_15__N_1285;
    
    wire n22530;
    wire [6:0]spi1_miso_N_2421;
    
    wire n1, spi1_sck_c_enable_325, n60, n23169, n12336, n12330, 
        n12348, n12342;
    wire [15:0]expected_next_15__N_1342;
    wire [15:0]expected_next_15__N_1301;
    wire [31:0]frame_end_N_2500;
    wire [15:0]rd_data_15__N_2535;
    
    wire frame_end_N_2499, n21805, n12390, n21847, n12396, n12402, 
        n12408, n12414, n12420, n12426, n12432, n12438, n12444, 
        n12450, n12456, n12462, n12468, n12474, n12480, n12486, 
        n12492, n7, n6, n6_adj_2985, n21804, n16, n22369, n165, 
        n164, n163, n162, n161, n160, n159, n158, n157, n156, 
        n155, n154, n153, n152, n151, n150, n149, n148, n147, 
        n146, n145, n144, n143, n142, n141, n140, n139, n138, 
        n137, n136, n135, n134, n21880, n12739, n12745, n23207, 
        n23168, spi1_sck_c_enable_43, n12652;
    wire [15:0]spi_byte_count_15__N_1579;
    
    wire stop_toggle_spi_N_2443, frame_toggle_spi_N_2432, n12498, n12504, 
        n12510, n12516, n12522, n12528, n12536, invalid_frame_spi_N_2450, 
        n12542, n12548, n12554, n12560, n12566, spi1_sck_c_enable_287, 
        n21784, n21865, n21705, n22774, n12572, n12578, n12584, 
        n22772, n12590, n12599, n21821, n12620, pll_clk_enable_721, 
        n12636, n22768, n22764, n12680, n21783, n22762, n22760, 
        n22756, n6_adj_2986, n22754, n22750, n91, n10, n22746, 
        n12_adj_2987, n9, n22742, n21863, n21790, n22738, n21924, 
        n23166, n23052, n23051, n21704, n22734, n23050, spi1_sck_c_enable_272, 
        n25, n14, spi1_sck_c_enable_177, n55, spi1_sck_c_enable_90, 
        n22732, n22726, spi1_sck_c_enable_324, n91_adj_2988, n9797, 
        n12252, pll_clk_enable_492, pll_clk_enable_461, pll_clk_enable_599, 
        n22724, pll_clk_enable_124, pll_clk_enable_19, pll_clk_enable_219, 
        spi1_sck_c_enable_318, n22722, n22720, n21702, n22716, n22714, 
        pll_clk_enable_9, n23205, n22710, n21703, spi1_sck_c_enable_36, 
        n40, n39, n38, n37, n36, n35, n34, n7_adj_2989, n29, 
        n28, n27, n26, n14_adj_2990, n22813, n40_adj_2991, n39_adj_2992, 
        n38_adj_2993, n37_adj_2994, n36_adj_2995, n35_adj_2996, n34_adj_2997, 
        n23209, n63;
    wire [3:0]frame_settle_3__N_1860;
    
    wire n21782, n21846, n21803, n21820, n22708, n14_adj_2998, n22704, 
        active_bank_N_795, n9487, n12_adj_2999, n23451, n22700, n22698, 
        n12733, n21845;
    wire [7:0]ev_clear_addr_7__N_2209;
    wire [3:0]ev_state_3__N_1912;
    wire [8:0]build_sum_8__N_2025;
    wire [3:0]ev_state_3__N_1900;
    wire [3:0]ev_state_3__N_1896;
    wire [6:0]ev_ch_6__N_1920;
    
    wire n15, n14_adj_3000, n21701, n14_adj_3001, n12666, n20, n19, 
        n18, n23163, n10_adj_3002;
    wire [3:0]ev_state_3__N_581;
    
    wire swap_pending_N_2475;
    wire [6:0]ev_ch_6__N_593;
    
    wire n22696, n22692, n21862, n21819, n22688, n4, n21802, n22682, 
        n22680, n22676, n13, n22674, n23449, n22672, n21861, n21843, 
        n10_adj_3003;
    wire [6:0]staging_rd_addr_6__N_785;
    
    wire n22409, n22477, n22668, n21842, n22662, n21860, n21841, 
        n22465, n6_adj_3004, n5, n22658, n40_adj_3005, n39_adj_3006, 
        n38_adj_3007, n37_adj_3008, n36_adj_3009, n35_adj_3010, n34_adj_3011, 
        n10_adj_3012, n12721, n12715, n21859, n21839, n21818, n21817, 
        n21838, n21858, n21816, n23447, n21837, n21815, n21857, 
        n9845, n9844, n9843, n9842, n9841, n9840, n9839, n9838, 
        mic_tick_N_2463, spi1_sck_N_457_enable_7, n21856, n12700, n21836, 
        n12727, n21814, n21813, n21835, n23145, mic_clk_N_2428, 
        n9837, n9836, n9835, n9834, n9833, n9832, n9831, n9830, 
        n9829, n9828, n9827, n9826, n9825, n9824, n9823, n9822, 
        n9821, n9820, n9819, n9818, n9817, n9816, n9815, n9814, 
        n9813, n9812, n9810, n9809, n9808, n9807, n9806, n9805, 
        n9804, n9803, n9802, n9801, n9800, n9799, n9798, n21812, 
        n21811, pll_clk_enable_715, n21834, n12354, pll_clk_enable_302, 
        n40_adj_3013, n39_adj_3014, n38_adj_3015, n37_adj_3016, n36_adj_3017, 
        n35_adj_3018, n34_adj_3019, n22638, pll_clk_enable_652, n22466, 
        n133, pll_clk_enable_758, pll_clk_enable_382, spi1_sck_c_enable_26, 
        pll_clk_enable_333, spi1_sck_c_enable_59, spi1_sck_c_enable_52, 
        n22632, n12258, n12246, n12240, n12234, n12228, n12222, 
        n12216, n12210, n12204, n12198, n12192, n12186, n12180, 
        n12174, n22630, n22622, n21855, n21854, n21787, n22424, 
        pll_clk_enable_268, n21801, pll_clk_enable_172, n21853, n21833, 
        n21832, n21947, n21852, pll_clk_enable_416, spi1_sck_c_enable_27, 
        n7_adj_3020, n21831, n36_adj_3021, n13915, n22604, n21830, 
        n21851, n21850, n13906, pll_clk_enable_10, n21849, n13910, 
        pll_clk_enable_27, n21810, n21800, pll_clk_enable_630, spi1_sck_c_enable_75, 
        n21788, n21867, n21789, n21866, n21799, n21809, n5_adj_3022, 
        n21808, n21798, n23206, n22416, n23162, n11189, n11187, 
        spi1_sck_c_enable_226, spi1_sck_c_enable_67, n21807, n21848, 
        n11024, n11022, n11020, n11018, n11016, n11014, n11012, 
        n11010, n11008, n11006, n11004, n11002, n11000, n10998, 
        n10996, n10994, n10992, n10990, n10988, n10986, n10984, 
        n10982, n10980, n10978, n10976, n10974, n10972, n10970, 
        n10968, n10966, n10964, n10962, n10960, n10958, n10956, 
        n10954, n10952, n10950, n10948, n10946, n10944, n10942, 
        n10940, n10938, n10936, n10934, n10932, n10930, n15201, 
        n10928, n10926, n10924, n10922, n10920, n10918, n10916, 
        n10914, n10912, n10910, n22467, n10908, n10906, n10904, 
        n10902, n10900, n10898, n10896, n10894, n10892, n10890, 
        n23142, n10888, n10886, n10884, n10882, n10880, n10878, 
        n10876, n10874, n10872, n10870, n21, n10868, n10866, n10864, 
        n10862, n10860, pll_clk_enable_759, pll_clk_enable_666, n10843, 
        n14130, n23160, n21806, n22596, n22594, n15_adj_3023, n22453, 
        spi1_sck_c_enable_83, spi1_sck_c_enable_139, n22584, n22582, 
        n22843, n22842, n22580, n22841, n22840, n22839, n22838, 
        n22837, n23428, n22836, n22835, n22834, n22833, n22832, 
        n22831, n23, n22830, n22829, n22828, n7_adj_3024, n22572, 
        n22827, n22826, n22825, n22824, n22823, n22822, n22821, 
        n22820, n22819, n22818, n22817, n15297, spi1_sck_c_enable_30, 
        n22816, n22815, n22814, n18687, n22485, pll_clk_enable_724, 
        n6_adj_3025, n23157, n174, pll_clk_enable_447, n22469, n23155, 
        n22812, n22811, n23201, n23200, n23199, n22810, spi1_sck_c_enable_326, 
        n23198, n22809, n23153, pll_clk_enable_1, n23196, n23203, 
        n22808, n22807, n166, n26_adj_3026, n23194, n22806, n22805, 
        n22804, n22803, n22550, n22802, n23152, n22801, n23193, 
        n24, n23192, n23190, n23095, n23094, n22800, n23092, n23091, 
        n22799, n22, n22798, n23189, n22797, n23187, n23186, n23202, 
        n23185, n22796, n22795, n22794, n23184, n18_adj_3027, n23210, 
        n22793, n22792, n22791, n22790, n13719, n13717, pll_clk_enable_18, 
        n23181, n23180, n22789, n22788, n22787, n22786, n22785, 
        n22784, n22783, n15103, n22781, n23208, n22780, n23149, 
        n22538, n23148, n23177, n23176, n23175, n23174, n23173, 
        n23172, n21957, n23171, n22779, n22782, n13211, n23170, 
        pll_clk_enable_648, n13217;
    
    VHI i2 (.Z(VCC_net));
    INV i14472 (.A(spi1_sck_c), .Z(spi1_sck_N_457));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:32])
    LUT4 i1_3_lut (.A(n13915), .B(init_shadow[69]), .C(ev_bit[69]), .Z(n12584)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut.init = 16'hecec;
    FD1S3AY cs_sync_355 (.D(cs_meta), .CK(pll_clk), .Q(cs_sync)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(244[12] 248[8])
    defparam cs_sync_355.GSR = "DISABLED";
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
    LUT4 i10_4_lut (.A(expected_next[11]), .B(expected_next[12]), .C(expected_next[14]), 
         .D(expected_next[15]), .Z(n24)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i10_4_lut.init = 16'hfffe;
    LUT4 i1_3_lut_adj_27 (.A(n13915), .B(init_shadow[68]), .C(ev_bit[68]), 
         .Z(n12578)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_27.init = 16'hecec;
    LUT4 i4_2_lut (.A(expected_next[8]), .B(expected_next[7]), .Z(n18_adj_3027)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i4_2_lut.init = 16'heeee;
    LUT4 i1_3_lut_adj_28 (.A(n13915), .B(init_shadow[67]), .C(ev_bit[67]), 
         .Z(n12572)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_28.init = 16'hecec;
    LUT4 i1264_2_lut_3_lut_4_lut (.A(ev_ch[2]), .B(n23177), .C(ev_ch[4]), 
         .D(ev_ch[3]), .Z(ev_ch_6__N_1920[4])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1264_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 i1_2_lut (.A(expected_next[0]), .B(expected_next[1]), .Z(n22465)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i1_2_lut.init = 16'heeee;
    LUT4 i1_4_lut (.A(spi_command[4]), .B(n7), .C(spi_command[6]), .D(n6), 
         .Z(n22409)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i1_4_lut.init = 16'h0002;
    FD1S3AY cs_sync_d_356 (.D(cs_sync), .CK(pll_clk), .Q(cs_sync_d)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(244[12] 248[8])
    defparam cs_sync_d_356.GSR = "DISABLED";
    FD1S3AX mem_1183 (.D(staging_rd_addr_6__N_785[2]), .CK(pll_clk), .Q(n9802));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1183.GSR = "DISABLED";
    FD1S3AX spi_rx_shift_i1 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(spi_rx_shift[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_rx_shift_i1.GSR = "ENABLED";
    FD1P3AX stop_toggle_seen_403 (.D(stop_toggle_sync), .SP(pll_clk_enable_1), 
            .CK(pll_clk), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam stop_toggle_seen_403.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_29 (.A(n13915), .B(init_shadow[66]), .C(ev_bit[66]), 
         .Z(n12566)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_29.init = 16'hecec;
    LUT4 i1_2_lut_3_lut_4_lut (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[3]), .D(spi_byte_count[2]), .Z(n21924)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut.init = 16'hfeee;
    LUT4 i1_2_lut_adj_30 (.A(spi_command[3]), .B(spi_command[7]), .Z(n6)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i1_2_lut_adj_30.init = 16'heeee;
    CCU2D global_phase_7__I_0_438_4 (.A0(global_phase[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21782), .COUT(n21783), .S0(next_phase[2]), 
          .S1(next_phase[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[34:67])
    defparam global_phase_7__I_0_438_4.INIT0 = 16'h5aaa;
    defparam global_phase_7__I_0_438_4.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_438_4.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_438_4.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_31 (.A(n13915), .B(init_shadow[65]), .C(ev_bit[65]), 
         .Z(n12560)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_31.init = 16'hecec;
    LUT4 i4015_4_lut (.A(ev_ch[2]), .B(staging_rd_addr[2]), .C(n15103), 
         .D(n22416), .Z(staging_rd_addr_6__N_785[2])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i4015_4_lut.init = 16'hcac0;
    LUT4 i1_3_lut_adj_32 (.A(n13915), .B(init_shadow[64]), .C(ev_bit[64]), 
         .Z(n12554)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_32.init = 16'hecec;
    LUT4 i1_3_lut_rep_65_4_lut (.A(ev_state[3]), .B(n23199), .C(ev_state[0]), 
         .D(n23194), .Z(pll_clk_enable_648)) /* synthesis lut_function=(!(A+(B+!(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i1_3_lut_rep_65_4_lut.init = 16'h1110;
    PFUMX i14352 (.BLUT(n23092), .ALUT(n23091), .C0(ev_state[3]), .Z(ev_state_3__N_581[2]));
    FD1S3AX mem_1181 (.D(staging_rd_addr_6__N_785[1]), .CK(pll_clk), .Q(n9800));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1181.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_33 (.A(ev_state[1]), .B(ev_state[0]), .Z(n22416)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_adj_33.init = 16'h2222;
    CCU2D fpga_time_1091_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21849), .COUT(n21850), .S0(n156), .S1(n155));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_11.INJECT1_1 = "NO";
    FD1P3AX spi_command_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i0.GSR = "ENABLED";
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    LUT4 i1_3_lut_adj_34 (.A(n13915), .B(init_shadow[63]), .C(ev_bit[63]), 
         .Z(n12548)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_34.init = 16'hecec;
    CCU2D add_491_7 (.A0(phase_frac[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21812), .COUT(n21813), .S0(phase_frac_sum[5]), .S1(phase_frac_sum[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_7.INIT0 = 16'h5555;
    defparam add_491_7.INIT1 = 16'h5555;
    defparam add_491_7.INJECT1_0 = "NO";
    defparam add_491_7.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_488_7 (.A0(expected_next_15__N_1301[7]), .B0(spi_extension_length[7]), 
          .C0(GND_net), .D0(GND_net), .A1(spi1_mosi_c_0), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21832), .COUT(n21833), .S0(expected_next[7]), 
          .S1(expected_next[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_7.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_488_7.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_488_7.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_7.INJECT1_1 = "NO";
    LUT4 i1_4_lut_4_lut (.A(ev_state[2]), .B(n23), .C(n21), .D(ev_state[0]), 
         .Z(ev_state_3__N_581[0])) /* synthesis lut_function=(!(A ((D)+!C)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i1_4_lut_4_lut.init = 16'h00f4;
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    FD1P3AX spi_version_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i0.GSR = "ENABLED";
    CCU2D add_491_5 (.A0(phase_frac[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21811), .COUT(n21812), .S0(phase_frac_sum[3]), .S1(phase_frac_sum[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_5.INIT0 = 16'h5aaa;
    defparam add_491_5.INIT1 = 16'h5aaa;
    defparam add_491_5.INJECT1_0 = "NO";
    defparam add_491_5.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i47 (.D(ev_bit[46]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i47.GSR = "DISABLED";
    FD1P3AX spi_update_flags_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_27), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1301[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_update_flags_i0_i0.GSR = "ENABLED";
    LUT4 i2_3_lut_4_lut (.A(frame_settle[0]), .B(n23181), .C(pll_clk_enable_1), 
         .D(pll_clk_enable_18), .Z(pll_clk_enable_27)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(387[17:37])
    defparam i2_3_lut_4_lut.init = 16'hfffe;
    LUT4 i7_4_lut (.A(ev_clear_addr[3]), .B(n14_adj_2998), .C(n10_adj_3002), 
         .D(ev_clear_addr[6]), .Z(ev_state_3__N_1912[1])) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut.init = 16'h8000;
    FD1P3AX spi_extension_length_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(expected_next[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i0.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i0.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i0 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i0.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i0 (.D(pending_sequence[0]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i0.GSR = "DISABLED";
    FD1S3AX phase_frac_i0 (.D(phase_frac_sum[0]), .CK(pll_clk), .Q(phase_frac[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i0.GSR = "DISABLED";
    LUT4 i10059_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[0]), .C(ev_state_3__N_1896[1]), 
         .D(n11189), .Z(n14_adj_2990)) /* synthesis lut_function=(!(A+!(B (C)+!B (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i10059_4_lut_4_lut.init = 16'h5140;
    FD1P3AX rgb_hold_i0_i0 (.D(rgb_values[0]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i0.GSR = "DISABLED";
    LUT4 mux_1209_i10_3_lut (.A(n9832), .B(n9833), .C(n9813), .Z(rd_data_15__N_2535[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i10_3_lut.init = 16'hcaca;
    LUT4 i6_4_lut (.A(ev_clear_addr[2]), .B(ev_clear_addr[1]), .C(ev_clear_addr[4]), 
         .D(ev_clear_addr[7]), .Z(n14_adj_2998)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut.init = 16'h8000;
    FD1P3AX spi_byte_count_i0_i0 (.D(spi_byte_count_15__N_1579[0]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i0.GSR = "ENABLED";
    FD1P3AX status_hold__i1 (.D(accepted_sequence[24]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i1.GSR = "DISABLED";
    FD1P3IX us_tx__i1 (.D(n10287), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_0)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i1.GSR = "DISABLED";
    FD1S3AX mem_1185 (.D(staging_rd_addr_6__N_785[3]), .CK(pll_clk), .Q(n9804));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1185.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_35 (.A(frame_settle[1]), .B(frame_settle[0]), .Z(n13717)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_35.init = 16'h9999;
    LUT4 i2_2_lut (.A(ev_clear_addr[0]), .B(ev_clear_addr[5]), .Z(n10_adj_3002)) /* synthesis lut_function=(A (B)) */ ;
    defparam i2_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_4_lut_adj_36 (.A(spi_bit_count[2]), .B(n23185), 
         .C(n23196), .D(n23145), .Z(n4)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i1_2_lut_3_lut_4_lut_adj_36.init = 16'h0008;
    LUT4 i14183_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[61]), 
         .C(status_hold[60]), .Z(n22827)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14183_3_lut_3_lut.init = 16'he4e4;
    FD1S3AX run_addr_reg_i0 (.D(n23180), .CK(pll_clk), .Q(run_addr_reg[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i0.GSR = "DISABLED";
    FD1P3IX ev_bit_i12 (.D(ev_bit[11]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i12.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_37 (.A(n13915), .B(init_shadow[62]), .C(ev_bit[62]), 
         .Z(n12542)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_37.init = 16'hecec;
    LUT4 i2_2_lut_adj_38 (.A(spi_command[5]), .B(spi_command[2]), .Z(n7)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i2_2_lut_adj_38.init = 16'heeee;
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    FD1S3AX phase_step_reg_378 (.D(phase_frac_sum[24]), .CK(pll_clk), .Q(phase_step_reg)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_step_reg_378.GSR = "DISABLED";
    LUT4 n15_bdd_2_lut_14338_4_lut (.A(n23194), .B(ev_state_3__N_1912[1]), 
         .C(ev_state[0]), .D(ev_state[1]), .Z(n23051)) /* synthesis lut_function=(A (B+((D)+!C))+!A (B (C+(D))+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam n15_bdd_2_lut_14338_4_lut.init = 16'hffca;
    LUT4 mux_1209_i1_3_lut (.A(n9814), .B(n9815), .C(n9813), .Z(rd_data_15__N_2535[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i1_3_lut.init = 16'hcaca;
    FD1S3AX phase_step_d1_380 (.D(phase_step_reg), .CK(pll_clk), .Q(phase_step_d1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_step_d1_380.GSR = "DISABLED";
    FD1S3AX phase_step_d2_381 (.D(phase_step_d1), .CK(pll_clk), .Q(phase_step_d2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_step_d2_381.GSR = "DISABLED";
    FD1S3AX phase_step_d3_382 (.D(phase_step_d2), .CK(pll_clk), .Q(phase_step_d3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_step_d3_382.GSR = "DISABLED";
    FD1S3AX swap_now_d1_383 (.D(swap_now), .CK(pll_clk), .Q(swap_now_d1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam swap_now_d1_383.GSR = "DISABLED";
    FD1S3AX swap_now_d2_384 (.D(swap_now_d1), .CK(pll_clk), .Q(swap_now_d2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam swap_now_d2_384.GSR = "DISABLED";
    FD1S3AX swap_now_d3_385 (.D(swap_now_d2), .CK(pll_clk), .Q(swap_now_d3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam swap_now_d3_385.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i0 (.D(ev_rd_data[0]), .CK(pll_clk), .Q(ev_run_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i0.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i0 (.D(accepted_sequence_spi[0]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i0.GSR = "DISABLED";
    FD1S3AX frame_toggle_meta_391 (.D(frame_toggle_spi), .CK(pll_clk), .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_toggle_meta_391.GSR = "DISABLED";
    FD1S3AX frame_toggle_sync_392 (.D(frame_toggle_meta), .CK(pll_clk), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_toggle_sync_392.GSR = "DISABLED";
    FD1S3AX stop_toggle_meta_393 (.D(stop_toggle_spi), .CK(pll_clk), .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam stop_toggle_meta_393.GSR = "DISABLED";
    FD1S3AX stop_toggle_sync_394 (.D(stop_toggle_meta), .CK(pll_clk), .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam stop_toggle_sync_394.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i0 (.D(accepted_sequence_meta[0]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i1 (.D(mic_data_0_c), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i1.GSR = "DISABLED";
    FD1S3AX invalid_frame_meta_397 (.D(invalid_frame_spi), .CK(pll_clk), 
            .Q(invalid_frame_meta)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam invalid_frame_meta_397.GSR = "DISABLED";
    FD1S3AX invalid_frame_sync_398 (.D(invalid_frame_meta), .CK(pll_clk), 
            .Q(status_flags_wire_15__N_1269[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam invalid_frame_sync_398.GSR = "DISABLED";
    FD1P3IX frame_req_401 (.D(n23428), .SP(pll_clk_enable_172), .CD(n13217), 
            .CK(pll_clk), .Q(frame_req)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_req_401.GSR = "DISABLED";
    LUT4 i14184_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[63]), 
         .C(status_hold[62]), .Z(n22828)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14184_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX swap_pending_405 (.D(swap_pending_N_2475), .SP(pll_clk_enable_9), 
            .CK(pll_clk), .Q(swap_pending)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam swap_pending_405.GSR = "DISABLED";
    FD1P3AX ev_ch_i0 (.D(ev_ch_6__N_593[0]), .SP(pll_clk_enable_10), .CK(pll_clk), 
            .Q(ev_ch[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_ch_i0.GSR = "DISABLED";
    FD1P3AX build_phase_i0 (.D(staging_q[8]), .SP(pll_clk_enable_333), .CK(pll_clk), 
            .Q(build_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i0.GSR = "DISABLED";
    LUT4 i14181_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[57]), 
         .C(status_hold[56]), .Z(n22825)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14181_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX build_sum_i0 (.D(build_sum_8__N_2025[0]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i0.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i0 (.D(ev_rd_data[0]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i0.GSR = "DISABLED";
    LUT4 i14179_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[53]), 
         .C(status_hold[52]), .Z(n22823)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14179_3_lut_3_lut.init = 16'he4e4;
    FD1S3AX staging_rd_addr_i0 (.D(staging_rd_addr_6__N_785[0]), .CK(pll_clk), 
            .Q(staging_rd_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam staging_rd_addr_i0.GSR = "DISABLED";
    FD1P3AX pending_sequence_i0 (.D(accepted_sequence_sync[0]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i0.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i1 (.D(mic_data_1_c), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i0.GSR = "DISABLED";
    FD1S3AX mic_tick_420 (.D(mic_tick_N_2463), .CK(pll_clk), .Q(mic_tick)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_tick_420.GSR = "DISABLED";
    FD1S3AX mic_clock_reg_422 (.D(mic_clk_N_2428), .CK(pll_clk), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_clock_reg_422.GSR = "DISABLED";
    LUT4 i1_2_lut_rep_59_3_lut_4_lut (.A(spi_bit_count[2]), .B(n23185), 
         .C(frame_end), .D(fpga_cs_n_c), .Z(spi1_sck_c_enable_30)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i1_2_lut_rep_59_3_lut_4_lut.init = 16'h0080;
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i0.GSR = "DISABLED";
    FD1S3AY cs_meta_354 (.D(fpga_cs_n_c), .CK(pll_clk), .Q(cs_meta)) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(244[12] 248[8])
    defparam cs_meta_354.GSR = "DISABLED";
    FD1P3IX ev_bit_i0 (.D(n23428), .SP(pll_clk_enable_447), .CD(n23172), 
            .CK(pll_clk), .Q(ev_bit[0])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i0.GSR = "DISABLED";
    FD1P3AX spi_expected_length_i0 (.D(expected_next[0]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i0.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i1 (.D(expected_next[1]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i1.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i2 (.D(expected_next[2]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[2])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i2.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i3 (.D(expected_next[3]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i3.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i4 (.D(expected_next[4]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i4.GSR = "ENABLED";
    FD1P3AY spi_expected_length_i5 (.D(expected_next[5]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[5])) /* synthesis lse_init_val=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i5.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i6 (.D(expected_next[6]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i6.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i7 (.D(expected_next[7]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i7.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i8 (.D(expected_next[8]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i8.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i9 (.D(expected_next[9]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i9.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i10 (.D(expected_next[10]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i10.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i11 (.D(expected_next[11]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i11.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i12 (.D(expected_next[12]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i12.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i13 (.D(expected_next[13]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i13.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i14 (.D(expected_next[14]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i14.GSR = "ENABLED";
    FD1P3AX spi_expected_length_i15 (.D(expected_next[15]), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .Q(spi_expected_length[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_expected_length_i15.GSR = "ENABLED";
    LUT4 i7_4_lut_adj_39 (.A(n9812), .B(n22710), .C(n22534), .D(n6_adj_3004), 
         .Z(n9813)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i7_4_lut_adj_39.init = 16'h0002;
    CCU2D global_phase_7__I_0_438_2 (.A0(global_phase[0]), .B0(phase_step_reg), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .COUT(n21782), .S1(next_phase[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[34:67])
    defparam global_phase_7__I_0_438_2.INIT0 = 16'h7000;
    defparam global_phase_7__I_0_438_2.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_438_2.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_438_2.INJECT1_1 = "NO";
    LUT4 i14180_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[55]), 
         .C(status_hold[54]), .Z(n22824)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14180_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_40 (.A(staging_q[0]), .B(n14_adj_3001), .C(n10_adj_3012), 
         .D(staging_q[6]), .Z(ev_state_3__N_1900[2])) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[32:56])
    defparam i7_4_lut_adj_40.init = 16'hfffe;
    LUT4 i6_4_lut_adj_41 (.A(staging_q[3]), .B(staging_q[1]), .C(staging_q[5]), 
         .D(staging_q[7]), .Z(n14_adj_3001)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[32:56])
    defparam i6_4_lut_adj_41.init = 16'hfffe;
    LUT4 mux_1209_i11_3_lut (.A(n9834), .B(n9835), .C(n9813), .Z(rd_data_15__N_2535[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i11_3_lut.init = 16'hcaca;
    LUT4 i14178_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[51]), 
         .C(status_hold[50]), .Z(n22822)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14178_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14177_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[49]), 
         .C(status_hold[48]), .Z(n22821)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14177_3_lut_3_lut.init = 16'he4e4;
    LUT4 mux_1209_i12_3_lut (.A(n9836), .B(n9837), .C(n9813), .Z(rd_data_15__N_2535[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i12_3_lut.init = 16'hcaca;
    LUT4 i14182_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[59]), 
         .C(status_hold[58]), .Z(n22826)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14182_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_adj_42 (.A(n13915), .B(init_shadow[61]), .C(ev_bit[61]), 
         .Z(n12536)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_42.init = 16'hecec;
    LUT4 mux_1209_i13_3_lut (.A(n9838), .B(n9839), .C(n9813), .Z(rd_data_15__N_2535[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i13_3_lut.init = 16'hcaca;
    LUT4 i2_2_lut_3_lut_4_lut (.A(spi_bit_count[2]), .B(n23185), .C(n23189), 
         .D(n23186), .Z(n7_adj_3020)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i2_2_lut_3_lut_4_lut.init = 16'h0080;
    LUT4 i14174_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[43]), 
         .C(status_hold[42]), .Z(n22818)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14174_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14067_4_lut (.A(n9801), .B(n22530), .C(n5), .D(n9802), .Z(n22710)) /* synthesis lut_function=(A (B+(C+!(D)))+!A (B+(C+(D)))) */ ;
    defparam i14067_4_lut.init = 16'hfdfe;
    LUT4 i2_2_lut_adj_43 (.A(staging_q[2]), .B(staging_q[4]), .Z(n10_adj_3012)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(438[32:56])
    defparam i2_2_lut_adj_43.init = 16'heeee;
    LUT4 run_addr_reg_8__I_0_i1_3_lut (.A(run_addr_reg[0]), .B(ev_rd_slot[0]), 
         .C(n15_adj_3023), .Z(event_rd_addr[0])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i1_3_lut.init = 16'hacac;
    LUT4 i14173_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[41]), 
         .C(status_hold[40]), .Z(n22817)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14173_3_lut_3_lut.init = 16'he4e4;
    LUT4 i3_3_lut_rep_60 (.A(spi_command[1]), .B(n6_adj_3025), .C(spi_command[0]), 
         .Z(n23142)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;
    defparam i3_3_lut_rep_60.init = 16'h0404;
    LUT4 mux_1209_i14_3_lut (.A(n9840), .B(n9841), .C(n9813), .Z(rd_data_15__N_2535[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i14_3_lut.init = 16'hcaca;
    LUT4 i14172_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[39]), 
         .C(status_hold[38]), .Z(n22816)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14172_3_lut_3_lut.init = 16'he4e4;
    LUT4 i10130_2_lut (.A(ev_state_3__N_1900[2]), .B(ev_state[1]), .Z(n11189)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i10130_2_lut.init = 16'h4444;
    LUT4 i14171_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[37]), 
         .C(status_hold[36]), .Z(n22815)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14171_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14176_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[47]), 
         .C(status_hold[46]), .Z(n22820)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14176_3_lut_3_lut.init = 16'he4e4;
    PFUMX i14190 (.BLUT(n22823), .ALUT(n22824), .C0(spi1_miso_N_2421[1]), 
          .Z(n22834));
    LUT4 i14170_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[35]), 
         .C(status_hold[34]), .Z(n22814)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14170_3_lut_3_lut.init = 16'he4e4;
    LUT4 mux_1209_i15_3_lut (.A(n9842), .B(n9843), .C(n9813), .Z(rd_data_15__N_2535[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i15_3_lut.init = 16'hcaca;
    LUT4 i14169_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[33]), 
         .C(status_hold[32]), .Z(n22813)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14169_3_lut_3_lut.init = 16'he4e4;
    LUT4 mux_1209_i16_3_lut (.A(n9844), .B(n9845), .C(n9813), .Z(rd_data_15__N_2535[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i16_3_lut.init = 16'hcaca;
    FD1P3AX frame_toggle_seen_399 (.D(frame_toggle_sync), .SP(pll_clk_enable_18), 
            .CK(pll_clk), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_toggle_seen_399.GSR = "DISABLED";
    FD1S3AX mem_1179 (.D(staging_rd_addr_6__N_785[0]), .CK(pll_clk), .Q(n9798));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1179.GSR = "DISABLED";
    LUT4 i14153_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[31]), 
         .C(status_hold[30]), .Z(n22797)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14153_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14152_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[29]), 
         .C(status_hold[28]), .Z(n22796)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14152_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_44 (.A(ev_run_hold[2]), .B(us_tx_c_2), .C(init_shadow[2]), 
         .D(swap_now_d3), .Z(n10862)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_44.init = 16'h5a66;
    LUT4 i14151_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[27]), 
         .C(status_hold[26]), .Z(n22795)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14151_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14150_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[25]), 
         .C(status_hold[24]), .Z(n22794)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14150_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14149_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[23]), 
         .C(status_hold[22]), .Z(n22793)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14149_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_45 (.A(ev_run_hold[3]), .B(us_tx_c_3), .C(init_shadow[3]), 
         .D(swap_now_d3), .Z(n10864)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_45.init = 16'h5a66;
    LUT4 i14175_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[45]), 
         .C(status_hold[44]), .Z(n22819)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14175_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14148_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[21]), 
         .C(status_hold[20]), .Z(n22792)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14148_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14147_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[19]), 
         .C(status_hold[18]), .Z(n22791)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14147_3_lut_3_lut.init = 16'he4e4;
    LUT4 i14146_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[17]), 
         .C(status_hold[16]), .Z(n22790)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14146_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_2_lut_adj_46 (.A(spi_byte_count[0]), .B(n22453), .Z(spi1_sck_c_enable_43)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i1_2_lut_adj_46.init = 16'h4444;
    LUT4 i14145_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[15]), 
         .C(status_hold[14]), .Z(n22789)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14145_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_47 (.A(ev_run_hold[4]), .B(us_tx_c_4), .C(init_shadow[4]), 
         .D(swap_now_d3), .Z(n10866)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_47.init = 16'h5a66;
    LUT4 i14144_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[13]), 
         .C(status_hold[12]), .Z(n22788)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14144_3_lut_3_lut.init = 16'he4e4;
    FD1P3AX global_phase_i0_i0 (.D(n23180), .SP(phase_step_reg), .CK(pll_clk), 
            .Q(global_phase[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i0.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_48 (.A(n13915), .B(init_shadow[60]), .C(ev_bit[60]), 
         .Z(n12528)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_48.init = 16'hecec;
    LUT4 mux_961_i1_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[0]), 
         .D(build_phase[0]), .Z(ev_wr_addr_8__N_796[0])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i1_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX spi_update_flags_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_27), 
            .CK(spi1_sck_c), .Q(expected_next_15__N_1342[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_update_flags_i0_i1.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_49 (.A(n13915), .B(init_shadow[59]), .C(ev_bit[59]), 
         .Z(n12522)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_49.init = 16'hecec;
    FD1P3AX active_bank_407 (.D(active_bank_N_795), .SP(pll_clk_enable_19), 
            .CK(pll_clk), .Q(active_bank)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam active_bank_407.GSR = "DISABLED";
    LUT4 mux_961_i2_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[1]), 
         .D(build_phase[1]), .Z(ev_wr_addr_8__N_796[1])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i2_3_lut_4_lut.init = 16'hf1e0;
    FD1S3AX mem_1194 (.D(spi_write), .CK(pll_clk), .Q(n9812));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1194.GSR = "DISABLED";
    LUT4 i13891_4_lut (.A(n9803), .B(n9797), .C(n9804), .D(n9798), .Z(n22534)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i13891_4_lut.init = 16'h7bde;
    CCU2D equal_1582_17 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n21705), 
          .S0(frame_end_N_2499));
    defparam equal_1582_17.INIT0 = 16'hFFFF;
    defparam equal_1582_17.INIT1 = 16'h0000;
    defparam equal_1582_17.INJECT1_0 = "NO";
    defparam equal_1582_17.INJECT1_1 = "NO";
    LUT4 frame_toggle_spi_I_0_2_lut_4_lut (.A(spi_command[1]), .B(n6_adj_3025), 
         .C(spi_command[0]), .D(frame_toggle_spi), .Z(frame_toggle_spi_N_2432)) /* synthesis lut_function=(A (D)+!A (B (C (D)+!C !(D))+!B (D))) */ ;
    defparam frame_toggle_spi_I_0_2_lut_4_lut.init = 16'hfb04;
    FD1S3AX mem_1187 (.D(staging_rd_addr_6__N_785[4]), .CK(pll_clk), .Q(n9806));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1187.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i7.GSR = "ENABLED";
    LUT4 i1_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[46]), 
         .D(ev_bit[46]), .Z(ev_wr_data[46])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut.init = 16'hddd0;
    LUT4 i14143_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[11]), 
         .C(status_hold[10]), .Z(n22787)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14143_3_lut_3_lut.init = 16'he4e4;
    CCU2D equal_1582_17_13152 (.A0(spi_expected_length[3]), .B0(spi_byte_count_15__N_1579[3]), 
          .C0(spi_expected_length[2]), .D0(spi_byte_count_15__N_1579[2]), 
          .A1(spi_expected_length[1]), .B1(spi_byte_count_15__N_1579[1]), 
          .C1(spi_expected_length[0]), .D1(spi_byte_count_15__N_1579[0]), 
          .CIN(n21704), .COUT(n21705));
    defparam equal_1582_17_13152.INIT0 = 16'h9009;
    defparam equal_1582_17_13152.INIT1 = 16'h9009;
    defparam equal_1582_17_13152.INJECT1_0 = "YES";
    defparam equal_1582_17_13152.INJECT1_1 = "YES";
    FD1P3IX ev_bit_i46 (.D(ev_bit[45]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i46.GSR = "DISABLED";
    CCU2D add_493_2 (.A0(staging_q[8]), .B0(staging_q[0]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[9]), .B1(staging_q[1]), .C1(GND_net), 
          .D1(GND_net), .COUT(n21787), .S1(build_sum_8__N_2025[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_493_2.INIT0 = 16'h7000;
    defparam add_493_2.INIT1 = 16'h5666;
    defparam add_493_2.INJECT1_0 = "NO";
    defparam add_493_2.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i11 (.D(ev_bit[10]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i11.GSR = "DISABLED";
    LUT4 i3_4_lut (.A(n22477), .B(spi_byte_count[3]), .C(n22469), .D(n23152), 
         .Z(n22453)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i3_4_lut.init = 16'h0100;
    LUT4 i1_3_lut_adj_50 (.A(n13915), .B(init_shadow[58]), .C(ev_bit[58]), 
         .Z(n12516)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_50.init = 16'hecec;
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    FD1P3AX invalid_frame_spi_375 (.D(invalid_frame_spi_N_2450), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .Q(invalid_frame_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam invalid_frame_spi_375.GSR = "DISABLED";
    FD1S3AX mem_1189 (.D(staging_rd_addr_6__N_785[5]), .CK(pll_clk), .Q(n9808));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1189.GSR = "DISABLED";
    LUT4 i13214_2_lut_3_lut_4_lut (.A(mic_sample_count[1]), .B(n23192), 
         .C(mic_sample_count[3]), .D(mic_sample_count[2]), .Z(n27)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam i13214_2_lut_3_lut_4_lut.init = 16'h78f0;
    FD1P3AX frame_toggle_spi_373 (.D(frame_toggle_spi_N_2432), .SP(spi1_sck_c_enable_30), 
            .CK(spi1_sck_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam frame_toggle_spi_373.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i6.GSR = "ENABLED";
    CCU2D add_493_6 (.A0(staging_q[12]), .B0(staging_q[4]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[13]), .B1(staging_q[5]), .C1(GND_net), 
          .D1(GND_net), .CIN(n21788), .COUT(n21789), .S0(build_sum_8__N_2025[4]), 
          .S1(build_sum_8__N_2025[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_493_6.INIT0 = 16'h5666;
    defparam add_493_6.INIT1 = 16'h5666;
    defparam add_493_6.INJECT1_0 = "NO";
    defparam add_493_6.INJECT1_1 = "NO";
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    OB spi1_miso_pad (.I(spi1_miso_c), .O(spi1_miso));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(63[24:33])
    FD1P3AX spi_version_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i5.GSR = "ENABLED";
    LUT4 i14250_3_lut_4_lut (.A(ev_state[3]), .B(pll_clk_enable_1), .C(n14), 
         .D(ev_state[0]), .Z(pll_clk_enable_652)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;
    defparam i14250_3_lut_4_lut.init = 16'hefff;
    LUT4 equal_1190_i6_2_lut (.A(n9807), .B(n9808), .Z(n6_adj_3004)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam equal_1190_i6_2_lut.init = 16'h6666;
    CCU2D equal_1582_15 (.A0(spi_expected_length[7]), .B0(spi_byte_count_15__N_1579[7]), 
          .C0(spi_expected_length[6]), .D0(spi_byte_count_15__N_1579[6]), 
          .A1(spi_expected_length[5]), .B1(spi_byte_count_15__N_1579[5]), 
          .C1(spi_expected_length[4]), .D1(spi_byte_count_15__N_1579[4]), 
          .CIN(n21703), .COUT(n21704));
    defparam equal_1582_15.INIT0 = 16'h9009;
    defparam equal_1582_15.INIT1 = 16'h9009;
    defparam equal_1582_15.INJECT1_0 = "YES";
    defparam equal_1582_15.INJECT1_1 = "YES";
    FD1S3AX mem (.D(spi_phase_pending[7]), .CK(spi1_sck_c), .Q(n9845));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem.GSR = "DISABLED";
    FD1P3IX running_404 (.D(n23428), .SP(swap_now_d1), .CD(pll_clk_enable_1), 
            .CK(pll_clk), .Q(status_flags_wire_15__N_1285[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam running_404.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i4.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_51 (.A(n13915), .B(init_shadow[57]), .C(ev_bit[57]), 
         .Z(n12510)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_51.init = 16'hecec;
    FD1S3AX mem_1209 (.D(spi_phase_pending[6]), .CK(spi1_sck_c), .Q(n9843));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1209.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_52 (.A(spi_byte_count[2]), .B(n23193), .C(spi_byte_count[5]), 
         .D(spi_byte_count[4]), .Z(n13910)) /* synthesis lut_function=(!(A+((C (D)+!C !(D))+!B))) */ ;
    defparam i1_3_lut_4_lut_adj_52.init = 16'h0440;
    LUT4 i1_3_lut_adj_53 (.A(n13915), .B(init_shadow[56]), .C(ev_bit[56]), 
         .Z(n12504)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_53.init = 16'hecec;
    FD1S3AX mem_1208 (.D(spi_phase_pending[5]), .CK(spi1_sck_c), .Q(n9841));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1208.GSR = "DISABLED";
    FD1S3IX frame_settle__i0 (.D(n14130), .CK(pll_clk), .CD(n11187), .Q(frame_settle[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_settle__i0.GSR = "DISABLED";
    FD1S3AX mem_1207 (.D(spi_phase_pending[4]), .CK(spi1_sck_c), .Q(n9839));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1207.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i3.GSR = "ENABLED";
    FD1S3AX mem_1206 (.D(spi_phase_pending[3]), .CK(spi1_sck_c), .Q(n9837));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1206.GSR = "DISABLED";
    FD1S3AX mem_1205 (.D(spi_phase_pending[2]), .CK(spi1_sck_c), .Q(n9835));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1205.GSR = "DISABLED";
    FD1S3AX mem_1204 (.D(spi_phase_pending[1]), .CK(spi1_sck_c), .Q(n9833));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1204.GSR = "DISABLED";
    FD1P3AX spi_version_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i2.GSR = "ENABLED";
    FD1S3AX mem_1203 (.D(spi_phase_pending[0]), .CK(spi1_sck_c), .Q(n9831));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1203.GSR = "DISABLED";
    LUT4 i14142_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[9]), 
         .C(status_hold[8]), .Z(n22786)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14142_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_2_lut_rep_74_4_lut (.A(ev_state[3]), .B(n23199), .C(ev_state[0]), 
         .D(n23194), .Z(pll_clk_enable_447)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i1_2_lut_rep_74_4_lut.init = 16'h0100;
    FD1P3AX spi_version_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .Q(spi_version[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_version_i0_i1.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_54 (.A(n13915), .B(init_shadow[55]), .C(ev_bit[55]), 
         .Z(n12498)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_54.init = 16'hecec;
    LUT4 i1_2_lut_3_lut_4_lut_adj_55 (.A(ev_state[2]), .B(n23198), .C(n23171), 
         .D(n23194), .Z(pll_clk_enable_758)) /* synthesis lut_function=(!(A (C+!(D))+!A !(B+!(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i1_2_lut_3_lut_4_lut_adj_55.init = 16'h4f44;
    LUT4 i7_4_lut_adj_56 (.A(ev_run_hold[5]), .B(us_tx_c_5), .C(init_shadow[5]), 
         .D(swap_now_d3), .Z(n10868)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_56.init = 16'h5a66;
    FD1S3AX mem_1202 (.D(spi_rx_shift[6]), .CK(spi1_sck_c), .Q(n9829));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1202.GSR = "DISABLED";
    CCU2D equal_1582_11 (.A0(spi_expected_length[15]), .B0(spi_byte_count_15__N_1579[15]), 
          .C0(spi_expected_length[14]), .D0(spi_byte_count_15__N_1579[14]), 
          .A1(spi_expected_length[13]), .B1(spi_byte_count_15__N_1579[13]), 
          .C1(spi_expected_length[12]), .D1(spi_byte_count_15__N_1579[12]), 
          .CIN(n21701), .COUT(n21702));
    defparam equal_1582_11.INIT0 = 16'h9009;
    defparam equal_1582_11.INIT1 = 16'h9009;
    defparam equal_1582_11.INJECT1_0 = "YES";
    defparam equal_1582_11.INJECT1_1 = "YES";
    FD1S3AX mem_1201 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(n9827));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1201.GSR = "DISABLED";
    FD1S3AX mem_1200 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(n9825));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1200.GSR = "DISABLED";
    FD1S3AX mem_1199 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(n9823));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1199.GSR = "DISABLED";
    FD1S3AX mem_1198 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(n9821));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1198.GSR = "DISABLED";
    FD1S3AX mem_1197 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(n9819));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1197.GSR = "DISABLED";
    FD1S3AX mem_1196 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(n9817));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1196.GSR = "DISABLED";
    FD1S3AX mem_1195 (.D(spi1_mosi_c_0), .CK(spi1_sck_c), .Q(n9815));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mem_1195.GSR = "DISABLED";
    FD1P3AX spi_command_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i7.GSR = "ENABLED";
    CCU2D equal_1582_13 (.A0(spi_expected_length[11]), .B0(spi_byte_count_15__N_1579[11]), 
          .C0(spi_expected_length[10]), .D0(spi_byte_count_15__N_1579[10]), 
          .A1(spi_expected_length[9]), .B1(spi_byte_count_15__N_1579[9]), 
          .C1(spi_expected_length[8]), .D1(spi_byte_count_15__N_1579[8]), 
          .CIN(n21702), .COUT(n21703));
    defparam equal_1582_13.INIT0 = 16'h9009;
    defparam equal_1582_13.INIT1 = 16'h9009;
    defparam equal_1582_13.INJECT1_0 = "YES";
    defparam equal_1582_13.INJECT1_1 = "YES";
    FD1P3AX spi_command_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i5.GSR = "ENABLED";
    LUT4 i6541_2_lut_3_lut_4_lut (.A(ev_state[2]), .B(n23198), .C(n23171), 
         .D(n23194), .Z(n15201)) /* synthesis lut_function=(!(A (C+!(D))+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i6541_2_lut_3_lut_4_lut.init = 16'h0b00;
    FD1P3AX spi_command_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i4.GSR = "ENABLED";
    LUT4 i7_4_lut_adj_57 (.A(ev_run_hold[6]), .B(us_tx_c_6), .C(init_shadow[6]), 
         .D(swap_now_d3), .Z(n10870)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_57.init = 16'h5a66;
    FD1P3AX spi_command_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i3.GSR = "ENABLED";
    LUT4 i7_4_lut_adj_58 (.A(ev_run_hold[7]), .B(us_tx_c_7), .C(init_shadow[7]), 
         .D(swap_now_d3), .Z(n10872)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_58.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_59 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[36]), 
         .D(ev_bit[36]), .Z(ev_wr_data[36])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_59.init = 16'hddd0;
    LUT4 i13887_4_lut (.A(n9809), .B(n9799), .C(n9810), .D(n9800), .Z(n22530)) /* synthesis lut_function=(!(A (B (C (D))+!B !((D)+!C))+!A !(B (C+!(D))+!B (C+(D))))) */ ;
    defparam i13887_4_lut.init = 16'h7bde;
    LUT4 i1_3_lut_adj_60 (.A(n13915), .B(init_shadow[54]), .C(ev_bit[54]), 
         .Z(n12492)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_60.init = 16'hecec;
    LUT4 i1_2_lut_rep_92 (.A(spi_byte_count[12]), .B(n22485), .Z(n23174)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(132[36:83])
    defparam i1_2_lut_rep_92.init = 16'heeee;
    LUT4 mux_961_i3_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[2]), 
         .D(build_phase[2]), .Z(ev_wr_addr_8__N_796[2])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i3_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i14089_3_lut_4_lut (.A(spi_byte_count[12]), .B(n22485), .C(spi_byte_count[13]), 
         .D(n23176), .Z(n22732)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(132[36:83])
    defparam i14089_3_lut_4_lut.init = 16'hfffe;
    CCU2D fpga_time_1091_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21848), .COUT(n21849), .S0(n158), .S1(n157));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_9.INJECT1_1 = "NO";
    FD1P3AX spi_command_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_command_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .Q(spi_command[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_command_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1088__i0 (.D(n40_adj_3013), .SP(spi1_sck_c_enable_324), 
            .CK(spi1_sck_c), .Q(spi_channel_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088__i0.GSR = "ENABLED";
    LUT4 i14141_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[7]), 
         .C(status_hold[6]), .Z(n22785)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14141_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_2_lut_adj_61 (.A(mic_clk_c), .B(mic_sample_count[0]), .Z(n21957)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_61.init = 16'h9999;
    LUT4 i1_2_lut_rep_94 (.A(spi_byte_count[15]), .B(spi_byte_count[14]), 
         .Z(n23176)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_94.init = 16'heeee;
    LUT4 i3_2_lut_3_lut_4_lut (.A(spi_byte_count[15]), .B(spi_byte_count[14]), 
         .C(n23189), .D(spi_byte_count[13]), .Z(n9)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i3_2_lut_3_lut_4_lut.init = 16'hffef;
    LUT4 equal_1190_i5_2_lut (.A(n9805), .B(n9806), .Z(n5)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam equal_1190_i5_2_lut.init = 16'h6666;
    LUT4 i1_2_lut_rep_84_3_lut (.A(spi_byte_count[15]), .B(spi_byte_count[14]), 
         .C(spi_byte_count[13]), .Z(n23166)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i1_2_lut_rep_84_3_lut.init = 16'hfefe;
    LUT4 i14140_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[5]), 
         .C(status_hold[4]), .Z(n22784)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14140_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1_3_lut_4_lut_adj_62 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[38]), 
         .D(ev_bit[38]), .Z(ev_wr_data[38])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_62.init = 16'hddd0;
    LUT4 i14139_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[3]), 
         .C(status_hold[2]), .Z(n22783)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14139_3_lut_3_lut.init = 16'he4e4;
    LUT4 i13929_3_lut_4_lut (.A(spi_byte_count[15]), .B(spi_byte_count[14]), 
         .C(n22485), .D(n13906), .Z(n22572)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i13929_3_lut_4_lut.init = 16'hfffe;
    LUT4 i14262_2_lut_4_lut (.A(ev_state[0]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(ev_state[3]), .Z(pll_clk_enable_333)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i14262_2_lut_4_lut.init = 16'h0400;
    FD1P3IX ev_bit_i45 (.D(ev_bit[44]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i45.GSR = "DISABLED";
    FD1P3IX ev_bit_i53 (.D(ev_bit[52]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i53.GSR = "DISABLED";
    FD1P3IX ev_bit_i44 (.D(ev_bit[43]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i44.GSR = "DISABLED";
    LUT4 i14138_3_lut_3_lut (.A(status_bit_index[0]), .B(status_hold[1]), 
         .C(status_hold[0]), .Z(n22782)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14138_3_lut_3_lut.init = 16'he4e4;
    LUT4 i1245_2_lut_rep_95 (.A(ev_ch[1]), .B(ev_ch[0]), .Z(n23177)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1245_2_lut_rep_95.init = 16'h8888;
    LUT4 stop_toggle_sync_I_0_2_lut_rep_115 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(pll_clk_enable_1)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam stop_toggle_sync_I_0_2_lut_rep_115.init = 16'h6666;
    LUT4 i4_4_lut (.A(n7_adj_3020), .B(n22732), .C(spi_byte_count[5]), 
         .D(n23168), .Z(spi1_sck_c_enable_26)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam i4_4_lut.init = 16'h0020;
    LUT4 i14257_2_lut_3_lut_4_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(n23052), .D(ev_state[3]), .Z(pll_clk_enable_724)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam i14257_2_lut_3_lut_4_lut.init = 16'hfff6;
    FD1P3AX status_bit_index_1087__i0 (.D(n40_adj_3005), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087__i0.GSR = "ENABLED";
    FD1P3AX fpga_time_1091__i0 (.D(n165), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i0.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1090__i0 (.D(n20), .CK(spi1_sck_c), .Q(spi_bit_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam spi_bit_count_1090__i0.GSR = "ENABLED";
    CCU2D add_491_3 (.A0(phase_frac[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21810), .COUT(n21811), .S0(phase_frac_sum[1]), .S1(phase_frac_sum[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_3.INIT0 = 16'h5aaa;
    defparam add_491_3.INIT1 = 16'h5aaa;
    defparam add_491_3.INJECT1_0 = "NO";
    defparam add_491_3.INJECT1_1 = "NO";
    LUT4 run_addr_reg_8__I_0_i2_3_lut (.A(run_addr_reg[1]), .B(ev_rd_slot[1]), 
         .C(n15_adj_3023), .Z(event_rd_addr[1])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i2_3_lut.init = 16'hacac;
    FD1P3IX ev_bit_i52 (.D(ev_bit[51]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i52.GSR = "DISABLED";
    LUT4 i1252_2_lut_rep_80_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(n23162)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1252_2_lut_rep_80_3_lut.init = 16'h8080;
    LUT4 i1_3_lut_4_lut_adj_63 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[47]), 
         .D(ev_bit[47]), .Z(ev_wr_data[47])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_63.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_64 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[66]), 
         .D(ev_bit[66]), .Z(ev_wr_data[66])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_64.init = 16'hddd0;
    LUT4 i569_1_lut_2_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), .Z(pll_clk_enable_19)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam i569_1_lut_2_lut.init = 16'h9999;
    LUT4 i14276_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_75)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i14276_2_lut_3_lut.init = 16'h0404;
    LUT4 i4_4_lut_adj_65 (.A(n23166), .B(spi_byte_count[12]), .C(spi_byte_count[10]), 
         .D(n6_adj_2985), .Z(n22477)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_65.init = 16'hfffe;
    FD1P3IX spi_channel_field_1089__i1 (.D(n14_adj_3000), .SP(spi1_sck_c_enable_325), 
            .CD(spi1_sck_c_enable_324), .CK(spi1_sck_c), .Q(spi_channel_field[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(316[78:102])
    defparam spi_channel_field_1089__i1.GSR = "ENABLED";
    FD1P3IX frame_settle__i3 (.D(frame_settle_3__N_1860[3]), .SP(pll_clk_enable_27), 
            .CD(pll_clk_enable_1), .CK(pll_clk), .Q(frame_settle[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_settle__i3.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_66 (.A(spi1_sck_c_enable_287), .B(n23145), .C(n23170), 
         .D(spi_byte_count[0]), .Z(n22369)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i1_3_lut_4_lut_adj_66.init = 16'hffdf;
    FD1P3IX ev_clear_addr_i7 (.D(ev_clear_addr_7__N_2209[7]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i7.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i6 (.D(ev_clear_addr_7__N_2209[6]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i6.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i5 (.D(ev_clear_addr_7__N_2209[5]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i5.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i4 (.D(ev_clear_addr_7__N_2209[4]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i4.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i3 (.D(ev_clear_addr_7__N_2209[3]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i3.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i2 (.D(ev_clear_addr_7__N_2209[2]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i2.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i1 (.D(ev_clear_addr_7__N_2209[1]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i1.GSR = "DISABLED";
    FD1P3IX init_shadow_i83 (.D(n12745), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i83.GSR = "DISABLED";
    FD1P3IX init_shadow_i82 (.D(n12739), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i82.GSR = "DISABLED";
    FD1P3IX init_shadow_i81 (.D(n12733), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i81.GSR = "DISABLED";
    FD1P3IX init_shadow_i80 (.D(n12727), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i80.GSR = "DISABLED";
    FD1P3IX init_shadow_i79 (.D(n12721), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i79.GSR = "DISABLED";
    FD1P3IX init_shadow_i78 (.D(n12715), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i78.GSR = "DISABLED";
    FD1P3IX init_shadow_i77 (.D(n12700), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i77.GSR = "DISABLED";
    FD1P3IX init_shadow_i76 (.D(n12680), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i76.GSR = "DISABLED";
    FD1P3IX init_shadow_i75 (.D(n12666), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i75.GSR = "DISABLED";
    FD1P3IX init_shadow_i74 (.D(n12652), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i74.GSR = "DISABLED";
    FD1P3IX init_shadow_i73 (.D(n12636), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i73.GSR = "DISABLED";
    FD1P3IX init_shadow_i72 (.D(n12620), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i72.GSR = "DISABLED";
    FD1P3IX init_shadow_i71 (.D(n12599), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i71.GSR = "DISABLED";
    FD1P3IX init_shadow_i70 (.D(n12590), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i70.GSR = "DISABLED";
    FD1P3IX init_shadow_i69 (.D(n12584), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i69.GSR = "DISABLED";
    FD1P3IX init_shadow_i68 (.D(n12578), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i68.GSR = "DISABLED";
    FD1P3IX init_shadow_i67 (.D(n12572), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i67.GSR = "DISABLED";
    FD1P3IX init_shadow_i66 (.D(n12566), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i66.GSR = "DISABLED";
    FD1P3IX init_shadow_i65 (.D(n12560), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i65.GSR = "DISABLED";
    FD1P3IX init_shadow_i64 (.D(n12554), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i64.GSR = "DISABLED";
    FD1P3IX init_shadow_i63 (.D(n12548), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i63.GSR = "DISABLED";
    FD1P3IX init_shadow_i62 (.D(n12542), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i62.GSR = "DISABLED";
    FD1P3IX init_shadow_i61 (.D(n12536), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i61.GSR = "DISABLED";
    FD1P3IX init_shadow_i60 (.D(n12528), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i60.GSR = "DISABLED";
    FD1P3IX init_shadow_i59 (.D(n12522), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i59.GSR = "DISABLED";
    FD1P3IX init_shadow_i58 (.D(n12516), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i58.GSR = "DISABLED";
    FD1P3IX init_shadow_i57 (.D(n12510), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i57.GSR = "DISABLED";
    FD1P3IX init_shadow_i56 (.D(n12504), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i56.GSR = "DISABLED";
    FD1P3IX init_shadow_i55 (.D(n12498), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i55.GSR = "DISABLED";
    FD1P3IX init_shadow_i54 (.D(n12492), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i54.GSR = "DISABLED";
    LUT4 i570_2_lut_3_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), .C(swap_now_d1), 
         .Z(pll_clk_enable_124)) /* synthesis lut_function=(A (B (C))+!A !(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam i570_2_lut_3_lut.init = 16'h9090;
    LUT4 mux_961_i6_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[5]), 
         .D(build_phase[5]), .Z(ev_wr_addr_8__N_796[5])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i6_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i1_2_lut_3_lut_4_lut_adj_67 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1285[4]), .Z(n9487)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam i1_2_lut_3_lut_4_lut_adj_67.init = 16'h6fff;
    LUT4 i2_3_lut_4_lut_adj_68 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(swap_pending_N_2475), .D(swap_now_d1), .Z(pll_clk_enable_9)) /* synthesis lut_function=(A ((C+(D))+!B)+!A (B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam i2_3_lut_4_lut_adj_68.init = 16'hfff6;
    FD1P3IX ev_bit_i51 (.D(ev_bit[50]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i51.GSR = "DISABLED";
    FD1P3IX ev_bit_i50 (.D(ev_bit[49]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i50.GSR = "DISABLED";
    FD1P3IX ev_bit_i49 (.D(ev_bit[48]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i49.GSR = "DISABLED";
    FD1P3IX ev_bit_i48 (.D(ev_bit[47]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i48.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_69 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[67]), 
         .D(ev_bit[67]), .Z(ev_wr_data[67])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_69.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_70 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[68]), 
         .D(ev_bit[68]), .Z(ev_wr_data[68])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_70.init = 16'hddd0;
    LUT4 i7_4_lut_adj_71 (.A(ev_run_hold[8]), .B(us_tx_c_8), .C(init_shadow[8]), 
         .D(swap_now_d3), .Z(n10874)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_71.init = 16'h5a66;
    FD1S3IX mic_divider_1094__i0 (.D(n40_adj_2991), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094__i0.GSR = "DISABLED";
    LUT4 i2453_2_lut_3_lut_4_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(frame_toggle_seen), .D(frame_toggle_sync), .Z(n11187)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D)))+!A !(B+!(C (D)+!C !(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam i2453_2_lut_3_lut_4_lut.init = 16'h6ff6;
    FD1S3AX spi_rx_shift_i7 (.D(spi_rx_shift[5]), .CK(spi1_sck_c), .Q(spi_rx_shift[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_rx_shift_i7.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i6 (.D(spi_rx_shift[4]), .CK(spi1_sck_c), .Q(spi_rx_shift[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_rx_shift_i6.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i5 (.D(spi_rx_shift[3]), .CK(spi1_sck_c), .Q(spi_rx_shift[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_rx_shift_i5.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i4 (.D(spi_rx_shift[2]), .CK(spi1_sck_c), .Q(spi_rx_shift[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_rx_shift_i4.GSR = "ENABLED";
    FD1S3AX spi_rx_shift_i3 (.D(spi_rx_shift[1]), .CK(spi1_sck_c), .Q(spi_rx_shift[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_rx_shift_i3.GSR = "ENABLED";
    LUT4 i1250_2_lut_3_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[2]), 
         .Z(ev_ch_6__N_1920[2])) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1250_2_lut_3_lut.init = 16'h7878;
    FD1S3AX spi_rx_shift_i2 (.D(spi_rx_shift[0]), .CK(spi1_sck_c), .Q(spi_rx_shift[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_rx_shift_i2.GSR = "ENABLED";
    FD1P3AX mic_sample_count_1093__i0 (.D(n21957), .SP(mic_tick), .CK(pll_clk), 
            .Q(mic_sample_count[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam mic_sample_count_1093__i0.GSR = "DISABLED";
    FD1S3AX time_divider_1092__i0 (.D(n40), .CK(pll_clk), .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092__i0.GSR = "DISABLED";
    FD1P3AX spi_extension_length_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(expected_next[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i1.GSR = "ENABLED";
    LUT4 i25_4_lut_4_lut (.A(ev_state[2]), .B(n15_adj_3023), .C(ev_state[1]), 
         .D(ev_state_3__N_1912[1]), .Z(n14)) /* synthesis lut_function=(A (B (C))+!A !(C+(D))) */ ;
    defparam i25_4_lut_4_lut.init = 16'h8085;
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(64[24:29])
    LUT4 i1278_3_lut_4_lut (.A(ev_ch[4]), .B(n23148), .C(ev_ch[5]), .D(ev_ch[6]), 
         .Z(ev_ch_6__N_1920[6])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1278_3_lut_4_lut.init = 16'h7f80;
    LUT4 i1259_2_lut_rep_66_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(n23148)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1259_2_lut_rep_66_3_lut_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_72 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[69]), 
         .D(ev_bit[69]), .Z(ev_wr_data[69])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_72.init = 16'hddd0;
    LUT4 i1257_2_lut_3_lut_4_lut (.A(ev_ch[1]), .B(ev_ch[0]), .C(ev_ch[3]), 
         .D(ev_ch[2]), .Z(ev_ch_6__N_1920[3])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1257_2_lut_3_lut_4_lut.init = 16'h78f0;
    LUT4 mux_961_i7_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[6]), 
         .D(build_phase[6]), .Z(ev_wr_addr_8__N_796[6])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i7_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i1_2_lut_rep_116 (.A(ev_state[0]), .B(ev_state[3]), .Z(n23198)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i1_2_lut_rep_116.init = 16'h8888;
    LUT4 mux_961_i8_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[7]), 
         .D(build_phase[7]), .Z(ev_wr_addr_8__N_796[7])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i8_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i13227_2_lut_rep_98 (.A(global_phase[0]), .B(phase_step_reg), .Z(n23180)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13227_2_lut_rep_98.init = 16'h6666;
    LUT4 i7_4_lut_adj_73 (.A(ev_run_hold[9]), .B(us_tx_c_9), .C(init_shadow[9]), 
         .D(swap_now_d3), .Z(n10876)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_73.init = 16'h5a66;
    LUT4 i7_4_lut_adj_74 (.A(ev_run_hold[10]), .B(us_tx_c_10), .C(init_shadow[10]), 
         .D(swap_now_d3), .Z(n10878)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_74.init = 16'h5a66;
    LUT4 i7_4_lut_adj_75 (.A(ev_run_hold[11]), .B(us_tx_c_11), .C(init_shadow[11]), 
         .D(swap_now_d3), .Z(n10880)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_75.init = 16'h5a66;
    LUT4 i7_4_lut_adj_76 (.A(ev_run_hold[12]), .B(us_tx_c_12), .C(init_shadow[12]), 
         .D(swap_now_d3), .Z(n10882)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_76.init = 16'h5a66;
    LUT4 i2_2_lut_rep_90_3_lut (.A(ev_state[0]), .B(ev_state[3]), .C(ev_state[2]), 
         .Z(n23172)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i2_2_lut_rep_90_3_lut.init = 16'h0808;
    LUT4 i7_4_lut_adj_77 (.A(ev_run_hold[13]), .B(us_tx_c_13), .C(init_shadow[13]), 
         .D(swap_now_d3), .Z(n10884)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_77.init = 16'h5a66;
    LUT4 i7_4_lut_adj_78 (.A(ev_run_hold[14]), .B(us_tx_c_14), .C(init_shadow[14]), 
         .D(swap_now_d3), .Z(n10886)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_78.init = 16'h5a66;
    LUT4 ev_state_3__N_1900_2__bdd_4_lut_14357 (.A(ev_state[0]), .B(ev_state[2]), 
         .C(ev_state[1]), .D(n15_adj_3023), .Z(n23095)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C))+!A)) */ ;
    defparam ev_state_3__N_1900_2__bdd_4_lut_14357.init = 16'h20a0;
    LUT4 i1_3_lut_4_lut_adj_79 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[70]), 
         .D(ev_bit[70]), .Z(ev_wr_data[70])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_79.init = 16'hddd0;
    LUT4 i14254_3_lut_4_lut (.A(ev_state[0]), .B(ev_state[3]), .C(ev_state[2]), 
         .D(ev_state_3__N_1896[1]), .Z(swap_pending_N_2475)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i14254_3_lut_4_lut.init = 16'h0008;
    LUT4 i1_2_lut_rep_117 (.A(ev_state[1]), .B(ev_state[2]), .Z(n23199)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i1_2_lut_rep_117.init = 16'heeee;
    LUT4 i10049_2_lut_3_lut_4_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[3]), 
         .D(n23149), .Z(n18687)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i10049_2_lut_3_lut_4_lut.init = 16'h0100;
    LUT4 i1_2_lut_rep_81_3_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[3]), 
         .Z(n23163)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i1_2_lut_rep_81_3_lut.init = 16'hfefe;
    LUT4 i6338_4_lut_4_lut_else_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(n23194), .Z(n23202)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i6338_4_lut_4_lut_else_4_lut.init = 16'h0100;
    LUT4 i2_3_lut_rep_89_4_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[0]), 
         .D(ev_state[3]), .Z(n23171)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i2_3_lut_rep_89_4_lut.init = 16'hfffe;
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
    LUT4 i6_4_lut_4_lut (.A(global_phase[0]), .B(phase_step_reg), .C(next_phase[6]), 
         .D(swap_pending), .Z(n16)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i6_4_lut_4_lut.init = 16'h0800;
    LUT4 active_bank_I_0_485_1_lut_rep_118 (.A(active_bank), .Z(n23200)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(215[56:68])
    defparam active_bank_I_0_485_1_lut_rep_118.init = 16'h5555;
    FD1P3AX spi_extension_length_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(spi_extension_length[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(spi_extension_length[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(spi_extension_length[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(spi_extension_length[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(spi_extension_length[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_extension_length_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .Q(spi_extension_length[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_extension_length_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i8 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i9 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i10 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i11 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i12 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i13 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i14 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i15 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i16 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i17 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i18 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i19 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i20 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i21 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i22 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i23 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i24 (.D(spi1_mosi_c_0), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i25 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i26 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i27 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i28 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i29 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i30 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_frame_sequence_i0_i31 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .Q(spi_frame_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_frame_sequence_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_phase_pending_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .Q(spi_phase_pending[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_phase_pending_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i8 (.D(spi_bitmap[0]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i9 (.D(spi_bitmap[1]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i10 (.D(spi_bitmap[2]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i11 (.D(spi_bitmap[3]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i12 (.D(spi_bitmap[4]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i13 (.D(spi_bitmap[5]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i14 (.D(spi_bitmap[6]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i15 (.D(spi_bitmap[7]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i15.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i16 (.D(spi_bitmap[8]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i16.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i17 (.D(spi_bitmap[9]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i17.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i18 (.D(spi_bitmap[10]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i18.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i19 (.D(spi_bitmap[11]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i19.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i20 (.D(spi_bitmap[12]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i20.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i21 (.D(spi_bitmap[13]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i21.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i22 (.D(spi_bitmap[14]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i22.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i23 (.D(spi_bitmap[15]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i23.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i24 (.D(spi_bitmap[16]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i24.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i25 (.D(spi_bitmap[17]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i25.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i26 (.D(spi_bitmap[18]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i26.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i27 (.D(spi_bitmap[19]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i27.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i28 (.D(spi_bitmap[20]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i28.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i29 (.D(spi_bitmap[21]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i29.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i30 (.D(spi_bitmap[22]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i30.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i31 (.D(spi_bitmap[23]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i31.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i32 (.D(spi_bitmap[24]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i32.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i33 (.D(spi_bitmap[25]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i33.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i34 (.D(spi_bitmap[26]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i34.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i35 (.D(spi_bitmap[27]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i35.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i36 (.D(spi_bitmap[28]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i36.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i37 (.D(spi_bitmap[29]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i37.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i38 (.D(spi_bitmap[30]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i38.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i39 (.D(spi_bitmap[31]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i39.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i40 (.D(spi_bitmap[32]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i40.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i41 (.D(spi_bitmap[33]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i41.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i42 (.D(spi_bitmap[34]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i42.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i43 (.D(spi_bitmap[35]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i43.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i44 (.D(spi_bitmap[36]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i44.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i45 (.D(spi_bitmap[37]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i45.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i46 (.D(spi_bitmap[38]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i46.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i47 (.D(spi_bitmap[39]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i47.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i48 (.D(spi_bitmap[40]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i48.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i49 (.D(spi_bitmap[41]), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .Q(spi_bitmap[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i49.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i50 (.D(spi_bitmap[42]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i50.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i51 (.D(spi_bitmap[43]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i51.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i52 (.D(spi_bitmap[44]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i52.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i53 (.D(spi_bitmap[45]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i53.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i54 (.D(spi_bitmap[46]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i54.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i55 (.D(spi_bitmap[47]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i55.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i56 (.D(spi_bitmap[48]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i56.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i57 (.D(spi_bitmap[49]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i57.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i58 (.D(spi_bitmap[50]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i58.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i59 (.D(spi_bitmap[51]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i59.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i60 (.D(spi_bitmap[52]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i60.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i61 (.D(spi_bitmap[53]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i61.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i62 (.D(spi_bitmap[54]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i62.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i63 (.D(spi_bitmap[55]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i63.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i64 (.D(spi_bitmap[56]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i64.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i65 (.D(spi_bitmap[57]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i65.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i66 (.D(spi_bitmap[58]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i66.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i67 (.D(spi_bitmap[59]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i67.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i68 (.D(spi_bitmap[60]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i68.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i69 (.D(spi_bitmap[61]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i69.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i70 (.D(spi_bitmap[62]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i70.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i71 (.D(spi_bitmap[63]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i71.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i72 (.D(spi_bitmap[64]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i72.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i73 (.D(spi_bitmap[65]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i73.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i74 (.D(spi_bitmap[66]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i74.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i75 (.D(spi_bitmap[67]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i75.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i76 (.D(spi_bitmap[68]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i76.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i77 (.D(spi_bitmap[69]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i77.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i78 (.D(spi_bitmap[70]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i78.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i79 (.D(spi_bitmap[71]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i79.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i80 (.D(spi_bitmap[72]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i80.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i81 (.D(spi_bitmap[73]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i81.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i82 (.D(spi_bitmap[74]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i82.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i83 (.D(spi_bitmap[75]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i83.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i84 (.D(spi_bitmap[76]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i84.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i85 (.D(spi_bitmap[77]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i85.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i86 (.D(spi_bitmap[78]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i86.GSR = "ENABLED";
    FD1P3AX spi_bitmap_i0_i87 (.D(spi_bitmap[79]), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .Q(spi_bitmap[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_bitmap_i0_i87.GSR = "ENABLED";
    FD1P3AX rgb_values_i0_i1 (.D(spi_rx_shift[0]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i1.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i2 (.D(spi_rx_shift[1]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i2.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i3 (.D(spi_rx_shift[2]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i3.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i4 (.D(spi_rx_shift[3]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i4.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i5 (.D(spi_rx_shift[4]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i5.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i6 (.D(spi_rx_shift[5]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i6.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i7 (.D(spi_rx_shift[6]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i7.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i8 (.D(rgb_values[0]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i8.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i9 (.D(rgb_values[1]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i9.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i10 (.D(rgb_values[2]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i10.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i11 (.D(rgb_values[3]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i11.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i12 (.D(rgb_values[4]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i12.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i13 (.D(rgb_values[5]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i13.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i14 (.D(rgb_values[6]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i14.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i15 (.D(rgb_values[7]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i15.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i16 (.D(rgb_values[8]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i16.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i17 (.D(rgb_values[9]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i17.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i18 (.D(rgb_values[10]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i18.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i19 (.D(rgb_values[11]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i19.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i20 (.D(rgb_values[12]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i20.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i21 (.D(rgb_values[13]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i21.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i22 (.D(rgb_values[14]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i22.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i23 (.D(rgb_values[15]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i23.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i24 (.D(rgb_values[16]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i24.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i25 (.D(rgb_values[17]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i25.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i26 (.D(rgb_values[18]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i26.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i27 (.D(rgb_values[19]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i27.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i28 (.D(rgb_values[20]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i28.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i29 (.D(rgb_values[21]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i29.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i30 (.D(rgb_values[22]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i30.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i31 (.D(rgb_values[23]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i31.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i32 (.D(rgb_values[24]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i32.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i33 (.D(rgb_values[25]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i33.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i34 (.D(rgb_values[26]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i34.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i35 (.D(rgb_values[27]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i35.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i36 (.D(rgb_values[28]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i36.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i37 (.D(rgb_values[29]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i37.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i38 (.D(rgb_values[30]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i38.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i39 (.D(rgb_values[31]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i39.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i40 (.D(rgb_values[32]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i40.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i41 (.D(rgb_values[33]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i41.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i42 (.D(rgb_values[34]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i42.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i43 (.D(rgb_values[35]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i43.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i44 (.D(rgb_values[36]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i44.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i45 (.D(rgb_values[37]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i45.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i46 (.D(rgb_values[38]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i46.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i47 (.D(rgb_values[39]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i47.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i48 (.D(rgb_values[40]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i48.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i49 (.D(rgb_values[41]), .SP(spi1_sck_c_enable_226), 
            .CK(spi1_sck_c), .Q(rgb_values[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i49.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i50 (.D(rgb_values[42]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i50.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i51 (.D(rgb_values[43]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i51.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i52 (.D(rgb_values[44]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i52.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i53 (.D(rgb_values[45]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i53.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i54 (.D(rgb_values[46]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i54.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i55 (.D(rgb_values[47]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i55.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i56 (.D(rgb_values[48]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i56.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i57 (.D(rgb_values[49]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i57.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i58 (.D(rgb_values[50]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i58.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i59 (.D(rgb_values[51]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i59.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i60 (.D(rgb_values[52]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i60.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i61 (.D(rgb_values[53]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i61.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i62 (.D(rgb_values[54]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i62.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i63 (.D(rgb_values[55]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i63.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i64 (.D(rgb_values[56]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i64.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i65 (.D(rgb_values[57]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i65.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i66 (.D(rgb_values[58]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i66.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i67 (.D(rgb_values[59]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i67.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i68 (.D(rgb_values[60]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i68.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i69 (.D(rgb_values[61]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i69.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i70 (.D(rgb_values[62]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i70.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i71 (.D(rgb_values[63]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i71.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i72 (.D(rgb_values[64]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i72.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i73 (.D(rgb_values[65]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i73.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i74 (.D(rgb_values[66]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i74.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i75 (.D(rgb_values[67]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i75.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i76 (.D(rgb_values[68]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i76.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i77 (.D(rgb_values[69]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i77.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i78 (.D(rgb_values[70]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i78.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i79 (.D(rgb_values[71]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i79.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i80 (.D(rgb_values[72]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i80.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i81 (.D(rgb_values[73]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i81.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i82 (.D(rgb_values[74]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i82.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i83 (.D(rgb_values[75]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i83.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i84 (.D(rgb_values[76]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i84.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i85 (.D(rgb_values[77]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i85.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i86 (.D(rgb_values[78]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i86.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i87 (.D(rgb_values[79]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i87.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i88 (.D(rgb_values[80]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i88.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i89 (.D(rgb_values[81]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[89])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i89.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i90 (.D(rgb_values[82]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[90])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i90.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i91 (.D(rgb_values[83]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[91])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i91.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i92 (.D(rgb_values[84]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[92])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i92.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i93 (.D(rgb_values[85]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[93])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i93.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i94 (.D(rgb_values[86]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[94])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i94.GSR = "DISABLED";
    FD1P3AX rgb_values_i0_i95 (.D(rgb_values[87]), .SP(spi1_sck_c_enable_272), 
            .CK(spi1_sck_c), .Q(rgb_values[95])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam rgb_values_i0_i95.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i1 (.D(pending_sequence[1]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i2 (.D(pending_sequence[2]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i3 (.D(pending_sequence[3]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i4 (.D(pending_sequence[4]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i5 (.D(pending_sequence[5]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i6 (.D(pending_sequence[6]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i7 (.D(pending_sequence[7]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i8 (.D(pending_sequence[8]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i9 (.D(pending_sequence[9]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i10 (.D(pending_sequence[10]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i11 (.D(pending_sequence[11]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i12 (.D(pending_sequence[12]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i13 (.D(pending_sequence[13]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i14 (.D(pending_sequence[14]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i15 (.D(pending_sequence[15]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i16 (.D(pending_sequence[16]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i17 (.D(pending_sequence[17]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i18 (.D(pending_sequence[18]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i19 (.D(pending_sequence[19]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i20 (.D(pending_sequence[20]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i21 (.D(pending_sequence[21]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i22 (.D(pending_sequence[22]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i23 (.D(pending_sequence[23]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i24 (.D(pending_sequence[24]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i25 (.D(pending_sequence[25]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i26 (.D(pending_sequence[26]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i27 (.D(pending_sequence[27]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i28 (.D(pending_sequence[28]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i29 (.D(pending_sequence[29]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i30 (.D(pending_sequence[30]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_i0_i31 (.D(pending_sequence[31]), .SP(pll_clk_enable_124), 
            .CK(pll_clk), .Q(accepted_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_i0_i31.GSR = "DISABLED";
    FD1S3AX phase_frac_i1 (.D(phase_frac_sum[1]), .CK(pll_clk), .Q(phase_frac[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i1.GSR = "DISABLED";
    FD1S3AX phase_frac_i2 (.D(phase_frac_sum[2]), .CK(pll_clk), .Q(phase_frac[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i2.GSR = "DISABLED";
    FD1S3AX phase_frac_i3 (.D(phase_frac_sum[3]), .CK(pll_clk), .Q(phase_frac[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i3.GSR = "DISABLED";
    FD1S3AX phase_frac_i4 (.D(phase_frac_sum[4]), .CK(pll_clk), .Q(phase_frac[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i4.GSR = "DISABLED";
    FD1S3AX phase_frac_i5 (.D(phase_frac_sum[5]), .CK(pll_clk), .Q(phase_frac[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i5.GSR = "DISABLED";
    FD1S3AX phase_frac_i6 (.D(phase_frac_sum[6]), .CK(pll_clk), .Q(phase_frac[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i6.GSR = "DISABLED";
    FD1S3AX phase_frac_i7 (.D(phase_frac_sum[7]), .CK(pll_clk), .Q(phase_frac[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i7.GSR = "DISABLED";
    FD1S3AX phase_frac_i8 (.D(phase_frac_sum[8]), .CK(pll_clk), .Q(phase_frac[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i8.GSR = "DISABLED";
    FD1S3AX phase_frac_i9 (.D(phase_frac_sum[9]), .CK(pll_clk), .Q(phase_frac[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i9.GSR = "DISABLED";
    FD1S3AX phase_frac_i10 (.D(phase_frac_sum[10]), .CK(pll_clk), .Q(phase_frac[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i10.GSR = "DISABLED";
    FD1S3AX phase_frac_i11 (.D(phase_frac_sum[11]), .CK(pll_clk), .Q(phase_frac[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i11.GSR = "DISABLED";
    FD1S3AX phase_frac_i12 (.D(phase_frac_sum[12]), .CK(pll_clk), .Q(phase_frac[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i12.GSR = "DISABLED";
    FD1S3AX phase_frac_i13 (.D(phase_frac_sum[13]), .CK(pll_clk), .Q(phase_frac[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i13.GSR = "DISABLED";
    FD1S3AX phase_frac_i14 (.D(phase_frac_sum[14]), .CK(pll_clk), .Q(phase_frac[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i14.GSR = "DISABLED";
    FD1S3AX phase_frac_i15 (.D(phase_frac_sum[15]), .CK(pll_clk), .Q(phase_frac[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i15.GSR = "DISABLED";
    FD1S3AX phase_frac_i16 (.D(phase_frac_sum[16]), .CK(pll_clk), .Q(phase_frac[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i16.GSR = "DISABLED";
    FD1S3AX phase_frac_i17 (.D(phase_frac_sum[17]), .CK(pll_clk), .Q(phase_frac[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i17.GSR = "DISABLED";
    FD1S3AX phase_frac_i18 (.D(phase_frac_sum[18]), .CK(pll_clk), .Q(phase_frac[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i18.GSR = "DISABLED";
    FD1S3AX phase_frac_i19 (.D(phase_frac_sum[19]), .CK(pll_clk), .Q(phase_frac[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i19.GSR = "DISABLED";
    FD1S3AX phase_frac_i20 (.D(phase_frac_sum[20]), .CK(pll_clk), .Q(phase_frac[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i20.GSR = "DISABLED";
    FD1S3AX phase_frac_i21 (.D(phase_frac_sum[21]), .CK(pll_clk), .Q(phase_frac[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i21.GSR = "DISABLED";
    FD1S3AX phase_frac_i22 (.D(phase_frac_sum[22]), .CK(pll_clk), .Q(phase_frac[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i22.GSR = "DISABLED";
    FD1S3AX phase_frac_i23 (.D(phase_frac_sum[23]), .CK(pll_clk), .Q(phase_frac[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam phase_frac_i23.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i1 (.D(rgb_values[1]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i1.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i2 (.D(rgb_values[2]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i2.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i3 (.D(rgb_values[3]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i3.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i4 (.D(rgb_values[4]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i4.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i5 (.D(rgb_values[5]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i5.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i6 (.D(rgb_values[6]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i6.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i7 (.D(rgb_values[7]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i7.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i8 (.D(rgb_values[8]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i8.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i9 (.D(rgb_values[9]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i9.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i10 (.D(rgb_values[10]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i10.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i11 (.D(rgb_values[11]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i11.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i12 (.D(rgb_values[12]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i12.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i13 (.D(rgb_values[13]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i13.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i14 (.D(rgb_values[14]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i14.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i15 (.D(rgb_values[15]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i15.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i16 (.D(rgb_values[16]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i16.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i17 (.D(rgb_values[17]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i17.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i18 (.D(rgb_values[18]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i18.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i19 (.D(rgb_values[19]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i19.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i20 (.D(rgb_values[20]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i20.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i21 (.D(rgb_values[21]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i21.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i22 (.D(rgb_values[22]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i22.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i23 (.D(rgb_values[23]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i23.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i24 (.D(rgb_values[24]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i24.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i25 (.D(rgb_values[25]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i25.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i26 (.D(rgb_values[26]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i26.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i27 (.D(rgb_values[27]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i27.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i28 (.D(rgb_values[28]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i28.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i29 (.D(rgb_values[29]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i29.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i30 (.D(rgb_values[30]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i30.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i31 (.D(rgb_values[31]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i31.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i32 (.D(rgb_values[32]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i32.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i33 (.D(rgb_values[33]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i33.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i34 (.D(rgb_values[34]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i34.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i35 (.D(rgb_values[35]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i35.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i36 (.D(rgb_values[36]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i36.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i37 (.D(rgb_values[37]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i37.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i38 (.D(rgb_values[38]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i38.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i39 (.D(rgb_values[39]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i39.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i40 (.D(rgb_values[40]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i40.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i41 (.D(rgb_values[41]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i41.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i42 (.D(rgb_values[42]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i42.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i43 (.D(rgb_values[43]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i43.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i44 (.D(rgb_values[44]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i44.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i45 (.D(rgb_values[45]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i45.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i46 (.D(rgb_values[46]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i46.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i47 (.D(rgb_values[47]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i47.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i48 (.D(rgb_values[48]), .SP(pll_clk_enable_172), 
            .CK(pll_clk), .Q(rgb_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i48.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i49 (.D(rgb_values[49]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i49.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i50 (.D(rgb_values[50]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i50.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i51 (.D(rgb_values[51]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i51.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i52 (.D(rgb_values[52]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i52.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i53 (.D(rgb_values[53]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i53.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i54 (.D(rgb_values[54]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i54.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i55 (.D(rgb_values[55]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i55.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i56 (.D(rgb_values[56]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i56.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i57 (.D(rgb_values[57]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i57.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i58 (.D(rgb_values[58]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i58.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i59 (.D(rgb_values[59]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i59.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i60 (.D(rgb_values[60]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i60.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i61 (.D(rgb_values[61]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i61.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i62 (.D(rgb_values[62]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i62.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i63 (.D(rgb_values[63]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i63.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i64 (.D(rgb_values[64]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i64.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i65 (.D(rgb_values[65]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i65.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i66 (.D(rgb_values[66]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i66.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i67 (.D(rgb_values[67]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i67.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i68 (.D(rgb_values[68]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i68.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i69 (.D(rgb_values[69]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i69.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i70 (.D(rgb_values[70]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i70.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i71 (.D(rgb_values[71]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i71.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i72 (.D(rgb_values[72]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i72.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i73 (.D(rgb_values[73]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i73.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i74 (.D(rgb_values[74]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i74.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i75 (.D(rgb_values[75]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i75.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i76 (.D(rgb_values[76]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i76.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i77 (.D(rgb_values[77]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i77.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i78 (.D(rgb_values[78]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i78.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i79 (.D(rgb_values[79]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i79.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i80 (.D(rgb_values[80]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i80.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i81 (.D(rgb_values[81]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i81.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i82 (.D(rgb_values[82]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i82.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i83 (.D(rgb_values[83]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i83.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i84 (.D(rgb_values[84]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[84])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i84.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i85 (.D(rgb_values[85]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[85])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i85.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i86 (.D(rgb_values[86]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[86])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i86.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i87 (.D(rgb_values[87]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[87])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i87.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i88 (.D(rgb_values[88]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i88.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i89 (.D(rgb_values[89]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[89])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i89.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i90 (.D(rgb_values[90]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[90])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i90.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i91 (.D(rgb_values[91]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[91])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i91.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i92 (.D(rgb_values[92]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[92])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i92.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i93 (.D(rgb_values[93]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[93])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i93.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i94 (.D(rgb_values[94]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[94])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i94.GSR = "DISABLED";
    FD1P3AX rgb_hold_i0_i95 (.D(rgb_values[95]), .SP(pll_clk_enable_219), 
            .CK(pll_clk), .Q(rgb_hold[95])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam rgb_hold_i0_i95.GSR = "DISABLED";
    FD1P3AX spi_byte_count_i0_i1 (.D(spi_byte_count_15__N_1579[1]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i1.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i2 (.D(spi_byte_count_15__N_1579[2]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i2.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i3 (.D(spi_byte_count_15__N_1579[3]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i3.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i4 (.D(spi_byte_count_15__N_1579[4]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i4.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i5 (.D(spi_byte_count_15__N_1579[5]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i5.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i6 (.D(spi_byte_count_15__N_1579[6]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i6.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i7 (.D(spi_byte_count_15__N_1579[7]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i7.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i8 (.D(spi_byte_count_15__N_1579[8]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i8.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i9 (.D(spi_byte_count_15__N_1579[9]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i9.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i10 (.D(spi_byte_count_15__N_1579[10]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i10.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i11 (.D(spi_byte_count_15__N_1579[11]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i11.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i12 (.D(spi_byte_count_15__N_1579[12]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i12.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i13 (.D(spi_byte_count_15__N_1579[13]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i13.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i14 (.D(spi_byte_count_15__N_1579[14]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i14.GSR = "ENABLED";
    FD1P3AX spi_byte_count_i0_i15 (.D(spi_byte_count_15__N_1579[15]), .SP(spi1_sck_c_enable_287), 
            .CK(spi1_sck_c), .Q(spi_byte_count[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam spi_byte_count_i0_i15.GSR = "ENABLED";
    FD1P3AX status_hold__i2 (.D(accepted_sequence[25]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i2.GSR = "DISABLED";
    FD1P3AX status_hold__i3 (.D(accepted_sequence[26]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i3.GSR = "DISABLED";
    FD1P3AX status_hold__i4 (.D(accepted_sequence[27]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i4.GSR = "DISABLED";
    FD1P3AX status_hold__i5 (.D(accepted_sequence[28]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i5.GSR = "DISABLED";
    FD1P3AX status_hold__i6 (.D(accepted_sequence[29]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i6.GSR = "DISABLED";
    FD1P3AX status_hold__i7 (.D(accepted_sequence[30]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i7.GSR = "DISABLED";
    FD1P3AX status_hold__i8 (.D(accepted_sequence[31]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i8.GSR = "DISABLED";
    FD1P3AX status_hold__i9 (.D(accepted_sequence[16]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i9.GSR = "DISABLED";
    FD1P3AX status_hold__i10 (.D(accepted_sequence[17]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i10.GSR = "DISABLED";
    FD1P3AX status_hold__i11 (.D(accepted_sequence[18]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i11.GSR = "DISABLED";
    FD1P3AX status_hold__i12 (.D(accepted_sequence[19]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i12.GSR = "DISABLED";
    FD1P3AX status_hold__i13 (.D(accepted_sequence[20]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i13.GSR = "DISABLED";
    FD1P3AX status_hold__i14 (.D(accepted_sequence[21]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i14.GSR = "DISABLED";
    FD1P3AX status_hold__i15 (.D(accepted_sequence[22]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i15.GSR = "DISABLED";
    FD1P3AX status_hold__i16 (.D(accepted_sequence[23]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i16.GSR = "DISABLED";
    FD1P3AX status_hold__i17 (.D(accepted_sequence[8]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i17.GSR = "DISABLED";
    FD1P3AX status_hold__i18 (.D(accepted_sequence[9]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i18.GSR = "DISABLED";
    FD1P3AX status_hold__i19 (.D(accepted_sequence[10]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i19.GSR = "DISABLED";
    FD1P3AX status_hold__i20 (.D(accepted_sequence[11]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i20.GSR = "DISABLED";
    FD1P3AX status_hold__i21 (.D(accepted_sequence[12]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i21.GSR = "DISABLED";
    FD1P3AX status_hold__i22 (.D(accepted_sequence[13]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i22.GSR = "DISABLED";
    FD1P3AX status_hold__i23 (.D(accepted_sequence[14]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i23.GSR = "DISABLED";
    FD1P3AX status_hold__i24 (.D(accepted_sequence[15]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i24.GSR = "DISABLED";
    FD1P3AX status_hold__i25 (.D(accepted_sequence[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i25.GSR = "DISABLED";
    FD1P3AX status_hold__i26 (.D(accepted_sequence[1]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i26.GSR = "DISABLED";
    FD1P3AX status_hold__i27 (.D(accepted_sequence[2]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i27.GSR = "DISABLED";
    FD1P3AX status_hold__i28 (.D(accepted_sequence[3]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i28.GSR = "DISABLED";
    FD1P3AX status_hold__i29 (.D(accepted_sequence[4]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i29.GSR = "DISABLED";
    FD1P3AX status_hold__i30 (.D(accepted_sequence[5]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i30.GSR = "DISABLED";
    FD1P3AX status_hold__i31 (.D(accepted_sequence[6]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i31.GSR = "DISABLED";
    FD1P3AX status_hold__i32 (.D(accepted_sequence[7]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i32.GSR = "DISABLED";
    FD1P3AX status_hold__i33 (.D(fpga_time[24]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i33.GSR = "DISABLED";
    FD1P3AX status_hold__i34 (.D(fpga_time[25]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i34.GSR = "DISABLED";
    FD1P3AX status_hold__i35 (.D(fpga_time[26]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i35.GSR = "DISABLED";
    FD1P3AX status_hold__i36 (.D(fpga_time[27]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i36.GSR = "DISABLED";
    FD1P3AX status_hold__i37 (.D(fpga_time[28]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i37.GSR = "DISABLED";
    FD1P3AX status_hold__i38 (.D(fpga_time[29]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i38.GSR = "DISABLED";
    FD1P3AX status_hold__i39 (.D(fpga_time[30]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i39.GSR = "DISABLED";
    FD1P3AX status_hold__i40 (.D(fpga_time[31]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i40.GSR = "DISABLED";
    FD1P3AX status_hold__i41 (.D(fpga_time[16]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i41.GSR = "DISABLED";
    FD1P3AX status_hold__i42 (.D(fpga_time[17]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i42.GSR = "DISABLED";
    FD1P3AX status_hold__i43 (.D(fpga_time[18]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i43.GSR = "DISABLED";
    FD1P3AX status_hold__i44 (.D(fpga_time[19]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i44.GSR = "DISABLED";
    FD1P3AX status_hold__i45 (.D(fpga_time[20]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i45.GSR = "DISABLED";
    FD1P3AX status_hold__i46 (.D(fpga_time[21]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i46.GSR = "DISABLED";
    FD1P3AX status_hold__i47 (.D(fpga_time[22]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i47.GSR = "DISABLED";
    FD1P3AX status_hold__i48 (.D(fpga_time[23]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i48.GSR = "DISABLED";
    FD1P3AX status_hold__i49 (.D(fpga_time[8]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i49.GSR = "DISABLED";
    FD1P3AX status_hold__i50 (.D(fpga_time[9]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i50.GSR = "DISABLED";
    FD1P3AX status_hold__i51 (.D(fpga_time[10]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i51.GSR = "DISABLED";
    FD1P3AX status_hold__i52 (.D(fpga_time[11]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i52.GSR = "DISABLED";
    FD1P3AX status_hold__i53 (.D(fpga_time[12]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i53.GSR = "DISABLED";
    FD1P3AX status_hold__i54 (.D(fpga_time[13]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i54.GSR = "DISABLED";
    FD1P3AX status_hold__i55 (.D(fpga_time[14]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i55.GSR = "DISABLED";
    FD1P3AX status_hold__i56 (.D(fpga_time[15]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i56.GSR = "DISABLED";
    FD1P3AX status_hold__i57 (.D(fpga_time[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i57.GSR = "DISABLED";
    FD1P3AX status_hold__i58 (.D(fpga_time[1]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i58.GSR = "DISABLED";
    FD1P3AX status_hold__i59 (.D(fpga_time[2]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i59.GSR = "DISABLED";
    FD1P3AX status_hold__i60 (.D(fpga_time[3]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i60.GSR = "DISABLED";
    FD1P3AX status_hold__i61 (.D(fpga_time[4]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i61.GSR = "DISABLED";
    FD1P3AX status_hold__i62 (.D(fpga_time[5]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i62.GSR = "DISABLED";
    FD1P3AX status_hold__i63 (.D(fpga_time[6]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i63.GSR = "DISABLED";
    FD1P3AX status_hold__i64 (.D(fpga_time[7]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i64.GSR = "DISABLED";
    FD1P3AX status_hold__i65 (.D(status_flags_wire_15__N_1269[2]), .SP(cs_fall), 
            .CK(pll_clk), .Q(status_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i65.GSR = "DISABLED";
    FD1P3AX status_hold__i66 (.D(status_flags_wire_15__N_1285[4]), .SP(cs_fall), 
            .CK(pll_clk), .Q(status_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i66.GSR = "DISABLED";
    FD1P3AX status_hold__i67 (.D(n23157), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[88])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i67.GSR = "DISABLED";
    FD1P3AX status_hold__i68 (.D(fifo_credit_wire[0]), .SP(cs_fall), .CK(pll_clk), 
            .Q(status_hold[104])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam status_hold__i68.GSR = "DISABLED";
    FD1P3IX us_tx__i2 (.D(n10860), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_1)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i2.GSR = "DISABLED";
    LUT4 run_addr_reg_8__I_0_i9_3_lut_3_lut (.A(active_bank), .B(n15_adj_3023), 
         .C(run_addr_reg[8]), .Z(event_rd_addr[8])) /* synthesis lut_function=(A (B (C))+!A ((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(215[56:68])
    defparam run_addr_reg_8__I_0_i9_3_lut_3_lut.init = 16'hd1d1;
    FD1P3IX us_tx__i3 (.D(n10862), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_2)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i3.GSR = "DISABLED";
    FD1P3IX us_tx__i4 (.D(n10864), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_3)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i4.GSR = "DISABLED";
    FD1P3IX us_tx__i5 (.D(n10866), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_4)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i5.GSR = "DISABLED";
    FD1P3IX us_tx__i6 (.D(n10868), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_5)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i6.GSR = "DISABLED";
    FD1P3IX us_tx__i7 (.D(n10870), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_6)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i7.GSR = "DISABLED";
    FD1P3IX us_tx__i8 (.D(n10872), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_7)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i8.GSR = "DISABLED";
    FD1P3IX us_tx__i9 (.D(n10874), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_8)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i9.GSR = "DISABLED";
    FD1P3IX us_tx__i10 (.D(n10876), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_9)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i10.GSR = "DISABLED";
    FD1P3IX us_tx__i11 (.D(n10878), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_10)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i11.GSR = "DISABLED";
    FD1P3IX us_tx__i12 (.D(n10880), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_11)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i12.GSR = "DISABLED";
    FD1P3IX us_tx__i13 (.D(n10882), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_12)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i13.GSR = "DISABLED";
    FD1P3IX us_tx__i14 (.D(n10884), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_13)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i14.GSR = "DISABLED";
    FD1P3IX us_tx__i15 (.D(n10886), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_14)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i15.GSR = "DISABLED";
    FD1P3IX us_tx__i16 (.D(n10888), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_15)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i16.GSR = "DISABLED";
    FD1P3IX us_tx__i17 (.D(n10890), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_16)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i17.GSR = "DISABLED";
    FD1P3IX us_tx__i18 (.D(n10892), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_17)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i18.GSR = "DISABLED";
    FD1P3IX us_tx__i19 (.D(n10894), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_18)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i19.GSR = "DISABLED";
    FD1P3IX us_tx__i20 (.D(n10896), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_19)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i20.GSR = "DISABLED";
    FD1P3IX us_tx__i21 (.D(n10898), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_20)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i21.GSR = "DISABLED";
    FD1P3IX us_tx__i22 (.D(n10900), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_21)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i22.GSR = "DISABLED";
    FD1P3IX us_tx__i23 (.D(n10902), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_22)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i23.GSR = "DISABLED";
    FD1P3IX us_tx__i24 (.D(n10904), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_23)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i24.GSR = "DISABLED";
    FD1P3IX us_tx__i25 (.D(n10906), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_24)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i25.GSR = "DISABLED";
    FD1P3IX us_tx__i26 (.D(n10908), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_25)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i26.GSR = "DISABLED";
    FD1P3IX us_tx__i27 (.D(n10910), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_26)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i27.GSR = "DISABLED";
    FD1P3IX us_tx__i28 (.D(n10912), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_27)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i28.GSR = "DISABLED";
    FD1P3IX us_tx__i29 (.D(n10914), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_28)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i29.GSR = "DISABLED";
    FD1P3IX us_tx__i30 (.D(n10916), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_29)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i30.GSR = "DISABLED";
    FD1P3IX us_tx__i31 (.D(n10918), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_30)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i31.GSR = "DISABLED";
    FD1P3IX us_tx__i32 (.D(n10920), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_31)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i32.GSR = "DISABLED";
    FD1P3IX us_tx__i33 (.D(n10922), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_32)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i33.GSR = "DISABLED";
    FD1P3IX us_tx__i34 (.D(n10924), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_33)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i34.GSR = "DISABLED";
    FD1P3IX us_tx__i35 (.D(n10926), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_34)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i35.GSR = "DISABLED";
    FD1P3IX us_tx__i36 (.D(n10928), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_35)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i36.GSR = "DISABLED";
    FD1P3IX us_tx__i37 (.D(n10930), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_36)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i37.GSR = "DISABLED";
    FD1P3IX us_tx__i38 (.D(n10932), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_37)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i38.GSR = "DISABLED";
    FD1P3IX us_tx__i39 (.D(n10934), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_38)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i39.GSR = "DISABLED";
    FD1P3IX us_tx__i40 (.D(n10936), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_39)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i40.GSR = "DISABLED";
    FD1P3IX us_tx__i41 (.D(n10938), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_40)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i41.GSR = "DISABLED";
    FD1P3IX us_tx__i42 (.D(n10940), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_41)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i42.GSR = "DISABLED";
    FD1P3IX us_tx__i43 (.D(n10942), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_42)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i43.GSR = "DISABLED";
    FD1P3IX us_tx__i44 (.D(n10944), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_43)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i44.GSR = "DISABLED";
    FD1P3IX us_tx__i45 (.D(n10946), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_44)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i45.GSR = "DISABLED";
    FD1P3IX us_tx__i46 (.D(n10948), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_45)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i46.GSR = "DISABLED";
    FD1P3IX us_tx__i47 (.D(n10950), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_46)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i47.GSR = "DISABLED";
    FD1P3IX us_tx__i48 (.D(n10952), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_47)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i48.GSR = "DISABLED";
    FD1P3IX us_tx__i49 (.D(n10954), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_48)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i49.GSR = "DISABLED";
    FD1P3IX us_tx__i50 (.D(n10956), .SP(pll_clk_enable_268), .CD(n23449), 
            .CK(pll_clk), .Q(us_tx_c_49)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i50.GSR = "DISABLED";
    FD1P3IX us_tx__i51 (.D(n10958), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_50)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i51.GSR = "DISABLED";
    FD1P3IX us_tx__i52 (.D(n10960), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_51)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i52.GSR = "DISABLED";
    FD1P3IX us_tx__i53 (.D(n10962), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_52)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i53.GSR = "DISABLED";
    FD1P3IX us_tx__i54 (.D(n10964), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_53)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i54.GSR = "DISABLED";
    FD1P3IX us_tx__i55 (.D(n10966), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_54)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i55.GSR = "DISABLED";
    FD1P3IX us_tx__i56 (.D(n10968), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_55)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i56.GSR = "DISABLED";
    FD1P3IX us_tx__i57 (.D(n10970), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_56)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i57.GSR = "DISABLED";
    FD1P3IX us_tx__i58 (.D(n10972), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_57)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i58.GSR = "DISABLED";
    FD1P3IX us_tx__i59 (.D(n10974), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_58)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i59.GSR = "DISABLED";
    FD1P3IX us_tx__i60 (.D(n10976), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_59)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i60.GSR = "DISABLED";
    FD1P3IX us_tx__i61 (.D(n10978), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_60)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i61.GSR = "DISABLED";
    FD1P3IX us_tx__i62 (.D(n10980), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_61)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i62.GSR = "DISABLED";
    FD1P3IX us_tx__i63 (.D(n10982), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_62)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i63.GSR = "DISABLED";
    FD1P3IX us_tx__i64 (.D(n10984), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_63)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i64.GSR = "DISABLED";
    FD1P3IX us_tx__i65 (.D(n10986), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_64)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i65.GSR = "DISABLED";
    FD1P3IX us_tx__i66 (.D(n10988), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_65)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i66.GSR = "DISABLED";
    FD1P3IX us_tx__i67 (.D(n10990), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_66)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i67.GSR = "DISABLED";
    FD1P3IX us_tx__i68 (.D(n10992), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_67)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i68.GSR = "DISABLED";
    FD1P3IX us_tx__i69 (.D(n10994), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_68)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i69.GSR = "DISABLED";
    FD1P3IX us_tx__i70 (.D(n10996), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_69)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i70.GSR = "DISABLED";
    FD1P3IX us_tx__i71 (.D(n10998), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_70)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i71.GSR = "DISABLED";
    FD1P3IX us_tx__i72 (.D(n11000), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_71)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i72.GSR = "DISABLED";
    FD1P3IX us_tx__i73 (.D(n11002), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_72)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i73.GSR = "DISABLED";
    FD1P3IX us_tx__i74 (.D(n11004), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_73)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i74.GSR = "DISABLED";
    FD1P3IX us_tx__i75 (.D(n11006), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_74)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i75.GSR = "DISABLED";
    FD1P3IX us_tx__i76 (.D(n11008), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_75)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i76.GSR = "DISABLED";
    FD1P3IX us_tx__i77 (.D(n11010), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_76)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i77.GSR = "DISABLED";
    FD1P3IX us_tx__i78 (.D(n11012), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_77)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i78.GSR = "DISABLED";
    FD1P3IX us_tx__i79 (.D(n11014), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_78)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i79.GSR = "DISABLED";
    FD1P3IX us_tx__i80 (.D(n11016), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_79)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i80.GSR = "DISABLED";
    FD1P3IX us_tx__i81 (.D(n11018), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_80)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i81.GSR = "DISABLED";
    FD1P3IX us_tx__i82 (.D(n11020), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_81)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i82.GSR = "DISABLED";
    FD1P3IX us_tx__i83 (.D(n11022), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_82)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i83.GSR = "DISABLED";
    FD1P3IX us_tx__i84 (.D(n11024), .SP(pll_clk_enable_302), .CD(n9487), 
            .CK(pll_clk), .Q(us_tx_c_83)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam us_tx__i84.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i1 (.D(next_phase[1]), .CK(pll_clk), .Q(run_addr_reg[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i1.GSR = "DISABLED";
    LUT4 i2_3_lut_4_lut_adj_80 (.A(status_flags_wire_15__N_1285[4]), .B(pll_locked), 
         .C(pll_clk_enable_1), .D(phase_step_d3), .Z(pll_clk_enable_302)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_adj_80.init = 16'hfff7;
    LUT4 i2_3_lut_rep_99 (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .Z(n23181)) /* synthesis lut_function=(A+(B+(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(387[17:37])
    defparam i2_3_lut_rep_99.init = 16'hfefe;
    LUT4 i1_2_lut_rep_83_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), 
         .C(frame_settle[2]), .D(frame_settle[0]), .Z(pll_clk_enable_599)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(387[17:37])
    defparam i1_2_lut_rep_83_4_lut.init = 16'hfffe;
    LUT4 i5394_2_lut_4_lut (.A(frame_settle[1]), .B(frame_settle[3]), .C(frame_settle[2]), 
         .D(frame_settle[0]), .Z(n14130)) /* synthesis lut_function=(!(A (D)+!A (B (D)+!B ((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(387[17:37])
    defparam i5394_2_lut_4_lut.init = 16'h00fe;
    LUT4 i1_3_lut_4_lut_adj_81 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[37]), 
         .D(ev_bit[37]), .Z(ev_wr_data[37])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_81.init = 16'hddd0;
    LUT4 frame_toggle_sync_I_0_2_lut_rep_100 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(pll_clk_enable_18)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(382[13:51])
    defparam frame_toggle_sync_I_0_2_lut_rep_100.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_82 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[39]), 
         .D(ev_bit[39]), .Z(ev_wr_data[39])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_82.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_83 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n23181), .D(frame_settle[0]), .Z(pll_clk_enable_219)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(382[13:51])
    defparam i1_3_lut_4_lut_adj_83.init = 16'h0900;
    LUT4 mux_961_i4_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[3]), 
         .D(build_phase[3]), .Z(ev_wr_addr_8__N_796[3])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i4_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i1_2_lut_rep_93_3_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[1]), 
         .Z(n23175)) /* synthesis lut_function=((B+!(C))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[55:75])
    defparam i1_2_lut_rep_93_3_lut.init = 16'hdfdf;
    LUT4 i1_2_lut_rep_78_3_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[0]), .D(ev_state[1]), .Z(n23160)) /* synthesis lut_function=((B+!(C (D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[55:75])
    defparam i1_2_lut_rep_78_3_lut_4_lut.init = 16'hdfff;
    LUT4 i10241_2_lut_rep_119 (.A(ev_state[0]), .B(ev_state[1]), .Z(n23201)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10241_2_lut_rep_119.init = 16'heeee;
    LUT4 i2_2_lut_3_lut_4_lut_adj_84 (.A(ev_state[0]), .B(ev_state[1]), 
         .C(ev_state[3]), .D(ev_state[2]), .Z(n21947)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i2_2_lut_3_lut_4_lut_adj_84.init = 16'hfeff;
    LUT4 i2_3_lut_4_lut_adj_85 (.A(ev_state[0]), .B(ev_state[1]), .C(build_sum[8]), 
         .D(n15_adj_3023), .Z(n13915)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i2_3_lut_4_lut_adj_85.init = 16'h0010;
    LUT4 i1385_1_lut (.A(status_bit_index[5]), .Z(spi1_miso_N_2421[5])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i1385_1_lut.init = 16'h5555;
    LUT4 i1_3_lut_4_lut_adj_86 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[71]), 
         .D(ev_bit[71]), .Z(ev_wr_data[71])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_86.init = 16'hddd0;
    LUT4 i13164_2_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), .Z(n19)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i13164_2_lut.init = 16'h6666;
    LUT4 i1298_2_lut_rep_102 (.A(frame_settle[1]), .B(frame_settle[0]), 
         .Z(n23184)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[29:48])
    defparam i1298_2_lut_rep_102.init = 16'heeee;
    FD1P3IX ev_bit_i10 (.D(ev_bit[9]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i10.GSR = "DISABLED";
    FD1P3IX ev_bit_i43 (.D(ev_bit[42]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i43.GSR = "DISABLED";
    LUT4 i1_2_lut_3_lut (.A(frame_settle[1]), .B(frame_settle[0]), .C(frame_settle[2]), 
         .Z(n13719)) /* synthesis lut_function=(A (C)+!A (B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(386[29:48])
    defparam i1_2_lut_3_lut.init = 16'he1e1;
    LUT4 i13167_2_lut_rep_103 (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .Z(n23185)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i13167_2_lut_rep_103.init = 16'h8888;
    LUT4 i11598_1_lut (.A(status_bit_index[1]), .Z(spi1_miso_N_2421[1])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam i11598_1_lut.init = 16'h5555;
    FD1S3AX run_addr_reg_i2 (.D(next_phase[2]), .CK(pll_clk), .Q(run_addr_reg[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i2.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i3 (.D(next_phase[3]), .CK(pll_clk), .Q(run_addr_reg[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i3.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i4 (.D(next_phase[4]), .CK(pll_clk), .Q(run_addr_reg[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i4.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i5 (.D(next_phase[5]), .CK(pll_clk), .Q(run_addr_reg[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i5.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i6 (.D(next_phase[6]), .CK(pll_clk), .Q(run_addr_reg[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i6.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i7 (.D(next_phase[7]), .CK(pll_clk), .Q(run_addr_reg[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i7.GSR = "DISABLED";
    FD1S3AX run_addr_reg_i8 (.D(run_bank), .CK(pll_clk), .Q(run_addr_reg[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam run_addr_reg_i8.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i1 (.D(ev_rd_data[1]), .CK(pll_clk), .Q(ev_run_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i1.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i2 (.D(ev_rd_data[2]), .CK(pll_clk), .Q(ev_run_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i2.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i3 (.D(ev_rd_data[3]), .CK(pll_clk), .Q(ev_run_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i3.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i4 (.D(ev_rd_data[4]), .CK(pll_clk), .Q(ev_run_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i4.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i5 (.D(ev_rd_data[5]), .CK(pll_clk), .Q(ev_run_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i5.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i6 (.D(ev_rd_data[6]), .CK(pll_clk), .Q(ev_run_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i6.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i7 (.D(ev_rd_data[7]), .CK(pll_clk), .Q(ev_run_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i7.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i8 (.D(ev_rd_data[8]), .CK(pll_clk), .Q(ev_run_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i8.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i9 (.D(ev_rd_data[9]), .CK(pll_clk), .Q(ev_run_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i9.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i10 (.D(ev_rd_data[10]), .CK(pll_clk), .Q(ev_run_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i10.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i11 (.D(ev_rd_data[11]), .CK(pll_clk), .Q(ev_run_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i11.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i12 (.D(ev_rd_data[12]), .CK(pll_clk), .Q(ev_run_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i12.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i13 (.D(ev_rd_data[13]), .CK(pll_clk), .Q(ev_run_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i13.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i14 (.D(ev_rd_data[14]), .CK(pll_clk), .Q(ev_run_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i14.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i15 (.D(ev_rd_data[15]), .CK(pll_clk), .Q(ev_run_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i15.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i16 (.D(ev_rd_data[16]), .CK(pll_clk), .Q(ev_run_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i16.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i17 (.D(ev_rd_data[17]), .CK(pll_clk), .Q(ev_run_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i17.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i18 (.D(ev_rd_data[18]), .CK(pll_clk), .Q(ev_run_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i18.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i19 (.D(ev_rd_data[19]), .CK(pll_clk), .Q(ev_run_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i19.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i20 (.D(ev_rd_data[20]), .CK(pll_clk), .Q(ev_run_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i20.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i21 (.D(ev_rd_data[21]), .CK(pll_clk), .Q(ev_run_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i21.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i22 (.D(ev_rd_data[22]), .CK(pll_clk), .Q(ev_run_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i22.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i23 (.D(ev_rd_data[23]), .CK(pll_clk), .Q(ev_run_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i23.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i24 (.D(ev_rd_data[24]), .CK(pll_clk), .Q(ev_run_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i24.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i25 (.D(ev_rd_data[25]), .CK(pll_clk), .Q(ev_run_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i25.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i26 (.D(ev_rd_data[26]), .CK(pll_clk), .Q(ev_run_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i26.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i27 (.D(ev_rd_data[27]), .CK(pll_clk), .Q(ev_run_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i27.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i28 (.D(ev_rd_data[28]), .CK(pll_clk), .Q(ev_run_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i28.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i29 (.D(ev_rd_data[29]), .CK(pll_clk), .Q(ev_run_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i29.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i30 (.D(ev_rd_data[30]), .CK(pll_clk), .Q(ev_run_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i30.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i31 (.D(ev_rd_data[31]), .CK(pll_clk), .Q(ev_run_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i31.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i32 (.D(ev_rd_data[32]), .CK(pll_clk), .Q(ev_run_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i32.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i33 (.D(ev_rd_data[33]), .CK(pll_clk), .Q(ev_run_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i33.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i34 (.D(ev_rd_data[34]), .CK(pll_clk), .Q(ev_run_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i34.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i35 (.D(ev_rd_data[35]), .CK(pll_clk), .Q(ev_run_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i35.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i36 (.D(ev_rd_data[36]), .CK(pll_clk), .Q(ev_run_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i36.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i37 (.D(ev_rd_data[37]), .CK(pll_clk), .Q(ev_run_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i37.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i38 (.D(ev_rd_data[38]), .CK(pll_clk), .Q(ev_run_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i38.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i39 (.D(ev_rd_data[39]), .CK(pll_clk), .Q(ev_run_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i39.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i40 (.D(ev_rd_data[40]), .CK(pll_clk), .Q(ev_run_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i40.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i41 (.D(ev_rd_data[41]), .CK(pll_clk), .Q(ev_run_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i41.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i42 (.D(ev_rd_data[42]), .CK(pll_clk), .Q(ev_run_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i42.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i43 (.D(ev_rd_data[43]), .CK(pll_clk), .Q(ev_run_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i43.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i44 (.D(ev_rd_data[44]), .CK(pll_clk), .Q(ev_run_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i44.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i45 (.D(ev_rd_data[45]), .CK(pll_clk), .Q(ev_run_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i45.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i46 (.D(ev_rd_data[46]), .CK(pll_clk), .Q(ev_run_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i46.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i47 (.D(ev_rd_data[47]), .CK(pll_clk), .Q(ev_run_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i47.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i48 (.D(ev_rd_data[48]), .CK(pll_clk), .Q(ev_run_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i48.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i49 (.D(ev_rd_data[49]), .CK(pll_clk), .Q(ev_run_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i49.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i50 (.D(ev_rd_data[50]), .CK(pll_clk), .Q(ev_run_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i50.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i51 (.D(ev_rd_data[51]), .CK(pll_clk), .Q(ev_run_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i51.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i52 (.D(ev_rd_data[52]), .CK(pll_clk), .Q(ev_run_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i52.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i53 (.D(ev_rd_data[53]), .CK(pll_clk), .Q(ev_run_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i53.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i54 (.D(ev_rd_data[54]), .CK(pll_clk), .Q(ev_run_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i54.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i55 (.D(ev_rd_data[55]), .CK(pll_clk), .Q(ev_run_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i55.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i56 (.D(ev_rd_data[56]), .CK(pll_clk), .Q(ev_run_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i56.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i57 (.D(ev_rd_data[57]), .CK(pll_clk), .Q(ev_run_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i57.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i58 (.D(ev_rd_data[58]), .CK(pll_clk), .Q(ev_run_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i58.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i59 (.D(ev_rd_data[59]), .CK(pll_clk), .Q(ev_run_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i59.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i60 (.D(ev_rd_data[60]), .CK(pll_clk), .Q(ev_run_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i60.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i61 (.D(ev_rd_data[61]), .CK(pll_clk), .Q(ev_run_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i61.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i62 (.D(ev_rd_data[62]), .CK(pll_clk), .Q(ev_run_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i62.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i63 (.D(ev_rd_data[63]), .CK(pll_clk), .Q(ev_run_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i63.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i64 (.D(ev_rd_data[64]), .CK(pll_clk), .Q(ev_run_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i64.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i65 (.D(ev_rd_data[65]), .CK(pll_clk), .Q(ev_run_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i65.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i66 (.D(ev_rd_data[66]), .CK(pll_clk), .Q(ev_run_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i66.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i67 (.D(ev_rd_data[67]), .CK(pll_clk), .Q(ev_run_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i67.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i68 (.D(ev_rd_data[68]), .CK(pll_clk), .Q(ev_run_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i68.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i69 (.D(ev_rd_data[69]), .CK(pll_clk), .Q(ev_run_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i69.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i70 (.D(ev_rd_data[70]), .CK(pll_clk), .Q(ev_run_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i70.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i71 (.D(ev_rd_data[71]), .CK(pll_clk), .Q(ev_run_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i71.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i72 (.D(ev_rd_data[72]), .CK(pll_clk), .Q(ev_run_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i72.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i73 (.D(ev_rd_data[73]), .CK(pll_clk), .Q(ev_run_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i73.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i74 (.D(ev_rd_data[74]), .CK(pll_clk), .Q(ev_run_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i74.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i75 (.D(ev_rd_data[75]), .CK(pll_clk), .Q(ev_run_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i75.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i76 (.D(ev_rd_data[76]), .CK(pll_clk), .Q(ev_run_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i76.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i77 (.D(ev_rd_data[77]), .CK(pll_clk), .Q(ev_run_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i77.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i78 (.D(ev_rd_data[78]), .CK(pll_clk), .Q(ev_run_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i78.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i79 (.D(ev_rd_data[79]), .CK(pll_clk), .Q(ev_run_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i79.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i80 (.D(ev_rd_data[80]), .CK(pll_clk), .Q(ev_run_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i80.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i81 (.D(ev_rd_data[81]), .CK(pll_clk), .Q(ev_run_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i81.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i82 (.D(ev_rd_data[82]), .CK(pll_clk), .Q(ev_run_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i82.GSR = "DISABLED";
    FD1S3AX ev_run_hold_i83 (.D(ev_rd_data[83]), .CK(pll_clk), .Q(ev_run_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_run_hold_i83.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i1 (.D(accepted_sequence_spi[1]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i2 (.D(accepted_sequence_spi[2]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i3 (.D(accepted_sequence_spi[3]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i4 (.D(accepted_sequence_spi[4]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i5 (.D(accepted_sequence_spi[5]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i6 (.D(accepted_sequence_spi[6]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i7 (.D(accepted_sequence_spi[7]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i8 (.D(accepted_sequence_spi[8]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i9 (.D(accepted_sequence_spi[9]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i10 (.D(accepted_sequence_spi[10]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i11 (.D(accepted_sequence_spi[11]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i12 (.D(accepted_sequence_spi[12]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i13 (.D(accepted_sequence_spi[13]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i14 (.D(accepted_sequence_spi[14]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i15 (.D(accepted_sequence_spi[15]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i16 (.D(accepted_sequence_spi[16]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i17 (.D(accepted_sequence_spi[17]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i18 (.D(accepted_sequence_spi[18]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i19 (.D(accepted_sequence_spi[19]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i20 (.D(accepted_sequence_spi[20]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i21 (.D(accepted_sequence_spi[21]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i22 (.D(accepted_sequence_spi[22]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i23 (.D(accepted_sequence_spi[23]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i24 (.D(accepted_sequence_spi[24]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i25 (.D(accepted_sequence_spi[25]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i26 (.D(accepted_sequence_spi[26]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i27 (.D(accepted_sequence_spi[27]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i28 (.D(accepted_sequence_spi[28]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i29 (.D(accepted_sequence_spi[29]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i30 (.D(accepted_sequence_spi[30]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_meta_i31 (.D(accepted_sequence_spi[31]), .CK(pll_clk), 
            .Q(accepted_sequence_meta[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_meta_i31.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i1 (.D(accepted_sequence_meta[1]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i1.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i2 (.D(accepted_sequence_meta[2]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i2.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i3 (.D(accepted_sequence_meta[3]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i3.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i4 (.D(accepted_sequence_meta[4]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i4.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i5 (.D(accepted_sequence_meta[5]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i5.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i6 (.D(accepted_sequence_meta[6]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i6.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i7 (.D(accepted_sequence_meta[7]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i7.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i8 (.D(accepted_sequence_meta[8]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i8.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i9 (.D(accepted_sequence_meta[9]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i9.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i10 (.D(accepted_sequence_meta[10]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i10.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i11 (.D(accepted_sequence_meta[11]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i11.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i12 (.D(accepted_sequence_meta[12]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i12.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i13 (.D(accepted_sequence_meta[13]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i13.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i14 (.D(accepted_sequence_meta[14]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i14.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i15 (.D(accepted_sequence_meta[15]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i15.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i16 (.D(accepted_sequence_meta[16]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i16.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i17 (.D(accepted_sequence_meta[17]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i17.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i18 (.D(accepted_sequence_meta[18]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i18.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i19 (.D(accepted_sequence_meta[19]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i19.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i20 (.D(accepted_sequence_meta[20]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i20.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i21 (.D(accepted_sequence_meta[21]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i21.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i22 (.D(accepted_sequence_meta[22]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i22.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i23 (.D(accepted_sequence_meta[23]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i23.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i24 (.D(accepted_sequence_meta[24]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i24.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i25 (.D(accepted_sequence_meta[25]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i25.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i26 (.D(accepted_sequence_meta[26]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i26.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i27 (.D(accepted_sequence_meta[27]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i27.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i28 (.D(accepted_sequence_meta[28]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i28.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i29 (.D(accepted_sequence_meta[29]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i29.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i30 (.D(accepted_sequence_meta[30]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i30.GSR = "DISABLED";
    FD1S3AX accepted_sequence_sync_i31 (.D(accepted_sequence_meta[31]), .CK(pll_clk), 
            .Q(accepted_sequence_sync[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam accepted_sequence_sync_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i2 (.D(mic_shift_0[0]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i3 (.D(mic_shift_0[1]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i4 (.D(mic_shift_0[2]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i5 (.D(mic_shift_0[3]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i6 (.D(mic_shift_0[4]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i7 (.D(mic_shift_0[5]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i8 (.D(mic_shift_0[6]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i9 (.D(mic_shift_0[7]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i10 (.D(mic_shift_0[8]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i11 (.D(mic_shift_0[9]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i12 (.D(mic_shift_0[10]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i13 (.D(mic_shift_0[11]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i14 (.D(mic_shift_0[12]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_0__i15 (.D(mic_shift_0[13]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_0[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_0__i15.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_87 (.A(ev_run_hold[15]), .B(us_tx_c_15), .C(init_shadow[15]), 
         .D(swap_now_d3), .Z(n10888)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_87.init = 16'h5a66;
    LUT4 i7_4_lut_adj_88 (.A(ev_run_hold[16]), .B(us_tx_c_16), .C(init_shadow[16]), 
         .D(swap_now_d3), .Z(n10890)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_88.init = 16'h5a66;
    PFUMX i14191 (.BLUT(n22825), .ALUT(n22826), .C0(spi1_miso_N_2421[1]), 
          .Z(n22835));
    LUT4 i1387_1_lut (.A(status_bit_index[3]), .Z(spi1_miso_N_2421[3])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i1387_1_lut.init = 16'h5555;
    LUT4 i1388_1_lut (.A(status_bit_index[2]), .Z(spi1_miso_N_2421[2])) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i1388_1_lut.init = 16'h5555;
    LUT4 i4025_4_lut (.A(ev_ch[4]), .B(staging_rd_addr[4]), .C(n15103), 
         .D(n22416), .Z(staging_rd_addr_6__N_785[4])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i4025_4_lut.init = 16'hcac0;
    FD1P3AX build_phase_i1 (.D(staging_q[9]), .SP(pll_clk_enable_333), .CK(pll_clk), 
            .Q(build_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i1.GSR = "DISABLED";
    FD1P3AX build_phase_i2 (.D(staging_q[10]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i2.GSR = "DISABLED";
    FD1P3AX build_phase_i3 (.D(staging_q[11]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i3.GSR = "DISABLED";
    FD1P3AX build_phase_i4 (.D(staging_q[12]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i4.GSR = "DISABLED";
    FD1P3AX build_phase_i5 (.D(staging_q[13]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i5.GSR = "DISABLED";
    FD1P3AX build_phase_i6 (.D(staging_q[14]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i6.GSR = "DISABLED";
    FD1P3AX build_phase_i7 (.D(staging_q[15]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_phase_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i1 (.D(build_sum_8__N_2025[1]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i1.GSR = "DISABLED";
    FD1P3AX build_sum_i2 (.D(build_sum_8__N_2025[2]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i2.GSR = "DISABLED";
    FD1P3AX build_sum_i3 (.D(build_sum_8__N_2025[3]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i3.GSR = "DISABLED";
    FD1P3AX build_sum_i4 (.D(build_sum_8__N_2025[4]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i4.GSR = "DISABLED";
    FD1P3AX build_sum_i5 (.D(build_sum_8__N_2025[5]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i5.GSR = "DISABLED";
    FD1P3AX build_sum_i6 (.D(build_sum_8__N_2025[6]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i6.GSR = "DISABLED";
    FD1P3AX build_sum_i7 (.D(build_sum_8__N_2025[7]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i7.GSR = "DISABLED";
    FD1P3AX build_sum_i8 (.D(build_sum_8__N_2025[8]), .SP(pll_clk_enable_333), 
            .CK(pll_clk), .Q(build_sum[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam build_sum_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i1 (.D(ev_rd_data[1]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i1.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i2 (.D(ev_rd_data[2]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i2.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i3 (.D(ev_rd_data[3]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i3.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i4 (.D(ev_rd_data[4]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i4.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i5 (.D(ev_rd_data[5]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i5.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i6 (.D(ev_rd_data[6]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i6.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i7 (.D(ev_rd_data[7]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i7.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i8 (.D(ev_rd_data[8]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i8.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i9 (.D(ev_rd_data[9]), .SP(pll_clk_enable_382), .CK(pll_clk), 
            .Q(ev_rd_hold[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i9.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i10 (.D(ev_rd_data[10]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i10.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i11 (.D(ev_rd_data[11]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i11.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i12 (.D(ev_rd_data[12]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i12.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i13 (.D(ev_rd_data[13]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i13.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i14 (.D(ev_rd_data[14]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i14.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i15 (.D(ev_rd_data[15]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i15.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i16 (.D(ev_rd_data[16]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i16.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i17 (.D(ev_rd_data[17]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i17.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i18 (.D(ev_rd_data[18]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i18.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i19 (.D(ev_rd_data[19]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i19.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i20 (.D(ev_rd_data[20]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i20.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i21 (.D(ev_rd_data[21]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i21.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i22 (.D(ev_rd_data[22]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i22.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i23 (.D(ev_rd_data[23]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i23.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i24 (.D(ev_rd_data[24]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i24.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i25 (.D(ev_rd_data[25]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i25.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i26 (.D(ev_rd_data[26]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i26.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i27 (.D(ev_rd_data[27]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i27.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i28 (.D(ev_rd_data[28]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i28.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i29 (.D(ev_rd_data[29]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i29.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i30 (.D(ev_rd_data[30]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i30.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i31 (.D(ev_rd_data[31]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i31.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i32 (.D(ev_rd_data[32]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i32.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i33 (.D(ev_rd_data[33]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i33.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i34 (.D(ev_rd_data[34]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i34.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i35 (.D(ev_rd_data[35]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i35.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i36 (.D(ev_rd_data[36]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i36.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i37 (.D(ev_rd_data[37]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i37.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i38 (.D(ev_rd_data[38]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i38.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i39 (.D(ev_rd_data[39]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i39.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i40 (.D(ev_rd_data[40]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i40.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i41 (.D(ev_rd_data[41]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i41.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i42 (.D(ev_rd_data[42]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i42.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i43 (.D(ev_rd_data[43]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i43.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i44 (.D(ev_rd_data[44]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i44.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i45 (.D(ev_rd_data[45]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i45.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i46 (.D(ev_rd_data[46]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i46.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i47 (.D(ev_rd_data[47]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i47.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i48 (.D(ev_rd_data[48]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i48.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i49 (.D(ev_rd_data[49]), .SP(pll_clk_enable_382), 
            .CK(pll_clk), .Q(ev_rd_hold[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i49.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i50 (.D(ev_rd_data[50]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i50.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i51 (.D(ev_rd_data[51]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i51.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i52 (.D(ev_rd_data[52]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i52.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i53 (.D(ev_rd_data[53]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i53.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i54 (.D(ev_rd_data[54]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i54.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i55 (.D(ev_rd_data[55]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i55.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i56 (.D(ev_rd_data[56]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i56.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i57 (.D(ev_rd_data[57]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i57.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i58 (.D(ev_rd_data[58]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i58.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i59 (.D(ev_rd_data[59]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i59.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i60 (.D(ev_rd_data[60]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i60.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i61 (.D(ev_rd_data[61]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i61.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i62 (.D(ev_rd_data[62]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i62.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i63 (.D(ev_rd_data[63]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i63.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i64 (.D(ev_rd_data[64]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i64.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i65 (.D(ev_rd_data[65]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i65.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i66 (.D(ev_rd_data[66]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i66.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i67 (.D(ev_rd_data[67]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i67.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i68 (.D(ev_rd_data[68]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i68.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i69 (.D(ev_rd_data[69]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i69.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i70 (.D(ev_rd_data[70]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i70.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i71 (.D(ev_rd_data[71]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i71.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i72 (.D(ev_rd_data[72]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i72.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i73 (.D(ev_rd_data[73]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i73.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i74 (.D(ev_rd_data[74]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i74.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i75 (.D(ev_rd_data[75]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i75.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i76 (.D(ev_rd_data[76]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i76.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i77 (.D(ev_rd_data[77]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i77.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i78 (.D(ev_rd_data[78]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i78.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i79 (.D(ev_rd_data[79]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i79.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i80 (.D(ev_rd_data[80]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i80.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i81 (.D(ev_rd_data[81]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i81.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i82 (.D(ev_rd_data[82]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i82.GSR = "DISABLED";
    FD1P3AX ev_rd_hold_i83 (.D(ev_rd_data[83]), .SP(pll_clk_enable_416), 
            .CK(pll_clk), .Q(ev_rd_hold[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_rd_hold_i83.GSR = "DISABLED";
    LUT4 i1_2_lut_adj_89 (.A(spi_byte_count[9]), .B(spi_byte_count[11]), 
         .Z(n6_adj_2985)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_adj_89.init = 16'heeee;
    LUT4 i7_4_lut_adj_90 (.A(ev_run_hold[17]), .B(us_tx_c_17), .C(init_shadow[17]), 
         .D(swap_now_d3), .Z(n10892)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_90.init = 16'h5a66;
    LUT4 i13171_2_lut_3_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .Z(n18)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i13171_2_lut_3_lut.init = 16'h7878;
    LUT4 i7_4_lut_adj_91 (.A(ev_run_hold[18]), .B(us_tx_c_18), .C(init_shadow[18]), 
         .D(swap_now_d3), .Z(n10894)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_91.init = 16'h5a66;
    LUT4 i3_4_lut_adj_92 (.A(spi_channel_index[0]), .B(spi_channel_index[1]), 
         .C(spi_channel_index[4]), .D(spi_channel_index[6]), .Z(n21880)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i3_4_lut_adj_92.init = 16'h8000;
    LUT4 i7_4_lut_adj_93 (.A(n13), .B(spi_byte_count[3]), .C(n23190), 
         .D(n23145), .Z(spi1_sck_c_enable_83)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i7_4_lut_adj_93.init = 16'h0008;
    LUT4 i2_2_lut_rep_85_3_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(spi_bit_count[2]), .Z(spi1_sck_c_enable_287)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i2_2_lut_rep_85_3_lut.init = 16'h8080;
    LUT4 i1_2_lut_adj_94 (.A(spi_channel_index[3]), .B(spi_channel_index[5]), 
         .Z(n5_adj_3022)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(311[43:69])
    defparam i1_2_lut_adj_94.init = 16'heeee;
    LUT4 i1_2_lut_rep_70_3_lut_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(n23186), .D(spi_bit_count[2]), .Z(n23152)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i1_2_lut_rep_70_3_lut_4_lut.init = 16'h0800;
    LUT4 mux_961_i5_3_lut_4_lut (.A(ev_state[0]), .B(n23175), .C(build_sum[4]), 
         .D(build_phase[4]), .Z(ev_wr_addr_8__N_796[4])) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam mux_961_i5_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i14264_2_lut_3_lut (.A(spi_byte_count[0]), .B(n1), .C(spi_byte_count[1]), 
         .Z(spi1_sck_c_enable_59)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i14264_2_lut_3_lut.init = 16'h4040;
    LUT4 i1_2_lut_rep_73_3_lut_4_lut (.A(spi_bit_count[1]), .B(spi_bit_count[0]), 
         .C(fpga_cs_n_c), .D(spi_bit_count[2]), .Z(n23155)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i1_2_lut_rep_73_3_lut_4_lut.init = 16'h0800;
    LUT4 i1_3_lut_adj_95 (.A(n13915), .B(init_shadow[0]), .C(ev_bit[0]), 
         .Z(n10843)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_95.init = 16'hecec;
    LUT4 i10239_2_lut_rep_104 (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .Z(n23186)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i10239_2_lut_rep_104.init = 16'heeee;
    LUT4 i1_3_lut_4_lut_adj_96 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[40]), 
         .D(ev_bit[40]), .Z(ev_wr_data[40])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_96.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_97 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[72]), 
         .D(ev_bit[72]), .Z(ev_wr_data[72])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_97.init = 16'hddd0;
    LUT4 i1_3_lut_rep_63_4_lut (.A(spi_byte_count[7]), .B(spi_byte_count[6]), 
         .C(n22477), .D(spi_byte_count[8]), .Z(n23145)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_3_lut_rep_63_4_lut.init = 16'hfffe;
    LUT4 i7_4_lut_adj_98 (.A(ev_run_hold[19]), .B(us_tx_c_19), .C(init_shadow[19]), 
         .D(swap_now_d3), .Z(n10896)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_98.init = 16'h5a66;
    LUT4 i7_4_lut_adj_99 (.A(ev_run_hold[20]), .B(us_tx_c_20), .C(init_shadow[20]), 
         .D(swap_now_d3), .Z(n10898)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_99.init = 16'h5a66;
    LUT4 i1_4_lut_adj_100 (.A(ev_state[3]), .B(ev_state[1]), .C(ev_state[2]), 
         .D(n15_adj_3023), .Z(n21)) /* synthesis lut_function=(!(A+!(B+!((D)+!C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i1_4_lut_adj_100.init = 16'h4454;
    LUT4 i7_4_lut_adj_101 (.A(ev_run_hold[21]), .B(us_tx_c_21), .C(init_shadow[21]), 
         .D(swap_now_d3), .Z(n10900)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_101.init = 16'h5a66;
    LUT4 i2_3_lut_rep_105 (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_bit_index[1]), .Z(n23187)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam i2_3_lut_rep_105.init = 16'h8080;
    LUT4 i1_2_lut_4_lut (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_bit_index[1]), .D(status_bit_index[6]), .Z(n6_adj_2986)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam i1_2_lut_4_lut.init = 16'h8000;
    LUT4 active_bank_I_0_432_2_lut (.A(active_bank), .B(swap_now_d1), .Z(active_bank_N_795)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(400[18] 405[12])
    defparam active_bank_I_0_432_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_102 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[41]), 
         .D(ev_bit[41]), .Z(ev_wr_data[41])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_102.init = 16'hddd0;
    LUT4 i5_4_lut (.A(spi_byte_count[1]), .B(spi1_sck_c_enable_287), .C(spi_byte_count[0]), 
         .D(spi_byte_count[5]), .Z(n13)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i5_4_lut.init = 16'h0040;
    FD1S3AX staging_rd_addr_i1 (.D(staging_rd_addr_6__N_785[1]), .CK(pll_clk), 
            .Q(staging_rd_addr[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam staging_rd_addr_i1.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i2 (.D(staging_rd_addr_6__N_785[2]), .CK(pll_clk), 
            .Q(staging_rd_addr[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam staging_rd_addr_i2.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i3 (.D(staging_rd_addr_6__N_785[3]), .CK(pll_clk), 
            .Q(staging_rd_addr[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam staging_rd_addr_i3.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i4 (.D(staging_rd_addr_6__N_785[4]), .CK(pll_clk), 
            .Q(staging_rd_addr[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam staging_rd_addr_i4.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i5 (.D(staging_rd_addr_6__N_785[5]), .CK(pll_clk), 
            .Q(staging_rd_addr[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam staging_rd_addr_i5.GSR = "DISABLED";
    FD1S3AX staging_rd_addr_i6 (.D(staging_rd_addr_6__N_785[6]), .CK(pll_clk), 
            .Q(staging_rd_addr[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam staging_rd_addr_i6.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_103 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[73]), 
         .D(ev_bit[73]), .Z(ev_wr_data[73])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_103.init = 16'hddd0;
    FD1P3AX pending_sequence_i1 (.D(accepted_sequence_sync[1]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i1.GSR = "DISABLED";
    FD1P3AX pending_sequence_i2 (.D(accepted_sequence_sync[2]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i2.GSR = "DISABLED";
    FD1P3AX pending_sequence_i3 (.D(accepted_sequence_sync[3]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i3.GSR = "DISABLED";
    FD1P3AX pending_sequence_i4 (.D(accepted_sequence_sync[4]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i4.GSR = "DISABLED";
    FD1P3AX pending_sequence_i5 (.D(accepted_sequence_sync[5]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i5.GSR = "DISABLED";
    FD1P3AX pending_sequence_i6 (.D(accepted_sequence_sync[6]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i6.GSR = "DISABLED";
    FD1P3AX pending_sequence_i7 (.D(accepted_sequence_sync[7]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i7.GSR = "DISABLED";
    FD1P3AX pending_sequence_i8 (.D(accepted_sequence_sync[8]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i8.GSR = "DISABLED";
    FD1P3AX pending_sequence_i9 (.D(accepted_sequence_sync[9]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i9.GSR = "DISABLED";
    FD1P3AX pending_sequence_i10 (.D(accepted_sequence_sync[10]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i10.GSR = "DISABLED";
    FD1P3AX pending_sequence_i11 (.D(accepted_sequence_sync[11]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i11.GSR = "DISABLED";
    FD1P3AX pending_sequence_i12 (.D(accepted_sequence_sync[12]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i12.GSR = "DISABLED";
    FD1P3AX pending_sequence_i13 (.D(accepted_sequence_sync[13]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i13.GSR = "DISABLED";
    FD1P3AX pending_sequence_i14 (.D(accepted_sequence_sync[14]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i14.GSR = "DISABLED";
    FD1P3AX pending_sequence_i15 (.D(accepted_sequence_sync[15]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i15.GSR = "DISABLED";
    FD1P3AX pending_sequence_i16 (.D(accepted_sequence_sync[16]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i16.GSR = "DISABLED";
    FD1P3AX pending_sequence_i17 (.D(accepted_sequence_sync[17]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i17.GSR = "DISABLED";
    FD1P3AX pending_sequence_i18 (.D(accepted_sequence_sync[18]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i18.GSR = "DISABLED";
    FD1P3AX pending_sequence_i19 (.D(accepted_sequence_sync[19]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i19.GSR = "DISABLED";
    FD1P3AX pending_sequence_i20 (.D(accepted_sequence_sync[20]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i20.GSR = "DISABLED";
    FD1P3AX pending_sequence_i21 (.D(accepted_sequence_sync[21]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i21.GSR = "DISABLED";
    FD1P3AX pending_sequence_i22 (.D(accepted_sequence_sync[22]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i22.GSR = "DISABLED";
    FD1P3AX pending_sequence_i23 (.D(accepted_sequence_sync[23]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i23.GSR = "DISABLED";
    FD1P3AX pending_sequence_i24 (.D(accepted_sequence_sync[24]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i24.GSR = "DISABLED";
    FD1P3AX pending_sequence_i25 (.D(accepted_sequence_sync[25]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i25.GSR = "DISABLED";
    FD1P3AX pending_sequence_i26 (.D(accepted_sequence_sync[26]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i26.GSR = "DISABLED";
    FD1P3AX pending_sequence_i27 (.D(accepted_sequence_sync[27]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i27.GSR = "DISABLED";
    FD1P3AX pending_sequence_i28 (.D(accepted_sequence_sync[28]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i28.GSR = "DISABLED";
    FD1P3AX pending_sequence_i29 (.D(accepted_sequence_sync[29]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i29.GSR = "DISABLED";
    FD1P3AX pending_sequence_i30 (.D(accepted_sequence_sync[30]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i30.GSR = "DISABLED";
    FD1P3AX pending_sequence_i31 (.D(accepted_sequence_sync[31]), .SP(pll_clk_enable_447), 
            .CK(pll_clk), .Q(pending_sequence[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam pending_sequence_i31.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i2 (.D(mic_shift_1[0]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i2.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i3 (.D(mic_shift_1[1]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i3.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i4 (.D(mic_shift_1[2]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i4.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i5 (.D(mic_shift_1[3]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i5.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i6 (.D(mic_shift_1[4]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i6.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i7 (.D(mic_shift_1[5]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i7.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i8 (.D(mic_shift_1[6]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i8.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i9 (.D(mic_shift_1[7]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i9.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i10 (.D(mic_shift_1[8]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i10.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i11 (.D(mic_shift_1[9]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i11.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i12 (.D(mic_shift_1[10]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i12.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i13 (.D(mic_shift_1[11]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i13.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i14 (.D(mic_shift_1[12]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i14.GSR = "DISABLED";
    FD1P3AX mic_shift_1__i15 (.D(mic_shift_1[13]), .SP(pll_clk_enable_461), 
            .CK(pll_clk), .Q(mic_shift_1[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_shift_1__i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1[0]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i1.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1[1]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i2.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1[2]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i3.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1[3]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i4.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1[4]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i5.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1[5]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i6.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1[6]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i7.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1[7]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i8.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1[8]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i9.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1[9]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i10.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1[10]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i11.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1[11]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i12.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1[12]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i13.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1[13]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i14.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1[14]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i15.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_data_0_c), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i16.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_0[0]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i17.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_0[1]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i18.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_0[2]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i19.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_0[3]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i20.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_0[4]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i21.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_0[5]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i22.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_0[6]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i23.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_0[7]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i24.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_0[8]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i25.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_0[9]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i26.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_0[10]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i27.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_0[11]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i28.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_0[12]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i29.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_0[13]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i30.GSR = "DISABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_0[14]), .SP(pll_clk_enable_492), 
            .CK(pll_clk), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mic_latest_i0_i31.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i1.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i2.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i3.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i4.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i5.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i6.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i7.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i8.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[9]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i9.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i10.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[11]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i11.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i12.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[13]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i13.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i14.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i15.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i16 (.D(spi_frame_sequence[16]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i16.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i17 (.D(spi_frame_sequence[17]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[17]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i17.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i18 (.D(spi_frame_sequence[18]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i18.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i19 (.D(spi_frame_sequence[19]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[19]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i19.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i20 (.D(spi_frame_sequence[20]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i20.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i21 (.D(spi_frame_sequence[21]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[21]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i21.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i22 (.D(spi_frame_sequence[22]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i22.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i23 (.D(spi_frame_sequence[23]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[23]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i23.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i24 (.D(spi_frame_sequence[24]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i24.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i25 (.D(spi_frame_sequence[25]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[25]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i25.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i26 (.D(spi_frame_sequence[26]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[26]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i26.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i27 (.D(spi_frame_sequence[27]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[27]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i27.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i28 (.D(spi_frame_sequence[28]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[28]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i28.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i29 (.D(spi_frame_sequence[29]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[29]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i29.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i30 (.D(spi_frame_sequence[30]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[30]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i30.GSR = "DISABLED";
    FD1P3AX accepted_sequence_spi_i0_i31 (.D(spi_frame_sequence[31]), .SP(spi1_sck_c_enable_318), 
            .CK(spi1_sck_c), .Q(accepted_sequence_spi[31]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam accepted_sequence_spi_i0_i31.GSR = "DISABLED";
    spi_mic_stream mic_stream_i (.mic_latest({mic_latest}), .sck_N_2937(sck_N_2937), 
            .spi_mic_cs_n_c(spi_mic_cs_n_c), .spi_mic_miso_c(spi_mic_miso_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(522[20] 525[6])
    FD1P3AX global_phase_i0_i1 (.D(next_phase[1]), .SP(phase_step_reg), 
            .CK(pll_clk), .Q(global_phase[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i1.GSR = "DISABLED";
    LUT4 i42_4_lut (.A(ev_state[3]), .B(ev_state_3__N_1900[2]), .C(ev_state[1]), 
         .D(n23194), .Z(n23)) /* synthesis lut_function=(!(A (B (C))+!A (B (C+!(D))+!B !(C+(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i42_4_lut.init = 16'h3f3a;
    LUT4 i7_4_lut_adj_104 (.A(ev_run_hold[22]), .B(us_tx_c_22), .C(init_shadow[22]), 
         .D(swap_now_d3), .Z(n10902)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_104.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_105 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[42]), 
         .D(ev_bit[42]), .Z(ev_wr_data[42])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_105.init = 16'hddd0;
    LUT4 i7_4_lut_adj_106 (.A(ev_run_hold[23]), .B(us_tx_c_23), .C(init_shadow[23]), 
         .D(swap_now_d3), .Z(n10904)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_106.init = 16'h5a66;
    LUT4 i14270_3_lut (.A(n22369), .B(spi_byte_count[5]), .C(spi_byte_count[4]), 
         .Z(spi1_sck_c_enable_27)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i14270_3_lut.init = 16'h1010;
    LUT4 i1_3_lut_4_lut_adj_107 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[74]), 
         .D(ev_bit[74]), .Z(ev_wr_data[74])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_107.init = 16'hddd0;
    LUT4 mux_962_i1_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[0]), 
         .D(ev_clear_addr[0]), .Z(ev_wr_addr[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i1_3_lut_adj_108 (.A(n13915), .B(init_shadow[53]), .C(ev_bit[53]), 
         .Z(n12486)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_108.init = 16'hecec;
    LUT4 i1_3_lut_adj_109 (.A(n13915), .B(init_shadow[52]), .C(ev_bit[52]), 
         .Z(n12480)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_109.init = 16'hecec;
    LUT4 i1_3_lut_adj_110 (.A(n13915), .B(init_shadow[51]), .C(ev_bit[51]), 
         .Z(n12474)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_110.init = 16'hecec;
    LUT4 i2_2_lut_3_lut (.A(spi_byte_count[7]), .B(spi_byte_count[6]), .C(n22580), 
         .Z(n7_adj_3024)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i2_2_lut_3_lut.init = 16'h0808;
    CCU2D fpga_time_1091_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21847), .COUT(n21848), .S0(n160), .S1(n159));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_7.INJECT1_1 = "NO";
    CCU2D add_491_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21810), .S1(phase_frac_sum[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_1.INIT0 = 16'hF000;
    defparam add_491_1.INIT1 = 16'h5555;
    defparam add_491_1.INJECT1_0 = "NO";
    defparam add_491_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_rep_107 (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .Z(n23189)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_rep_107.init = 16'h8888;
    LUT4 i7_4_lut_adj_111 (.A(ev_run_hold[24]), .B(us_tx_c_24), .C(init_shadow[24]), 
         .D(swap_now_d3), .Z(n10906)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_111.init = 16'h5a66;
    LUT4 i68_3_lut_4_lut (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[3]), .Z(n55)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C+(D)))+!A !(C+(D)))) */ ;
    defparam i68_3_lut_4_lut.init = 16'h7ff0;
    LUT4 i2_2_lut_3_lut_adj_112 (.A(spi_byte_count[1]), .B(spi_byte_count[0]), 
         .C(n1), .Z(spi1_sck_c_enable_67)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i2_2_lut_3_lut_adj_112.init = 16'h8080;
    CCU2D add_242_9 (.A0(ev_clear_addr[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21809), .S0(ev_clear_addr_7__N_2209[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_9.INIT0 = 16'h5aaa;
    defparam add_242_9.INIT1 = 16'h0000;
    defparam add_242_9.INJECT1_0 = "NO";
    defparam add_242_9.INJECT1_1 = "NO";
    LUT4 ev_state_1__bdd_4_lut (.A(ev_state[1]), .B(ev_state[2]), .C(ev_state[0]), 
         .D(ev_state[3]), .Z(pll_clk_enable_416)) /* synthesis lut_function=(!(A+(B ((D)+!C)+!B (C+!(D))))) */ ;
    defparam ev_state_1__bdd_4_lut.init = 16'h0140;
    LUT4 i1_2_lut_rep_108 (.A(spi_byte_count[2]), .B(spi_byte_count[4]), 
         .Z(n23190)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_108.init = 16'heeee;
    LUT4 i14267_3_lut (.A(n22369), .B(spi_byte_count[5]), .C(spi_byte_count[4]), 
         .Z(spi1_sck_c_enable_52)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i14267_3_lut.init = 16'h0404;
    LUT4 i3_4_lut_adj_113 (.A(n23155), .B(n23173), .C(n23205), .D(n22732), 
         .Z(spi_write)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i3_4_lut_adj_113.init = 16'h0080;
    LUT4 i1_3_lut_adj_114 (.A(n13915), .B(init_shadow[50]), .C(ev_bit[50]), 
         .Z(n12468)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_114.init = 16'hecec;
    FD1P3AX global_phase_i0_i2 (.D(next_phase[2]), .SP(phase_step_reg), 
            .CK(pll_clk), .Q(global_phase[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i2.GSR = "DISABLED";
    FD1P3AX global_phase_i0_i3 (.D(next_phase[3]), .SP(phase_step_reg), 
            .CK(pll_clk), .Q(global_phase[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i3.GSR = "DISABLED";
    FD1P3AX global_phase_i0_i4 (.D(next_phase[4]), .SP(phase_step_reg), 
            .CK(pll_clk), .Q(global_phase[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i4.GSR = "DISABLED";
    FD1P3AX global_phase_i0_i5 (.D(next_phase[5]), .SP(phase_step_reg), 
            .CK(pll_clk), .Q(global_phase[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i5.GSR = "DISABLED";
    FD1P3AX global_phase_i0_i6 (.D(next_phase[6]), .SP(phase_step_reg), 
            .CK(pll_clk), .Q(global_phase[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i6.GSR = "DISABLED";
    FD1P3AX global_phase_i0_i7 (.D(next_phase[7]), .SP(phase_step_reg), 
            .CK(pll_clk), .Q(global_phase[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam global_phase_i0_i7.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_115 (.A(ev_run_hold[25]), .B(us_tx_c_25), .C(init_shadow[25]), 
         .D(swap_now_d3), .Z(n10908)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_115.init = 16'h5a66;
    LUT4 i1159_2_lut_3_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[5]), .D(spi_byte_count[3]), .Z(n12)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (C (D)))) */ ;
    defparam i1159_2_lut_3_lut_4_lut.init = 16'hf0e0;
    LUT4 i1_3_lut_adj_116 (.A(n13915), .B(init_shadow[49]), .C(ev_bit[49]), 
         .Z(n12462)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_116.init = 16'hecec;
    LUT4 run_addr_reg_8__I_0_i3_3_lut (.A(run_addr_reg[2]), .B(ev_rd_slot[2]), 
         .C(n15_adj_3023), .Z(event_rd_addr[2])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i3_3_lut.init = 16'hacac;
    LUT4 i7_4_lut_adj_117 (.A(ev_run_hold[26]), .B(us_tx_c_26), .C(init_shadow[26]), 
         .D(swap_now_d3), .Z(n10910)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_117.init = 16'h5a66;
    LUT4 i2_2_lut_rep_86_3_lut (.A(spi_byte_count[2]), .B(spi_byte_count[4]), 
         .C(spi_byte_count[3]), .Z(n23168)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_2_lut_rep_86_3_lut.init = 16'hfefe;
    LUT4 i1_4_lut_then_4_lut (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n23207)) /* synthesis lut_function=(!((B (D)+!B !(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(126[17:33])
    defparam i1_4_lut_then_4_lut.init = 16'h2088;
    LUT4 mux_962_i2_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[1]), 
         .D(ev_clear_addr[1]), .Z(ev_wr_addr[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i2_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i7_4_lut_adj_118 (.A(ev_run_hold[27]), .B(us_tx_c_27), .C(init_shadow[27]), 
         .D(swap_now_d3), .Z(n10912)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_118.init = 16'h5a66;
    LUT4 run_addr_reg_8__I_0_i7_3_lut (.A(run_addr_reg[6]), .B(ev_rd_slot[6]), 
         .C(n15_adj_3023), .Z(event_rd_addr[6])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i7_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_119 (.A(n13915), .B(init_shadow[48]), .C(ev_bit[48]), 
         .Z(n12456)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_119.init = 16'hecec;
    LUT4 mux_962_i3_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[2]), 
         .D(ev_clear_addr[2]), .Z(ev_wr_addr[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i7_4_lut_adj_120 (.A(ev_run_hold[28]), .B(us_tx_c_28), .C(init_shadow[28]), 
         .D(swap_now_d3), .Z(n10914)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_120.init = 16'h5a66;
    LUT4 i4013_4_lut (.A(ev_ch[1]), .B(staging_rd_addr[1]), .C(n15103), 
         .D(n22416), .Z(staging_rd_addr_6__N_785[1])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i4013_4_lut.init = 16'hcac0;
    LUT4 i7_4_lut_adj_121 (.A(ev_run_hold[29]), .B(us_tx_c_29), .C(init_shadow[29]), 
         .D(swap_now_d3), .Z(n10916)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_121.init = 16'h5a66;
    LUT4 mux_962_i4_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[3]), 
         .D(ev_clear_addr[3]), .Z(ev_wr_addr[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i36_3_lut_3_lut (.A(spi_byte_count[2]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[4]), .Z(n25)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A (B+!(C)))) */ ;
    defparam i36_3_lut_3_lut.init = 16'h3838;
    LUT4 i7_4_lut_adj_122 (.A(ev_run_hold[30]), .B(us_tx_c_30), .C(init_shadow[30]), 
         .D(swap_now_d3), .Z(n10918)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_122.init = 16'h5a66;
    LUT4 i4_4_lut_adj_123 (.A(n91_adj_2988), .B(n23155), .C(spi_byte_count[8]), 
         .D(n22538), .Z(n22424)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i4_4_lut_adj_123.init = 16'h0008;
    LUT4 i112_4_lut (.A(spi_byte_count[7]), .B(n21924), .C(spi_byte_count[6]), 
         .D(n12), .Z(n91_adj_2988)) /* synthesis lut_function=(!(A (B (C))+!A !(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i112_4_lut.init = 16'h7f7a;
    LUT4 mux_962_i5_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[4]), 
         .D(ev_clear_addr[4]), .Z(ev_wr_addr[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i5_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i13895_2_lut (.A(n22477), .B(n13906), .Z(n22538)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i13895_2_lut.init = 16'heeee;
    LUT4 i7_4_lut_adj_124 (.A(ev_run_hold[31]), .B(us_tx_c_31), .C(init_shadow[31]), 
         .D(swap_now_d3), .Z(n10920)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_124.init = 16'h5a66;
    LUT4 i7_4_lut_adj_125 (.A(ev_run_hold[32]), .B(us_tx_c_32), .C(init_shadow[32]), 
         .D(swap_now_d3), .Z(n10922)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_125.init = 16'h5a66;
    LUT4 mux_962_i6_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[5]), 
         .D(ev_clear_addr[5]), .Z(ev_wr_addr[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i6_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_962_i7_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[6]), 
         .D(ev_clear_addr[6]), .Z(ev_wr_addr[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i7_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i2_3_lut_4_lut_adj_126 (.A(expected_next_15__N_1301[7]), .B(spi_channel_field[1]), 
         .C(spi_channel_field[0]), .D(n22424), .Z(spi1_sck_c_enable_90)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;
    defparam i2_3_lut_4_lut_adj_126.init = 16'h0200;
    CCU2D expected_next_15__I_0_488_5 (.A0(expected_next_15__N_1301[7]), .B0(spi_extension_length[5]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1301[7]), 
          .B1(spi_extension_length[6]), .C1(GND_net), .D1(GND_net), .CIN(n21831), 
          .COUT(n21832), .S0(expected_next[5]), .S1(expected_next[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_5.INIT0 = 16'ha999;
    defparam expected_next_15__I_0_488_5.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_488_5.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_5.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_127 (.A(n60), .B(n23145), .C(n13910), .D(n23196), 
         .Z(n13906)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(291[17] 323[24])
    defparam i1_4_lut_adj_127.init = 16'h3032;
    LUT4 i1_2_lut_rep_91_3_lut (.A(expected_next_15__N_1301[7]), .B(spi_channel_field[1]), 
         .C(spi_channel_field[0]), .Z(n23173)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;
    defparam i1_2_lut_rep_91_3_lut.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_4_lut_adj_128 (.A(expected_next_15__N_1301[7]), .B(spi_channel_field[1]), 
         .C(n22424), .D(spi_channel_field[0]), .Z(spi1_sck_c_enable_324)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_128.init = 16'h2000;
    LUT4 i7_4_lut_adj_129 (.A(ev_run_hold[33]), .B(us_tx_c_33), .C(init_shadow[33]), 
         .D(swap_now_d3), .Z(n10924)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_129.init = 16'h5a66;
    LUT4 i7_4_lut_adj_130 (.A(ev_run_hold[34]), .B(us_tx_c_34), .C(init_shadow[34]), 
         .D(swap_now_d3), .Z(n10926)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_130.init = 16'h5a66;
    FD1P3IX ev_bit_i42 (.D(ev_bit[41]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i42.GSR = "DISABLED";
    LUT4 mux_962_i8_3_lut_4_lut (.A(ev_state[0]), .B(n23163), .C(ev_wr_addr_8__N_796[7]), 
         .D(ev_clear_addr[7]), .Z(ev_wr_addr[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam mux_962_i8_3_lut_4_lut.init = 16'hf2d0;
    PFUMX i14370 (.BLUT(n23202), .ALUT(n23203), .C0(ev_state[0]), .Z(pll_clk_enable_721));
    LUT4 i1_3_lut_4_lut_adj_131 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[48]), 
         .D(ev_bit[48]), .Z(ev_wr_data[48])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_131.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_132 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[49]), 
         .D(ev_bit[49]), .Z(ev_wr_data[49])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_132.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_133 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[50]), 
         .D(ev_bit[50]), .Z(ev_wr_data[50])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_133.init = 16'hddd0;
    LUT4 i1_4_lut_adj_134 (.A(n22756), .B(spi1_sck_c_enable_30), .C(spi_command[1]), 
         .D(n23153), .Z(spi1_sck_c_enable_29)) /* synthesis lut_function=(A (B)+!A (B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i1_4_lut_adj_134.init = 16'hccc8;
    LUT4 i13194_2_lut_rep_110 (.A(mic_clk_c), .B(mic_sample_count[0]), .Z(n23192)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam i13194_2_lut_rep_110.init = 16'h4444;
    FD1P3IX ev_bit_i9 (.D(ev_bit[8]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i9.GSR = "DISABLED";
    FD1P3IX ev_bit_i8 (.D(ev_bit[7]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i8.GSR = "DISABLED";
    FD1P3IX ev_bit_i7 (.D(ev_bit[6]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i7.GSR = "DISABLED";
    FD1P3IX ev_bit_i6 (.D(ev_bit[5]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i6.GSR = "DISABLED";
    FD1P3IX ev_bit_i5 (.D(ev_bit[4]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i5.GSR = "DISABLED";
    FD1P3IX ev_bit_i4 (.D(ev_bit[3]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i4.GSR = "DISABLED";
    FD1P3IX ev_bit_i3 (.D(ev_bit[2]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i3.GSR = "DISABLED";
    FD1P3IX ev_bit_i41 (.D(ev_bit[40]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i41.GSR = "DISABLED";
    LUT4 i13200_2_lut_3_lut (.A(mic_clk_c), .B(mic_sample_count[0]), .C(mic_sample_count[1]), 
         .Z(n29)) /* synthesis lut_function=(A (C)+!A !(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam i13200_2_lut_3_lut.init = 16'hb4b4;
    LUT4 i14113_4_lut (.A(n22680), .B(spi_command[0]), .C(n7), .D(spi_command[7]), 
         .Z(n22756)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;
    defparam i14113_4_lut.init = 16'hfffb;
    LUT4 i13207_2_lut_3_lut_4_lut (.A(mic_clk_c), .B(mic_sample_count[0]), 
         .C(mic_sample_count[2]), .D(mic_sample_count[1]), .Z(n28)) /* synthesis lut_function=(A (C)+!A !(B (C (D)+!C !(D))+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam i13207_2_lut_3_lut_4_lut.init = 16'hb4f0;
    LUT4 i13203_2_lut_rep_87_3_lut (.A(mic_clk_c), .B(mic_sample_count[0]), 
         .C(mic_sample_count[1]), .Z(n23169)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam i13203_2_lut_rep_87_3_lut.init = 16'h4040;
    LUT4 i1_3_lut_4_lut_adj_135 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[43]), 
         .D(ev_bit[43]), .Z(ev_wr_data[43])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_135.init = 16'hddd0;
    LUT4 i1_2_lut_rep_111 (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .Z(n23193)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(291[17] 323[24])
    defparam i1_2_lut_rep_111.init = 16'h4444;
    LUT4 i14037_3_lut (.A(spi_command[6]), .B(spi_command[4]), .C(spi_command[3]), 
         .Z(n22680)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i14037_3_lut.init = 16'hfefe;
    LUT4 i1_3_lut_4_lut_adj_136 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[44]), 
         .D(ev_bit[44]), .Z(ev_wr_data[44])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_136.init = 16'hddd0;
    LUT4 i1_3_lut_adj_137 (.A(n13915), .B(init_shadow[47]), .C(ev_bit[47]), 
         .Z(n12450)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_137.init = 16'hecec;
    LUT4 i1_3_lut_adj_138 (.A(n13915), .B(init_shadow[46]), .C(ev_bit[46]), 
         .Z(n12444)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_138.init = 16'hecec;
    LUT4 i7_4_lut_adj_139 (.A(ev_run_hold[35]), .B(us_tx_c_35), .C(init_shadow[35]), 
         .D(swap_now_d3), .Z(n10928)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_139.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_140 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[45]), 
         .D(ev_bit[45]), .Z(ev_wr_data[45])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_140.init = 16'hddd0;
    FD1P3IX frame_settle__i1 (.D(n13717), .SP(pll_clk_enable_599), .CD(n11187), 
            .CK(pll_clk), .Q(frame_settle[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_settle__i1.GSR = "DISABLED";
    LUT4 i7_4_lut_adj_141 (.A(ev_run_hold[36]), .B(us_tx_c_36), .C(init_shadow[36]), 
         .D(swap_now_d3), .Z(n10930)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_141.init = 16'h5a66;
    LUT4 i7_4_lut_adj_142 (.A(ev_run_hold[1]), .B(us_tx_c_1), .C(init_shadow[1]), 
         .D(swap_now_d3), .Z(n10860)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_142.init = 16'h5a66;
    CCU2D expected_next_15__I_0_488_3 (.A0(expected_next_15__N_1301[7]), .B0(spi_extension_length[3]), 
          .C0(GND_net), .D0(GND_net), .A1(expected_next_15__N_1342[3]), 
          .B1(spi_extension_length[4]), .C1(GND_net), .D1(GND_net), .CIN(n21830), 
          .COUT(n21831), .S0(expected_next[3]), .S1(expected_next[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_3.INIT0 = 16'h5666;
    defparam expected_next_15__I_0_488_3.INIT1 = 16'h5666;
    defparam expected_next_15__I_0_488_3.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_3.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_143 (.A(ev_run_hold[37]), .B(us_tx_c_37), .C(init_shadow[37]), 
         .D(swap_now_d3), .Z(n10932)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_143.init = 16'h5a66;
    LUT4 i3_4_lut_adj_144 (.A(n55), .B(n23152), .C(spi_byte_count[4]), 
         .D(n22580), .Z(spi1_sck_c_enable_177)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i3_4_lut_adj_144.init = 16'h0080;
    LUT4 i2_2_lut_rep_88_3_lut (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .Z(n23170)) /* synthesis lut_function=(!(A+((C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(291[17] 323[24])
    defparam i2_2_lut_rep_88_3_lut.init = 16'h0404;
    LUT4 i13937_4_lut (.A(spi_byte_count[12]), .B(n22572), .C(spi_byte_count[5]), 
         .D(spi_byte_count[13]), .Z(n22580)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i13937_4_lut.init = 16'hfffe;
    LUT4 i4027_4_lut (.A(ev_ch[5]), .B(staging_rd_addr[5]), .C(n15103), 
         .D(n22416), .Z(staging_rd_addr_6__N_785[5])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i4027_4_lut.init = 16'hcac0;
    LUT4 i7_4_lut_adj_145 (.A(ev_run_hold[38]), .B(us_tx_c_38), .C(init_shadow[38]), 
         .D(swap_now_d3), .Z(n10934)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_145.init = 16'h5a66;
    LUT4 i7_4_lut_adj_146 (.A(ev_run_hold[39]), .B(us_tx_c_39), .C(init_shadow[39]), 
         .D(swap_now_d3), .Z(n10936)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_146.init = 16'h5a66;
    LUT4 i7_4_lut_adj_147 (.A(ev_run_hold[40]), .B(us_tx_c_40), .C(init_shadow[40]), 
         .D(swap_now_d3), .Z(n10938)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_147.init = 16'h5a66;
    LUT4 i81_3_lut_3_lut (.A(spi_byte_count[3]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[2]), .Z(n60)) /* synthesis lut_function=(!(A (B+(C))+!A !(B (C)+!B !(C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(291[17] 323[24])
    defparam i81_3_lut_3_lut.init = 16'h4343;
    LUT4 i1_3_lut_adj_148 (.A(n13915), .B(init_shadow[45]), .C(ev_bit[45]), 
         .Z(n12438)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_148.init = 16'hecec;
    LUT4 frame_req_I_0_480_2_lut_rep_112 (.A(frame_req), .B(swap_pending), 
         .Z(n23194)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(409[21:47])
    defparam frame_req_I_0_480_2_lut_rep_112.init = 16'h2222;
    LUT4 i7_4_lut_adj_149 (.A(ev_run_hold[41]), .B(us_tx_c_41), .C(init_shadow[41]), 
         .D(swap_now_d3), .Z(n10940)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_149.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_150 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[51]), 
         .D(ev_bit[51]), .Z(ev_wr_data[51])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_150.init = 16'hddd0;
    LUT4 i1_3_lut_adj_151 (.A(n13915), .B(init_shadow[44]), .C(ev_bit[44]), 
         .Z(n12432)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_151.init = 16'hecec;
    CCU2D fpga_time_1091_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21846), .COUT(n21847), .S0(n162), .S1(n161));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_5.INJECT1_1 = "NO";
    LUT4 i9962_4_lut (.A(n13211), .B(fpga_cs_n_c), .C(n63), .D(status_bit_index[6]), 
         .Z(spi1_miso_c)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(263[24:81])
    defparam i9962_4_lut.init = 16'h3022;
    LUT4 i10263_2_lut (.A(n22781), .B(status_bit_index[3]), .Z(n13211)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(263[55:80])
    defparam i10263_2_lut.init = 16'h2222;
    LUT4 i4483_2_lut_3_lut_4_lut (.A(frame_req), .B(swap_pending), .C(pll_clk_enable_1), 
         .D(n23171), .Z(n13217)) /* synthesis lut_function=(A (B (C)+!B (C+!(D)))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(409[21:47])
    defparam i4483_2_lut_3_lut_4_lut.init = 16'hf0f2;
    LUT4 i3_4_lut_adj_152 (.A(spi_byte_count[8]), .B(spi_byte_count[11]), 
         .C(spi_byte_count[9]), .D(spi_byte_count[10]), .Z(n22485)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(132[36:83])
    defparam i3_4_lut_adj_152.init = 16'hfffe;
    LUT4 i4029_4_lut (.A(ev_ch[6]), .B(staging_rd_addr[6]), .C(n15103), 
         .D(n22416), .Z(staging_rd_addr_6__N_785[6])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i4029_4_lut.init = 16'hcac0;
    LUT4 i1_3_lut_adj_153 (.A(n13915), .B(init_shadow[43]), .C(ev_bit[43]), 
         .Z(n12426)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_153.init = 16'hecec;
    LUT4 i7_4_lut_adj_154 (.A(ev_run_hold[42]), .B(us_tx_c_42), .C(init_shadow[42]), 
         .D(swap_now_d3), .Z(n10942)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_154.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_155 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[52]), 
         .D(ev_bit[52]), .Z(ev_wr_data[52])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_155.init = 16'hddd0;
    LUT4 run_addr_reg_8__I_0_i4_3_lut (.A(run_addr_reg[3]), .B(ev_rd_slot[3]), 
         .C(n15_adj_3023), .Z(event_rd_addr[3])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i4_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_4_lut_adj_156 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[53]), 
         .D(ev_bit[53]), .Z(ev_wr_data[53])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_156.init = 16'hddd0;
    CCU2D expected_next_15__I_0_488_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(expected_next_15__N_1342[3]), .B1(spi_extension_length[2]), 
          .C1(GND_net), .D1(GND_net), .COUT(n21830), .S1(expected_next[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_1.INIT0 = 16'hF000;
    defparam expected_next_15__I_0_488_1.INIT1 = 16'ha999;
    defparam expected_next_15__I_0_488_1.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_1.INJECT1_1 = "NO";
    CCU2D fpga_time_1091_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21845), .COUT(n21846), .S0(n164), .S1(n163));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_3.INJECT1_1 = "NO";
    LUT4 i1626_3_lut_rep_67_4_lut (.A(frame_req), .B(swap_pending), .C(ev_state[0]), 
         .D(ev_state_3__N_1912[1]), .Z(n23149)) /* synthesis lut_function=(A (B (C (D))+!B ((D)+!C))+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(409[21:47])
    defparam i1626_3_lut_rep_67_4_lut.init = 16'hf202;
    LUT4 i7_4_lut_adj_157 (.A(ev_run_hold[43]), .B(us_tx_c_43), .C(init_shadow[43]), 
         .D(swap_now_d3), .Z(n10944)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_157.init = 16'h5a66;
    LUT4 i4_4_lut_adj_158 (.A(n7_adj_3024), .B(n23155), .C(expected_next_15__N_1342[3]), 
         .D(n25), .Z(spi1_sck_c_enable_272)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i4_4_lut_adj_158.init = 16'h8000;
    CCU2D time_divider_1092_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21867), .S0(n35), .S1(n34));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_1092_add_4_7.INIT1 = 16'hfaaa;
    defparam time_divider_1092_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_1092_add_4_7.INJECT1_1 = "NO";
    LUT4 n15_bdd_3_lut_14337 (.A(n15_adj_3023), .B(ev_state[1]), .C(ev_state[0]), 
         .Z(n23050)) /* synthesis lut_function=(!(A (B (C)+!B !(C)))) */ ;
    defparam n15_bdd_3_lut_14337.init = 16'h7d7d;
    LUT4 i1_3_lut_4_lut_adj_159 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[54]), 
         .D(ev_bit[54]), .Z(ev_wr_data[54])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_159.init = 16'hddd0;
    LUT4 i14136_3_lut_3_lut (.A(status_bit_index[4]), .B(n23187), .C(status_hold[104]), 
         .Z(n22780)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14136_3_lut_3_lut.init = 16'hc4c4;
    LUT4 i14199_3_lut_3_lut (.A(status_bit_index[4]), .B(n22842), .C(n22841), 
         .Z(n22843)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14199_3_lut_3_lut.init = 16'he4e4;
    LUT4 i7_4_lut_adj_160 (.A(ev_run_hold[44]), .B(us_tx_c_44), .C(init_shadow[44]), 
         .D(swap_now_d3), .Z(n10946)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_160.init = 16'h5a66;
    LUT4 i14233_3_lut_4_lut_4_lut (.A(status_bit_index[4]), .B(n23208), 
         .C(status_hold[88]), .D(n23187), .Z(n22779)) /* synthesis lut_function=(A (B)+!A (C (D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14233_3_lut_4_lut_4_lut.init = 16'hd888;
    LUT4 i3_4_lut_adj_161 (.A(spi_extension_length[4]), .B(spi_extension_length[2]), 
         .C(spi_extension_length[3]), .D(n22465), .Z(n22467)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i3_4_lut_adj_161.init = 16'hfffe;
    LUT4 i14168_3_lut_3_lut (.A(status_bit_index[4]), .B(n22811), .C(n22810), 
         .Z(n22812)) /* synthesis lut_function=(A (C)+!A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[18:44])
    defparam i14168_3_lut_3_lut.init = 16'he4e4;
    LUT4 i6338_4_lut_4_lut_then_4_lut (.A(ev_state[2]), .B(ev_state[3]), 
         .C(ev_state[1]), .D(ev_state_3__N_1912[1]), .Z(n23203)) /* synthesis lut_function=(!(A+!(B+!(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i6338_4_lut_4_lut_then_4_lut.init = 16'h4544;
    LUT4 i1_3_lut_4_lut_adj_162 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[21]), 
         .D(ev_bit[21]), .Z(ev_wr_data[21])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_162.init = 16'hddd0;
    CCU2D time_divider_1092_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21866), .COUT(n21867), .S0(n37), 
          .S1(n36));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_1092_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_1092_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_1092_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_163 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[55]), 
         .D(ev_bit[55]), .Z(ev_wr_data[55])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_163.init = 16'hddd0;
    CCU2D fpga_time_1091_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n21845), .S1(n165));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_1091_add_4_1.INIT1 = 16'h0555;
    defparam fpga_time_1091_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_164 (.A(ev_run_hold[45]), .B(us_tx_c_45), .C(init_shadow[45]), 
         .D(swap_now_d3), .Z(n10948)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_164.init = 16'h5a66;
    CCU2D status_bit_index_1087_add_4_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21843), .S0(n35_adj_3010), 
          .S1(n34_adj_3011));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087_add_4_7.INIT0 = 16'hfaaa;
    defparam status_bit_index_1087_add_4_7.INIT1 = 16'hfaaa;
    defparam status_bit_index_1087_add_4_7.INJECT1_0 = "NO";
    defparam status_bit_index_1087_add_4_7.INJECT1_1 = "NO";
    CCU2D time_divider_1092_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21865), .COUT(n21866), .S0(n39), 
          .S1(n38));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_1092_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_1092_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_1092_add_4_3.INJECT1_1 = "NO";
    FD1P3IX frame_settle__i2 (.D(n13719), .SP(pll_clk_enable_599), .CD(n11187), 
            .CK(pll_clk), .Q(frame_settle[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam frame_settle__i2.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_165 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[56]), 
         .D(ev_bit[56]), .Z(ev_wr_data[56])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_165.init = 16'hddd0;
    FD1P3AX spi_channel_index_1088__i1 (.D(n39_adj_3014), .SP(spi1_sck_c_enable_324), 
            .CK(spi1_sck_c), .Q(spi_channel_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088__i1.GSR = "ENABLED";
    LUT4 i2_3_lut_4_lut_adj_166 (.A(spi_byte_count[2]), .B(spi_byte_count[1]), 
         .C(spi_byte_count[8]), .D(n23196), .Z(n22469)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i2_3_lut_4_lut_adj_166.init = 16'hfffe;
    LUT4 i1_2_lut_rep_114 (.A(spi_byte_count[5]), .B(spi_byte_count[4]), 
         .Z(n23196)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i1_2_lut_rep_114.init = 16'heeee;
    LUT4 i7_4_lut_adj_167 (.A(ev_run_hold[46]), .B(us_tx_c_46), .C(init_shadow[46]), 
         .D(swap_now_d3), .Z(n10950)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_167.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_168 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[57]), 
         .D(ev_bit[57]), .Z(ev_wr_data[57])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_168.init = 16'hddd0;
    LUT4 i14025_3_lut (.A(n22772), .B(expected_next_15__N_1301[7]), .C(n174), 
         .Z(n22668)) /* synthesis lut_function=(A (B (C))+!A (B)) */ ;
    defparam i14025_3_lut.init = 16'hc4c4;
    LUT4 i7_4_lut_adj_169 (.A(ev_run_hold[47]), .B(us_tx_c_47), .C(init_shadow[47]), 
         .D(swap_now_d3), .Z(n10952)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_169.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_170 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[58]), 
         .D(ev_bit[58]), .Z(ev_wr_data[58])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_170.init = 16'hddd0;
    LUT4 i7_4_lut_adj_171 (.A(ev_run_hold[48]), .B(us_tx_c_48), .C(init_shadow[48]), 
         .D(swap_now_d3), .Z(n10954)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_171.init = 16'h5a66;
    LUT4 i7_4_lut_adj_172 (.A(ev_run_hold[49]), .B(us_tx_c_49), .C(init_shadow[49]), 
         .D(swap_now_d3), .Z(n10956)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_172.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_173 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[59]), 
         .D(ev_bit[59]), .Z(ev_wr_data[59])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_173.init = 16'hddd0;
    LUT4 i7_4_lut_adj_174 (.A(ev_run_hold[50]), .B(us_tx_c_50), .C(init_shadow[50]), 
         .D(swap_now_d3), .Z(n10958)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_174.init = 16'h5a66;
    LUT4 i7_4_lut_adj_175 (.A(ev_run_hold[51]), .B(us_tx_c_51), .C(init_shadow[51]), 
         .D(swap_now_d3), .Z(n10960)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_175.init = 16'h5a66;
    CCU2D time_divider_1092_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n21865), .S1(n40));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_1092_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_1092_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_1092_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_176 (.A(n13915), .B(init_shadow[42]), .C(ev_bit[42]), 
         .Z(n12420)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_176.init = 16'hecec;
    LUT4 i1_3_lut_adj_177 (.A(n13915), .B(init_shadow[41]), .C(ev_bit[41]), 
         .Z(n12414)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_177.init = 16'hecec;
    LUT4 i1_3_lut_adj_178 (.A(n13915), .B(init_shadow[40]), .C(ev_bit[40]), 
         .Z(n12408)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_178.init = 16'hecec;
    CCU2D status_bit_index_1087_add_4_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21842), .COUT(n21843), .S0(n37_adj_3008), 
          .S1(n36_adj_3009));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087_add_4_5.INIT0 = 16'hfaaa;
    defparam status_bit_index_1087_add_4_5.INIT1 = 16'hfaaa;
    defparam status_bit_index_1087_add_4_5.INJECT1_0 = "NO";
    defparam status_bit_index_1087_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_179 (.A(n13915), .B(init_shadow[39]), .C(ev_bit[39]), 
         .Z(n12402)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_179.init = 16'hecec;
    LUT4 i1_3_lut_adj_180 (.A(n13915), .B(init_shadow[38]), .C(ev_bit[38]), 
         .Z(n12396)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_180.init = 16'hecec;
    LUT4 i1_3_lut_adj_181 (.A(n13915), .B(init_shadow[37]), .C(ev_bit[37]), 
         .Z(n12390)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_181.init = 16'hecec;
    LUT4 i1_3_lut_adj_182 (.A(n13915), .B(init_shadow[36]), .C(ev_bit[36]), 
         .Z(n12384)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_182.init = 16'hecec;
    LUT4 i1_3_lut_adj_183 (.A(n13915), .B(init_shadow[35]), .C(ev_bit[35]), 
         .Z(n12378)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_183.init = 16'hecec;
    LUT4 i1_3_lut_adj_184 (.A(n13915), .B(init_shadow[34]), .C(ev_bit[34]), 
         .Z(n12372)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_184.init = 16'hecec;
    LUT4 i1_3_lut_adj_185 (.A(n13915), .B(init_shadow[33]), .C(ev_bit[33]), 
         .Z(n12366)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_185.init = 16'hecec;
    LUT4 i1_3_lut_adj_186 (.A(n13915), .B(init_shadow[32]), .C(ev_bit[32]), 
         .Z(n12360)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_186.init = 16'hecec;
    PFUMX ev_state_3__I_0_441_Mux_1_i15 (.BLUT(n7_adj_2989), .ALUT(n14_adj_2990), 
          .C0(ev_state[3]), .Z(ev_state_3__N_581[1]));
    LUT4 i1_3_lut_adj_187 (.A(n13915), .B(init_shadow[31]), .C(ev_bit[31]), 
         .Z(n12354)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_187.init = 16'hecec;
    FD1P3AX spi_channel_index_1088__i2 (.D(n38_adj_3015), .SP(spi1_sck_c_enable_324), 
            .CK(spi1_sck_c), .Q(spi_channel_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088__i2.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1088__i3 (.D(n37_adj_3016), .SP(spi1_sck_c_enable_324), 
            .CK(spi1_sck_c), .Q(spi_channel_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088__i3.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1088__i4 (.D(n36_adj_3017), .SP(spi1_sck_c_enable_324), 
            .CK(spi1_sck_c), .Q(spi_channel_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088__i4.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1088__i5 (.D(n35_adj_3018), .SP(spi1_sck_c_enable_324), 
            .CK(spi1_sck_c), .Q(spi_channel_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088__i5.GSR = "ENABLED";
    FD1P3AX spi_channel_index_1088__i6 (.D(n34_adj_3019), .SP(spi1_sck_c_enable_324), 
            .CK(spi1_sck_c), .Q(spi_channel_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088__i6.GSR = "ENABLED";
    FD1P3AX status_bit_index_1087__i1 (.D(n39_adj_3006), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087__i1.GSR = "ENABLED";
    CCU2D mic_divider_1094_add_4_7 (.A0(mic_divider[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21863), .S0(n35_adj_2996), .S1(n34_adj_2997));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_divider_1094_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_divider_1094_add_4_7.INJECT1_0 = "NO";
    defparam mic_divider_1094_add_4_7.INJECT1_1 = "NO";
    FD1P3AX status_bit_index_1087__i2 (.D(n38_adj_3007), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087__i2.GSR = "ENABLED";
    FD1P3AX status_bit_index_1087__i3 (.D(n37_adj_3008), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087__i3.GSR = "ENABLED";
    FD1P3AX status_bit_index_1087__i4 (.D(n36_adj_3009), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087__i4.GSR = "ENABLED";
    FD1P3AX status_bit_index_1087__i5 (.D(n35_adj_3010), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087__i5.GSR = "ENABLED";
    FD1P3AX status_bit_index_1087__i6 (.D(n34_adj_3011), .SP(spi1_sck_N_457_enable_7), 
            .CK(spi1_sck_N_457), .Q(status_bit_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087__i6.GSR = "ENABLED";
    FD1P3AX fpga_time_1091__i1 (.D(n164), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i1.GSR = "DISABLED";
    CCU2D add_242_7 (.A0(ev_clear_addr[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21808), .COUT(n21809), .S0(ev_clear_addr_7__N_2209[5]), 
          .S1(ev_clear_addr_7__N_2209[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_7.INIT0 = 16'h5aaa;
    defparam add_242_7.INIT1 = 16'h5aaa;
    defparam add_242_7.INJECT1_0 = "NO";
    defparam add_242_7.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_188 (.A(n13915), .B(init_shadow[30]), .C(ev_bit[30]), 
         .Z(n12348)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_188.init = 16'hecec;
    LUT4 i1_3_lut_adj_189 (.A(n13915), .B(init_shadow[29]), .C(ev_bit[29]), 
         .Z(n12342)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_189.init = 16'hecec;
    CCU2D status_bit_index_1087_add_4_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21841), .COUT(n21842), .S0(n39_adj_3006), 
          .S1(n38_adj_3007));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087_add_4_3.INIT0 = 16'hfaaa;
    defparam status_bit_index_1087_add_4_3.INIT1 = 16'hfaaa;
    defparam status_bit_index_1087_add_4_3.INJECT1_0 = "NO";
    defparam status_bit_index_1087_add_4_3.INJECT1_1 = "NO";
    CCU2D mic_divider_1094_add_4_5 (.A0(mic_divider[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21862), .COUT(n21863), .S0(n37_adj_2994), 
          .S1(n36_adj_2995));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_divider_1094_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_divider_1094_add_4_5.INJECT1_0 = "NO";
    defparam mic_divider_1094_add_4_5.INJECT1_1 = "NO";
    PFUMX i104575_i1 (.BLUT(n22812), .ALUT(n22843), .C0(spi1_miso_N_2421[5]), 
          .Z(n63));
    LUT4 i1_3_lut_adj_190 (.A(n13915), .B(init_shadow[28]), .C(ev_bit[28]), 
         .Z(n12336)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_190.init = 16'hecec;
    FD1P3AX fpga_time_1091__i2 (.D(n163), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i2.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i3 (.D(n162), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i3.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i4 (.D(n161), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i4.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i5 (.D(n160), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i5.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i6 (.D(n159), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i6.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i7 (.D(n158), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i7.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i8 (.D(n157), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i8.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i9 (.D(n156), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i9.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i10 (.D(n155), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i10.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i11 (.D(n154), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i11.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i12 (.D(n153), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i12.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i13 (.D(n152), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i13.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i14 (.D(n151), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i14.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i15 (.D(n150), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i15.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i16 (.D(n149), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i16.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i17 (.D(n148), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i17.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i18 (.D(n147), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i18.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i19 (.D(n146), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i19.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i20 (.D(n145), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i20.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i21 (.D(n144), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i21.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i22 (.D(n143), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i22.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i23 (.D(n142), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i23.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i24 (.D(n141), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i24.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i25 (.D(n140), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i25.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i26 (.D(n139), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i26.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i27 (.D(n138), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i27.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i28 (.D(n137), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i28.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i29 (.D(n136), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i29.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i30 (.D(n135), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i30.GSR = "DISABLED";
    FD1P3AX fpga_time_1091__i31 (.D(n134), .SP(pll_clk_enable_630), .CK(pll_clk), 
            .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091__i31.GSR = "DISABLED";
    FD1S3AX spi_bit_count_1090__i1 (.D(n19), .CK(spi1_sck_c), .Q(spi_bit_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam spi_bit_count_1090__i1.GSR = "ENABLED";
    LUT4 i1_3_lut_adj_191 (.A(n13915), .B(init_shadow[27]), .C(ev_bit[27]), 
         .Z(n12330)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_191.init = 16'hecec;
    LUT4 i1_3_lut_adj_192 (.A(n13915), .B(init_shadow[26]), .C(ev_bit[26]), 
         .Z(n12324)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_192.init = 16'hecec;
    LUT4 i1_3_lut_adj_193 (.A(n13915), .B(init_shadow[25]), .C(ev_bit[25]), 
         .Z(n12318)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_193.init = 16'hecec;
    LUT4 i1_3_lut_adj_194 (.A(n13915), .B(init_shadow[24]), .C(ev_bit[24]), 
         .Z(n12312)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_194.init = 16'hecec;
    LUT4 i1_3_lut_adj_195 (.A(n13915), .B(init_shadow[23]), .C(ev_bit[23]), 
         .Z(n12306)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_195.init = 16'hecec;
    LUT4 i7_4_lut_adj_196 (.A(ev_run_hold[52]), .B(us_tx_c_52), .C(init_shadow[52]), 
         .D(swap_now_d3), .Z(n10962)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_196.init = 16'h5a66;
    LUT4 i1_3_lut_adj_197 (.A(n13915), .B(init_shadow[22]), .C(ev_bit[22]), 
         .Z(n12300)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_197.init = 16'hecec;
    FD1P3IX ev_bit_i2 (.D(ev_bit[1]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i2.GSR = "DISABLED";
    FD1P3IX ev_bit_i40 (.D(ev_bit[39]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i40.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_198 (.A(n13915), .B(init_shadow[21]), .C(ev_bit[21]), 
         .Z(n12294)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_198.init = 16'hecec;
    LUT4 i1_3_lut_adj_199 (.A(n13915), .B(init_shadow[20]), .C(ev_bit[20]), 
         .Z(n12288)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_199.init = 16'hecec;
    FD1P3IX ev_bit_i39 (.D(ev_bit[38]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i39.GSR = "DISABLED";
    PFUMX i14137 (.BLUT(n22779), .ALUT(n22780), .C0(spi1_miso_N_2421[5]), 
          .Z(n22781));
    FD1P3IX ev_bit_i38 (.D(ev_bit[37]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i38.GSR = "DISABLED";
    FD1P3IX ev_bit_i37 (.D(ev_bit[36]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i37.GSR = "DISABLED";
    FD1P3IX ev_bit_i36 (.D(ev_bit[35]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i36.GSR = "DISABLED";
    FD1P3IX ev_bit_i35 (.D(ev_bit[34]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i35.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_200 (.A(n13915), .B(init_shadow[19]), .C(ev_bit[19]), 
         .Z(n12282)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_200.init = 16'hecec;
    FD1P3IX ev_bit_i34 (.D(ev_bit[33]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i34.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_201 (.A(n13915), .B(init_shadow[18]), .C(ev_bit[18]), 
         .Z(n12276)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_201.init = 16'hecec;
    LUT4 i7_4_lut_adj_202 (.A(ev_run_hold[53]), .B(us_tx_c_53), .C(init_shadow[53]), 
         .D(swap_now_d3), .Z(n10964)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_202.init = 16'h5a66;
    LUT4 i1_3_lut_adj_203 (.A(n13915), .B(init_shadow[17]), .C(ev_bit[17]), 
         .Z(n12270)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_203.init = 16'hecec;
    LUT4 i1_3_lut_adj_204 (.A(n13915), .B(init_shadow[16]), .C(ev_bit[16]), 
         .Z(n12264)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_204.init = 16'hecec;
    LUT4 i35_1_lut (.A(fpga_cs_n_c), .Z(fpga_cs_n_N_2458)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i35_1_lut.init = 16'h5555;
    LUT4 i7_4_lut_adj_205 (.A(ev_run_hold[54]), .B(us_tx_c_54), .C(init_shadow[54]), 
         .D(swap_now_d3), .Z(n10966)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_205.init = 16'h5a66;
    LUT4 i7_4_lut_adj_206 (.A(ev_run_hold[55]), .B(us_tx_c_55), .C(init_shadow[55]), 
         .D(swap_now_d3), .Z(n10968)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_206.init = 16'h5a66;
    LUT4 i7_4_lut_adj_207 (.A(ev_run_hold[56]), .B(us_tx_c_56), .C(init_shadow[56]), 
         .D(swap_now_d3), .Z(n10970)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_207.init = 16'h5a66;
    LUT4 i14279_4_lut (.A(status_bit_index[4]), .B(status_bit_index[5]), 
         .C(status_bit_index[3]), .D(n6_adj_2986), .Z(spi1_sck_N_457_enable_7)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;
    defparam i14279_4_lut.init = 16'h7fff;
    LUT4 i1_3_lut_adj_208 (.A(n13915), .B(init_shadow[15]), .C(ev_bit[15]), 
         .Z(n12258)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_208.init = 16'hecec;
    CCU2D add_242_5 (.A0(ev_clear_addr[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21807), .COUT(n21808), .S0(ev_clear_addr_7__N_2209[3]), 
          .S1(ev_clear_addr_7__N_2209[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_5.INIT0 = 16'h5aaa;
    defparam add_242_5.INIT1 = 16'h5aaa;
    defparam add_242_5.INJECT1_0 = "NO";
    defparam add_242_5.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_209 (.A(ev_run_hold[57]), .B(us_tx_c_57), .C(init_shadow[57]), 
         .D(swap_now_d3), .Z(n10972)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_209.init = 16'h5a66;
    LUT4 i6_4_lut_adj_210 (.A(time_divider[2]), .B(n12_adj_2987), .C(time_divider[6]), 
         .D(time_divider[1]), .Z(pll_clk_enable_630)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut_adj_210.init = 16'h8000;
    CCU2D add_242_3 (.A0(ev_clear_addr[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(ev_clear_addr[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21806), .COUT(n21807), .S0(ev_clear_addr_7__N_2209[1]), 
          .S1(ev_clear_addr_7__N_2209[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_3.INIT0 = 16'h5aaa;
    defparam add_242_3.INIT1 = 16'h5aaa;
    defparam add_242_3.INJECT1_0 = "NO";
    defparam add_242_3.INJECT1_1 = "NO";
    LUT4 i5_4_lut_adj_211 (.A(time_divider[0]), .B(time_divider[5]), .C(time_divider[4]), 
         .D(time_divider[3]), .Z(n12_adj_2987)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut_adj_211.init = 16'h8000;
    LUT4 i7_4_lut_adj_212 (.A(ev_run_hold[58]), .B(us_tx_c_58), .C(init_shadow[58]), 
         .D(swap_now_d3), .Z(n10974)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_212.init = 16'h5a66;
    LUT4 i7_4_lut_adj_213 (.A(ev_run_hold[59]), .B(us_tx_c_59), .C(init_shadow[59]), 
         .D(swap_now_d3), .Z(n10976)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_213.init = 16'h5a66;
    LUT4 i1_3_lut_adj_214 (.A(n13915), .B(init_shadow[14]), .C(ev_bit[14]), 
         .Z(n12252)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_214.init = 16'hecec;
    LUT4 i13162_1_lut (.A(spi_bit_count[0]), .Z(n20)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam i13162_1_lut.init = 16'h5555;
    LUT4 i14129_4_lut (.A(n22698), .B(n22764), .C(n22742), .D(n22696), 
         .Z(n22772)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14129_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_adj_215 (.A(expected_next_15__N_1301[7]), .B(n22424), 
         .Z(spi1_sck_c_enable_325)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_215.init = 16'h8888;
    CCU2D add_242_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(ev_clear_addr[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21806), .S1(ev_clear_addr_7__N_2209[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(420[34:54])
    defparam add_242_1.INIT0 = 16'hF000;
    defparam add_242_1.INIT1 = 16'h5555;
    defparam add_242_1.INJECT1_0 = "NO";
    defparam add_242_1.INJECT1_1 = "NO";
    LUT4 mux_1209_i8_3_lut (.A(n9828), .B(n9829), .C(n9813), .Z(rd_data_15__N_2535[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i8_3_lut.init = 16'hcaca;
    CCU2D add_135_17 (.A0(spi_byte_count[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21805), .S0(spi_byte_count_15__N_1579[15]), .S1(frame_end_N_2500[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_17.INIT0 = 16'h5aaa;
    defparam add_135_17.INIT1 = 16'h0000;
    defparam add_135_17.INJECT1_0 = "NO";
    defparam add_135_17.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i33 (.D(ev_bit[32]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i33.GSR = "DISABLED";
    LUT4 i13186_2_lut (.A(spi_channel_field[1]), .B(spi_channel_field[0]), 
         .Z(n14_adj_3000)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(316[78:102])
    defparam i13186_2_lut.init = 16'h6666;
    CCU2D mic_divider_1094_add_4_3 (.A0(mic_divider[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21861), .COUT(n21862), .S0(n39_adj_2992), 
          .S1(n38_adj_2993));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_divider_1094_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_divider_1094_add_4_3.INJECT1_0 = "NO";
    defparam mic_divider_1094_add_4_3.INJECT1_1 = "NO";
    LUT4 run_addr_reg_8__I_0_i5_3_lut (.A(run_addr_reg[4]), .B(ev_rd_slot[4]), 
         .C(n15_adj_3023), .Z(event_rd_addr[4])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i5_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_216 (.A(n13915), .B(init_shadow[13]), .C(ev_bit[13]), 
         .Z(n12246)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_216.init = 16'hecec;
    FD1P3IX ev_bit_i32 (.D(ev_bit[31]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i32.GSR = "DISABLED";
    LUT4 i86_4_lut (.A(n22672), .B(n22774), .C(n166), .D(n22674), .Z(n174)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;
    defparam i86_4_lut.init = 16'hf7ff;
    CCU2D status_bit_index_1087_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(status_bit_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n21841), .S1(n40_adj_3005));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(267[66:89])
    defparam status_bit_index_1087_add_4_1.INIT0 = 16'hF000;
    defparam status_bit_index_1087_add_4_1.INIT1 = 16'h0555;
    defparam status_bit_index_1087_add_4_1.INJECT1_0 = "NO";
    defparam status_bit_index_1087_add_4_1.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_217 (.A(ev_run_hold[60]), .B(us_tx_c_60), .C(init_shadow[60]), 
         .D(swap_now_d3), .Z(n10978)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_217.init = 16'h5a66;
    LUT4 i7_4_lut_adj_218 (.A(ev_run_hold[61]), .B(us_tx_c_61), .C(init_shadow[61]), 
         .D(swap_now_d3), .Z(n10980)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_218.init = 16'h5a66;
    CCU2D add_135_15 (.A0(spi_byte_count[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21804), .COUT(n21805), .S0(spi_byte_count_15__N_1579[13]), 
          .S1(spi_byte_count_15__N_1579[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_15.INIT0 = 16'h5aaa;
    defparam add_135_15.INIT1 = 16'h5aaa;
    defparam add_135_15.INJECT1_0 = "NO";
    defparam add_135_15.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_219 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[60]), 
         .D(ev_bit[60]), .Z(ev_wr_data[60])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_219.init = 16'hddd0;
    LUT4 i10038_4_lut (.A(frame_settle[3]), .B(pll_clk_enable_18), .C(frame_settle[2]), 
         .D(n23184), .Z(frame_settle_3__N_1860[3])) /* synthesis lut_function=(A (B+(C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(385[18] 391[12])
    defparam i10038_4_lut.init = 16'heeed;
    CCU2D add_135_13 (.A0(spi_byte_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21803), .COUT(n21804), .S0(spi_byte_count_15__N_1579[11]), 
          .S1(spi_byte_count_15__N_1579[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_13.INIT0 = 16'h5aaa;
    defparam add_135_13.INIT1 = 16'h5aaa;
    defparam add_135_13.INJECT1_0 = "NO";
    defparam add_135_13.INJECT1_1 = "NO";
    CCU2D add_491_25 (.A0(phase_frac[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21821), .S0(phase_frac_sum[23]), .S1(phase_frac_sum[24]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_25.INIT0 = 16'h5aaa;
    defparam add_491_25.INIT1 = 16'h0000;
    defparam add_491_25.INJECT1_0 = "NO";
    defparam add_491_25.INJECT1_1 = "NO";
    CCU2D add_135_11 (.A0(spi_byte_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21802), .COUT(n21803), .S0(spi_byte_count_15__N_1579[9]), 
          .S1(spi_byte_count_15__N_1579[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_11.INIT0 = 16'h5aaa;
    defparam add_135_11.INIT1 = 16'h5aaa;
    defparam add_135_11.INJECT1_0 = "NO";
    defparam add_135_11.INJECT1_1 = "NO";
    CCU2D add_135_9 (.A0(spi_byte_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21801), .COUT(n21802), .S0(spi_byte_count_15__N_1579[7]), 
          .S1(spi_byte_count_15__N_1579[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_9.INIT0 = 16'h5aaa;
    defparam add_135_9.INIT1 = 16'h5aaa;
    defparam add_135_9.INJECT1_0 = "NO";
    defparam add_135_9.INJECT1_1 = "NO";
    CCU2D add_135_7 (.A0(spi_byte_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21800), .COUT(n21801), .S0(spi_byte_count_15__N_1579[5]), 
          .S1(spi_byte_count_15__N_1579[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_7.INIT0 = 16'h5aaa;
    defparam add_135_7.INIT1 = 16'h5aaa;
    defparam add_135_7.INJECT1_0 = "NO";
    defparam add_135_7.INJECT1_1 = "NO";
    CCU2D mic_divider_1094_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n21861), .S1(n40_adj_2991));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094_add_4_1.INIT0 = 16'hF000;
    defparam mic_divider_1094_add_4_1.INIT1 = 16'h0555;
    defparam mic_divider_1094_add_4_1.INJECT1_0 = "NO";
    defparam mic_divider_1094_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_220 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[61]), 
         .D(ev_bit[61]), .Z(ev_wr_data[61])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_220.init = 16'hddd0;
    LUT4 i7_4_lut_adj_221 (.A(ev_run_hold[62]), .B(us_tx_c_62), .C(init_shadow[62]), 
         .D(swap_now_d3), .Z(n10982)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_221.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_222 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[62]), 
         .D(ev_bit[62]), .Z(ev_wr_data[62])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_222.init = 16'hddd0;
    LUT4 i1_3_lut_adj_223 (.A(n13915), .B(init_shadow[12]), .C(ev_bit[12]), 
         .Z(n12240)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_223.init = 16'hecec;
    CCU2D spi_channel_index_1088_add_4_7 (.A0(spi_channel_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21839), .S0(n35_adj_3018), 
          .S1(n34_adj_3019));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088_add_4_7.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1088_add_4_7.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1088_add_4_7.INJECT1_0 = "NO";
    defparam spi_channel_index_1088_add_4_7.INJECT1_1 = "NO";
    L6MUX21 i14166 (.D0(n22806), .D1(n22807), .SD(spi1_miso_N_2421[3]), 
            .Z(n22810));
    LUT4 i1_3_lut_4_lut_adj_224 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[63]), 
         .D(ev_bit[63]), .Z(ev_wr_data[63])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_224.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_225 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[64]), 
         .D(ev_bit[64]), .Z(ev_wr_data[64])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_225.init = 16'hddd0;
    CCU2D fpga_time_1091_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21860), .S0(n134));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_1091_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_33.INJECT1_1 = "NO";
    L6MUX21 i14167 (.D0(n22808), .D1(n22809), .SD(spi1_miso_N_2421[3]), 
            .Z(n22811));
    L6MUX21 i14197 (.D0(n22837), .D1(n22838), .SD(spi1_miso_N_2421[3]), 
            .Z(n22841));
    L6MUX21 i14198 (.D0(n22839), .D1(n22840), .SD(spi1_miso_N_2421[3]), 
            .Z(n22842));
    L6MUX21 i14162 (.D0(n22798), .D1(n22799), .SD(spi1_miso_N_2421[2]), 
            .Z(n22806));
    L6MUX21 i14163 (.D0(n22800), .D1(n22801), .SD(spi1_miso_N_2421[2]), 
            .Z(n22807));
    L6MUX21 i14164 (.D0(n22802), .D1(n22803), .SD(spi1_miso_N_2421[2]), 
            .Z(n22808));
    LUT4 i7_4_lut_adj_226 (.A(ev_run_hold[63]), .B(us_tx_c_63), .C(init_shadow[63]), 
         .D(swap_now_d3), .Z(n10984)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_226.init = 16'h5a66;
    L6MUX21 i14165 (.D0(n22804), .D1(n22805), .SD(spi1_miso_N_2421[2]), 
            .Z(n22809));
    L6MUX21 i14193 (.D0(n22829), .D1(n22830), .SD(spi1_miso_N_2421[2]), 
            .Z(n22837));
    LUT4 i1_3_lut_4_lut_adj_227 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[65]), 
         .D(ev_bit[65]), .Z(ev_wr_data[65])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_227.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_228 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[0]), 
         .D(ev_bit[0]), .Z(ev_wr_data[0])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_228.init = 16'hddd0;
    CCU2D fpga_time_1091_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21859), .COUT(n21860), .S0(n136), .S1(n135));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_31.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_229 (.A(ev_run_hold[64]), .B(us_tx_c_64), .C(init_shadow[64]), 
         .D(swap_now_d3), .Z(n10986)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_229.init = 16'h5a66;
    L6MUX21 i14194 (.D0(n22831), .D1(n22832), .SD(spi1_miso_N_2421[2]), 
            .Z(n22838));
    L6MUX21 i14195 (.D0(n22833), .D1(n22834), .SD(spi1_miso_N_2421[2]), 
            .Z(n22839));
    L6MUX21 i14196 (.D0(n22835), .D1(n22836), .SD(spi1_miso_N_2421[2]), 
            .Z(n22840));
    PFUMX i14154 (.BLUT(n22782), .ALUT(n22783), .C0(spi1_miso_N_2421[1]), 
          .Z(n22798));
    LUT4 i1_3_lut_adj_230 (.A(n13915), .B(init_shadow[11]), .C(ev_bit[11]), 
         .Z(n12234)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_230.init = 16'hecec;
    CCU2D spi_channel_index_1088_add_4_5 (.A0(spi_channel_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21838), .COUT(n21839), .S0(n37_adj_3016), 
          .S1(n36_adj_3017));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088_add_4_5.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1088_add_4_5.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1088_add_4_5.INJECT1_0 = "NO";
    defparam spi_channel_index_1088_add_4_5.INJECT1_1 = "NO";
    CCU2D add_491_23 (.A0(phase_frac[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21820), .COUT(n21821), .S0(phase_frac_sum[21]), 
          .S1(phase_frac_sum[22]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_23.INIT0 = 16'h5aaa;
    defparam add_491_23.INIT1 = 16'h5aaa;
    defparam add_491_23.INJECT1_0 = "NO";
    defparam add_491_23.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_231 (.A(ev_run_hold[65]), .B(us_tx_c_65), .C(init_shadow[65]), 
         .D(swap_now_d3), .Z(n10988)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_231.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_232 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[1]), 
         .D(ev_bit[1]), .Z(ev_wr_data[1])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_232.init = 16'hddd0;
    CCU2D add_491_21 (.A0(phase_frac[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21819), .COUT(n21820), .S0(phase_frac_sum[19]), 
          .S1(phase_frac_sum[20]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_21.INIT0 = 16'h5aaa;
    defparam add_491_21.INIT1 = 16'h5555;
    defparam add_491_21.INJECT1_0 = "NO";
    defparam add_491_21.INJECT1_1 = "NO";
    CCU2D spi_channel_index_1088_add_4_3 (.A0(spi_channel_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_channel_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21837), .COUT(n21838), .S0(n39_adj_3014), 
          .S1(n38_adj_3015));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088_add_4_3.INIT0 = 16'hfaaa;
    defparam spi_channel_index_1088_add_4_3.INIT1 = 16'hfaaa;
    defparam spi_channel_index_1088_add_4_3.INJECT1_0 = "NO";
    defparam spi_channel_index_1088_add_4_3.INJECT1_1 = "NO";
    CCU2D add_491_19 (.A0(phase_frac[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21818), .COUT(n21819), .S0(phase_frac_sum[17]), 
          .S1(phase_frac_sum[18]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_19.INIT0 = 16'h5aaa;
    defparam add_491_19.INIT1 = 16'h5555;
    defparam add_491_19.INJECT1_0 = "NO";
    defparam add_491_19.INJECT1_1 = "NO";
    PFUMX i14155 (.BLUT(n22784), .ALUT(n22785), .C0(spi1_miso_N_2421[1]), 
          .Z(n22799));
    FD1P3IX ev_bit_i31 (.D(ev_bit[30]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i31.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_233 (.A(n13915), .B(init_shadow[10]), .C(ev_bit[10]), 
         .Z(n12228)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_233.init = 16'hecec;
    CCU2D add_491_17 (.A0(phase_frac[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21817), .COUT(n21818), .S0(phase_frac_sum[15]), 
          .S1(phase_frac_sum[16]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_17.INIT0 = 16'h5aaa;
    defparam add_491_17.INIT1 = 16'h5aaa;
    defparam add_491_17.INJECT1_0 = "NO";
    defparam add_491_17.INJECT1_1 = "NO";
    LUT4 cs_sync_d_I_0_2_lut (.A(cs_sync_d), .B(cs_sync), .Z(cs_fall)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(243[20:41])
    defparam cs_sync_d_I_0_2_lut.init = 16'h2222;
    CCU2D fpga_time_1091_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21858), .COUT(n21859), .S0(n138), .S1(n137));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_29.INJECT1_1 = "NO";
    CCU2D spi_channel_index_1088_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_channel_index[0]), .B1(n21880), .C1(spi_channel_index[2]), 
          .D1(n5_adj_3022), .COUT(n21837), .S1(n40_adj_3013));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(312[64:88])
    defparam spi_channel_index_1088_add_4_1.INIT0 = 16'hF000;
    defparam spi_channel_index_1088_add_4_1.INIT1 = 16'h5559;
    defparam spi_channel_index_1088_add_4_1.INJECT1_0 = "NO";
    defparam spi_channel_index_1088_add_4_1.INJECT1_1 = "NO";
    CCU2D fpga_time_1091_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21857), .COUT(n21858), .S0(n140), .S1(n139));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_27.INJECT1_1 = "NO";
    PFUMX i14156 (.BLUT(n22786), .ALUT(n22787), .C0(spi1_miso_N_2421[1]), 
          .Z(n22800));
    CCU2D expected_next_15__I_0_488_15 (.A0(spi_rx_shift[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21836), .S0(expected_next[15]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_15.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_488_15.INIT1 = 16'h0000;
    defparam expected_next_15__I_0_488_15.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_15.INJECT1_1 = "NO";
    CCU2D add_491_15 (.A0(phase_frac[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21816), .COUT(n21817), .S0(phase_frac_sum[13]), 
          .S1(phase_frac_sum[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_15.INIT0 = 16'h5555;
    defparam add_491_15.INIT1 = 16'h5555;
    defparam add_491_15.INJECT1_0 = "NO";
    defparam add_491_15.INJECT1_1 = "NO";
    LUT4 i1_3_lut_adj_234 (.A(n13915), .B(init_shadow[9]), .C(ev_bit[9]), 
         .Z(n12222)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_234.init = 16'hecec;
    CCU2D fpga_time_1091_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21856), .COUT(n21857), .S0(n142), .S1(n141));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_25.INJECT1_1 = "NO";
    CCU2D expected_next_15__I_0_488_13 (.A0(spi_rx_shift[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21835), .COUT(n21836), .S0(expected_next[13]), 
          .S1(expected_next[14]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_13.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_488_13.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_488_13.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_13.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_235 (.A(ev_run_hold[66]), .B(us_tx_c_66), .C(init_shadow[66]), 
         .D(swap_now_d3), .Z(n10990)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_235.init = 16'h5a66;
    CCU2D add_491_13 (.A0(phase_frac[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21815), .COUT(n21816), .S0(phase_frac_sum[11]), 
          .S1(phase_frac_sum[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_13.INIT0 = 16'h5555;
    defparam add_491_13.INIT1 = 16'h5555;
    defparam add_491_13.INJECT1_0 = "NO";
    defparam add_491_13.INJECT1_1 = "NO";
    LUT4 i7_4_lut_adj_236 (.A(ev_run_hold[67]), .B(us_tx_c_67), .C(init_shadow[67]), 
         .D(swap_now_d3), .Z(n10992)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_236.init = 16'h5a66;
    PFUMX i14157 (.BLUT(n22788), .ALUT(n22789), .C0(spi1_miso_N_2421[1]), 
          .Z(n22801));
    LUT4 i1_3_lut_4_lut_adj_237 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[2]), 
         .D(ev_bit[2]), .Z(ev_wr_data[2])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_237.init = 16'hddd0;
    LUT4 i7_4_lut_adj_238 (.A(ev_run_hold[68]), .B(us_tx_c_68), .C(init_shadow[68]), 
         .D(swap_now_d3), .Z(n10994)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_238.init = 16'h5a66;
    LUT4 i7_4_lut_adj_239 (.A(ev_run_hold[69]), .B(us_tx_c_69), .C(init_shadow[69]), 
         .D(swap_now_d3), .Z(n10996)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_239.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_240 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[3]), 
         .D(ev_bit[3]), .Z(ev_wr_data[3])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_240.init = 16'hddd0;
    LUT4 i7_4_lut_adj_241 (.A(ev_run_hold[0]), .B(us_tx_c_0), .C(init_shadow[0]), 
         .D(swap_now_d3), .Z(n10287)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_241.init = 16'h5a66;
    CCU2D add_135_5 (.A0(spi_byte_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21799), .COUT(n21800), .S0(spi_byte_count_15__N_1579[3]), 
          .S1(spi_byte_count_15__N_1579[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_5.INIT0 = 16'h5aaa;
    defparam add_135_5.INIT1 = 16'h5aaa;
    defparam add_135_5.INJECT1_0 = "NO";
    defparam add_135_5.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i30 (.D(ev_bit[29]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i30.GSR = "DISABLED";
    LUT4 i4017_4_lut (.A(ev_ch[3]), .B(staging_rd_addr[3]), .C(n15103), 
         .D(n22416), .Z(staging_rd_addr_6__N_785[3])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i4017_4_lut.init = 16'hcac0;
    LUT4 i9_4_lut (.A(n22676), .B(next_phase[1]), .C(n16), .D(next_phase[7]), 
         .Z(swap_now)) /* synthesis lut_function=(!(A+(B+((D)+!C)))) */ ;
    defparam i9_4_lut.init = 16'h0010;
    LUT4 i14033_4_lut (.A(next_phase[2]), .B(next_phase[3]), .C(next_phase[4]), 
         .D(next_phase[5]), .Z(n22676)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14033_4_lut.init = 16'hfffe;
    PFUMX i14158 (.BLUT(n22790), .ALUT(n22791), .C0(spi1_miso_N_2421[1]), 
          .Z(n22802));
    LUT4 i7_4_lut_adj_242 (.A(ev_run_hold[70]), .B(us_tx_c_70), .C(init_shadow[70]), 
         .D(swap_now_d3), .Z(n10998)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_242.init = 16'h5a66;
    LUT4 i1_3_lut_adj_243 (.A(n13915), .B(init_shadow[8]), .C(ev_bit[8]), 
         .Z(n12216)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_243.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_244 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[4]), 
         .D(ev_bit[4]), .Z(ev_wr_data[4])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_244.init = 16'hddd0;
    LUT4 i588_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(pll_clk_enable_461)) /* synthesis lut_function=(!(A+!(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(507[18] 509[12])
    defparam i588_2_lut.init = 16'h4444;
    LUT4 i1_3_lut_adj_245 (.A(n13915), .B(init_shadow[7]), .C(ev_bit[7]), 
         .Z(n12210)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_245.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_246 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[5]), 
         .D(ev_bit[5]), .Z(ev_wr_data[5])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_246.init = 16'hddd0;
    LUT4 i3_4_lut_adj_247 (.A(ev_ch[5]), .B(ev_ch[2]), .C(ev_ch[3]), .D(n22724), 
         .Z(ev_state_3__N_1896[1])) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(441[21:35])
    defparam i3_4_lut_adj_247.init = 16'hfeff;
    PFUMX i14159 (.BLUT(n22792), .ALUT(n22793), .C0(spi1_miso_N_2421[1]), 
          .Z(n22803));
    LUT4 i7_4_lut_adj_248 (.A(ev_run_hold[71]), .B(us_tx_c_71), .C(init_shadow[71]), 
         .D(swap_now_d3), .Z(n11000)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_248.init = 16'h5a66;
    PFUMX i14335 (.BLUT(n23051), .ALUT(n23050), .C0(ev_state[2]), .Z(n23052));
    LUT4 i14081_4_lut (.A(ev_ch[6]), .B(ev_ch[0]), .C(ev_ch[1]), .D(ev_ch[4]), 
         .Z(n22724)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14081_4_lut.init = 16'h8000;
    PFUMX i14160 (.BLUT(n22794), .ALUT(n22795), .C0(spi1_miso_N_2421[1]), 
          .Z(n22804));
    LUT4 run_addr_reg_8__I_0_i6_3_lut (.A(run_addr_reg[5]), .B(ev_rd_slot[5]), 
         .C(n15_adj_3023), .Z(event_rd_addr[5])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i6_3_lut.init = 16'hacac;
    LUT4 i1_3_lut_adj_249 (.A(n13915), .B(init_shadow[6]), .C(ev_bit[6]), 
         .Z(n12204)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_249.init = 16'hecec;
    CCU2D add_493_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n21790), 
          .S0(build_sum_8__N_2025[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_493_cout.INIT0 = 16'h0000;
    defparam add_493_cout.INIT1 = 16'h0000;
    defparam add_493_cout.INJECT1_0 = "NO";
    defparam add_493_cout.INJECT1_1 = "NO";
    LUT4 i1_3_lut_4_lut_adj_250 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[6]), 
         .D(ev_bit[6]), .Z(ev_wr_data[6])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_250.init = 16'hddd0;
    PFUMX i14161 (.BLUT(n22796), .ALUT(n22797), .C0(spi1_miso_N_2421[1]), 
          .Z(n22805));
    LUT4 i1_3_lut_4_lut_adj_251 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[7]), 
         .D(ev_bit[7]), .Z(ev_wr_data[7])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_251.init = 16'hddd0;
    PFUMX i14185 (.BLUT(n22813), .ALUT(n22814), .C0(spi1_miso_N_2421[1]), 
          .Z(n22829));
    LUT4 i1_3_lut_4_lut_adj_252 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[8]), 
         .D(ev_bit[8]), .Z(ev_wr_data[8])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_252.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_253 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[9]), 
         .D(ev_bit[9]), .Z(ev_wr_data[9])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_253.init = 16'hddd0;
    PFUMX i14186 (.BLUT(n22815), .ALUT(n22816), .C0(spi1_miso_N_2421[1]), 
          .Z(n22830));
    CCU2D fpga_time_1091_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21855), .COUT(n21856), .S0(n144), .S1(n143));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_23.INJECT1_1 = "NO";
    CCU2D fpga_time_1091_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21854), .COUT(n21855), .S0(n146), .S1(n145));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_21.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_254 (.A(ev_state[2]), .B(ev_state[1]), .C(ev_state[3]), 
         .D(n23149), .Z(pll_clk_enable_10)) /* synthesis lut_function=(!(A+!(B (C)+!B (C+(D))))) */ ;
    defparam i1_4_lut_adj_254.init = 16'h5150;
    LUT4 i1_3_lut_adj_255 (.A(n13915), .B(init_shadow[5]), .C(ev_bit[5]), 
         .Z(n12198)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_255.init = 16'hecec;
    LUT4 i10041_3_lut (.A(ev_ch[0]), .B(ev_state[3]), .C(ev_state[0]), 
         .Z(ev_ch_6__N_593[0])) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i10041_3_lut.init = 16'h4848;
    PFUMX i14187 (.BLUT(n22817), .ALUT(n22818), .C0(spi1_miso_N_2421[1]), 
          .Z(n22831));
    LUT4 i1_3_lut_adj_256 (.A(n13915), .B(init_shadow[4]), .C(ev_bit[4]), 
         .Z(n12192)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_256.init = 16'hecec;
    LUT4 i7_4_lut_adj_257 (.A(ev_run_hold[72]), .B(us_tx_c_72), .C(init_shadow[72]), 
         .D(swap_now_d3), .Z(n11002)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_257.init = 16'h5a66;
    LUT4 i7_4_lut_adj_258 (.A(ev_run_hold[73]), .B(us_tx_c_73), .C(init_shadow[73]), 
         .D(swap_now_d3), .Z(n11004)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_258.init = 16'h5a66;
    LUT4 i7_4_lut_adj_259 (.A(ev_run_hold[74]), .B(us_tx_c_74), .C(init_shadow[74]), 
         .D(swap_now_d3), .Z(n11006)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_259.init = 16'h5a66;
    CCU2D add_493_8 (.A0(staging_q[14]), .B0(staging_q[6]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[15]), .B1(staging_q[7]), .C1(GND_net), 
          .D1(GND_net), .CIN(n21789), .COUT(n21790), .S0(build_sum_8__N_2025[6]), 
          .S1(build_sum_8__N_2025[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_493_8.INIT0 = 16'h5666;
    defparam add_493_8.INIT1 = 16'h5666;
    defparam add_493_8.INJECT1_0 = "NO";
    defparam add_493_8.INJECT1_1 = "NO";
    PFUMX i14188 (.BLUT(n22819), .ALUT(n22820), .C0(spi1_miso_N_2421[1]), 
          .Z(n22832));
    PFUMX i14192 (.BLUT(n22827), .ALUT(n22828), .C0(spi1_miso_N_2421[1]), 
          .Z(n22836));
    LUT4 i1_3_lut_4_lut_adj_260 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[10]), 
         .D(ev_bit[10]), .Z(ev_wr_data[10])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_260.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_261 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[11]), 
         .D(ev_bit[11]), .Z(ev_wr_data[11])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_261.init = 16'hddd0;
    LUT4 i7_4_lut_adj_262 (.A(ev_run_hold[75]), .B(us_tx_c_75), .C(init_shadow[75]), 
         .D(swap_now_d3), .Z(n11008)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_262.init = 16'h5a66;
    LUT4 i1_3_lut_adj_263 (.A(n13915), .B(init_shadow[3]), .C(ev_bit[3]), 
         .Z(n12186)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_263.init = 16'hecec;
    PFUMX i14189 (.BLUT(n22821), .ALUT(n22822), .C0(spi1_miso_N_2421[1]), 
          .Z(n22833));
    LUT4 i1_3_lut_adj_264 (.A(n13915), .B(init_shadow[2]), .C(ev_bit[2]), 
         .Z(n12180)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_264.init = 16'hecec;
    LUT4 i1_3_lut_adj_265 (.A(n13915), .B(init_shadow[1]), .C(ev_bit[1]), 
         .Z(n12174)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_265.init = 16'hecec;
    FD1P3IX ev_bit_i29 (.D(ev_bit[28]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i29.GSR = "DISABLED";
    LUT4 i1243_2_lut (.A(ev_ch[1]), .B(ev_ch[0]), .Z(ev_ch_6__N_1920[1])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1243_2_lut.init = 16'h6666;
    LUT4 i13184_1_lut (.A(spi_channel_field[0]), .Z(n15)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(316[78:102])
    defparam i13184_1_lut.init = 16'h5555;
    LUT4 i14252_4_lut (.A(spi1_sck_c_enable_287), .B(n23142), .C(frame_end), 
         .D(fpga_cs_n_c), .Z(spi1_sck_c_enable_326)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i14252_4_lut.init = 16'h0020;
    LUT4 i7_4_lut_adj_266 (.A(ev_run_hold[76]), .B(us_tx_c_76), .C(init_shadow[76]), 
         .D(swap_now_d3), .Z(n11010)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_266.init = 16'h5a66;
    LUT4 i13228_2_lut (.A(staging_q[8]), .B(staging_q[0]), .Z(build_sum_8__N_2025[0])) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i13228_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_267 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[12]), 
         .D(ev_bit[12]), .Z(ev_wr_data[12])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_267.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_268 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[13]), 
         .D(ev_bit[13]), .Z(ev_wr_data[13])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_268.init = 16'hddd0;
    LUT4 i7_4_lut_adj_269 (.A(ev_run_hold[77]), .B(us_tx_c_77), .C(init_shadow[77]), 
         .D(swap_now_d3), .Z(n11012)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_269.init = 16'h5a66;
    LUT4 i14029_4_lut (.A(spi_bitmap[64]), .B(spi_bitmap[25]), .C(spi_bitmap[19]), 
         .D(spi_bitmap[30]), .Z(n22672)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14029_4_lut.init = 16'h8000;
    LUT4 n15_bdd_4_lut (.A(n15_adj_3023), .B(ev_state[2]), .C(ev_state[1]), 
         .D(ev_state[0]), .Z(n23092)) /* synthesis lut_function=(!(A ((C (D)+!C !(D))+!B)+!A ((C (D))+!B))) */ ;
    defparam n15_bdd_4_lut.init = 16'h0cc4;
    LUT4 i1_3_lut_4_lut_adj_270 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[14]), 
         .D(ev_bit[14]), .Z(ev_wr_data[14])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_270.init = 16'hddd0;
    LUT4 ev_state_3__N_1900_2__bdd_4_lut_14354 (.A(ev_state_3__N_1900[2]), 
         .B(ev_state[0]), .C(ev_state[2]), .D(ev_state[1]), .Z(n23094)) /* synthesis lut_function=(!(A (B+(C+(D)))+!A (B+(C)))) */ ;
    defparam ev_state_3__N_1900_2__bdd_4_lut_14354.init = 16'h0103;
    LUT4 i7_4_lut_adj_271 (.A(ev_run_hold[78]), .B(us_tx_c_78), .C(init_shadow[78]), 
         .D(swap_now_d3), .Z(n11014)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_271.init = 16'h5a66;
    LUT4 i14131_4_lut (.A(n22722), .B(n22768), .C(n22750), .D(n22720), 
         .Z(n22774)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14131_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_272 (.A(ev_run_hold[79]), .B(us_tx_c_79), .C(init_shadow[79]), 
         .D(swap_now_d3), .Z(n11016)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_272.init = 16'h5a66;
    LUT4 i2194_4_lut (.A(ev_ch[0]), .B(staging_rd_addr[0]), .C(n15103), 
         .D(n22416), .Z(staging_rd_addr_6__N_785[0])) /* synthesis lut_function=(A (B (C+(D))+!B !(C+!(D)))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i2194_4_lut.init = 16'hcac0;
    LUT4 i13995_2_lut (.A(spi_bitmap[68]), .B(spi_bitmap[16]), .Z(n22638)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13995_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_273 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[15]), 
         .D(ev_bit[15]), .Z(ev_wr_data[15])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_273.init = 16'hddd0;
    LUT4 i13989_2_lut (.A(spi_bitmap[65]), .B(spi_bitmap[76]), .Z(n22632)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13989_2_lut.init = 16'h8888;
    LUT4 i14103_4_lut (.A(spi_bitmap[20]), .B(n22700), .C(n22622), .D(spi_bitmap[26]), 
         .Z(n22746)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14103_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_274 (.A(ev_run_hold[80]), .B(us_tx_c_80), .C(init_shadow[80]), 
         .D(swap_now_d3), .Z(n11018)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_274.init = 16'h5a66;
    LUT4 i78_4_lut (.A(n133), .B(n22754), .C(n91), .D(n22662), .Z(n166)) /* synthesis lut_function=(A+((C+!(D))+!B)) */ ;
    defparam i78_4_lut.init = 16'hfbff;
    LUT4 i14031_4_lut (.A(spi_bitmap[47]), .B(spi_bitmap[66]), .C(spi_bitmap[71]), 
         .D(spi_bitmap[17]), .Z(n22674)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14031_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_275 (.A(ev_run_hold[81]), .B(us_tx_c_81), .C(init_shadow[81]), 
         .D(swap_now_d3), .Z(n11020)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_275.init = 16'h5a66;
    LUT4 i6_4_lut_adj_276 (.A(mic_sample_count[3]), .B(n12_adj_2999), .C(mic_sample_count[4]), 
         .D(mic_clk_c), .Z(pll_clk_enable_492)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i6_4_lut_adj_276.init = 16'h0008;
    LUT4 i7_4_lut_adj_277 (.A(ev_run_hold[82]), .B(us_tx_c_82), .C(init_shadow[82]), 
         .D(swap_now_d3), .Z(n11022)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_277.init = 16'h5a66;
    LUT4 i1_3_lut_4_lut_adj_278 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[16]), 
         .D(ev_bit[16]), .Z(ev_wr_data[16])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_278.init = 16'hddd0;
    LUT4 i5_4_lut_adj_279 (.A(mic_sample_count[0]), .B(mic_sample_count[2]), 
         .C(mic_sample_count[1]), .D(mic_tick), .Z(n12_adj_2999)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i5_4_lut_adj_279.init = 16'h8000;
    LUT4 i1_2_lut_adj_280 (.A(spi_byte_count[0]), .B(n22453), .Z(spi1_sck_c_enable_36)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i1_2_lut_adj_280.init = 16'h8888;
    LUT4 i14055_4_lut (.A(spi_bitmap[56]), .B(spi_bitmap[67]), .C(spi_bitmap[60]), 
         .D(spi_bitmap[6]), .Z(n22698)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14055_4_lut.init = 16'h8000;
    CCU2D add_135_3 (.A0(spi_byte_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_byte_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21798), .COUT(n21799), .S0(spi_byte_count_15__N_1579[1]), 
          .S1(spi_byte_count_15__N_1579[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_3.INIT0 = 16'h5aaa;
    defparam add_135_3.INIT1 = 16'h5aaa;
    defparam add_135_3.INJECT1_0 = "NO";
    defparam add_135_3.INJECT1_1 = "NO";
    LUT4 i14121_4_lut (.A(n22596), .B(n22738), .C(n22688), .D(n22594), 
         .Z(n22764)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14121_4_lut.init = 16'h8000;
    LUT4 i7_4_lut_adj_281 (.A(ev_run_hold[83]), .B(us_tx_c_83), .C(init_shadow[83]), 
         .D(swap_now_d3), .Z(n11024)) /* synthesis lut_function=(!(A (B (C+!(D))+!B (C (D)))+!A !(B (C+!(D))+!B (C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(152[17:28])
    defparam i7_4_lut_adj_281.init = 16'h5a66;
    LUT4 i2_3_lut (.A(swap_now_d1), .B(swap_now), .C(active_bank), .Z(run_bank)) /* synthesis lut_function=(A (B (C)+!B !(C))+!A !(B (C)+!B !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(400[18] 405[12])
    defparam i2_3_lut.init = 16'h9696;
    LUT4 i14099_4_lut (.A(spi_bitmap[45]), .B(n22692), .C(n22604), .D(spi_bitmap[5]), 
         .Z(n22742)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14099_4_lut.init = 16'h8000;
    LUT4 i14053_4_lut (.A(spi_bitmap[12]), .B(spi_bitmap[24]), .C(spi_bitmap[13]), 
         .D(spi_bitmap[27]), .Z(n22696)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14053_4_lut.init = 16'h8000;
    CCU2D add_491_11 (.A0(phase_frac[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_frac[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21814), .COUT(n21815), .S0(phase_frac_sum[9]), 
          .S1(phase_frac_sum[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_11.INIT0 = 16'h5555;
    defparam add_491_11.INIT1 = 16'h5aaa;
    defparam add_491_11.INJECT1_0 = "NO";
    defparam add_491_11.INJECT1_1 = "NO";
    LUT4 mux_1209_i2_3_lut (.A(n9816), .B(n9817), .C(n9813), .Z(rd_data_15__N_2535[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i2_3_lut.init = 16'hcaca;
    FD1P3IX ev_bit_i28 (.D(ev_bit[27]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i28.GSR = "DISABLED";
    CCU2D add_135_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_byte_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21798), .S1(spi_byte_count_15__N_1579[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(324[35:57])
    defparam add_135_1.INIT0 = 16'hF000;
    defparam add_135_1.INIT1 = 16'h5555;
    defparam add_135_1.INJECT1_0 = "NO";
    defparam add_135_1.INJECT1_1 = "NO";
    LUT4 i13953_2_lut (.A(spi_bitmap[80]), .B(spi_bitmap[18]), .Z(n22596)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13953_2_lut.init = 16'h8888;
    LUT4 i14248_4_lut (.A(mic_divider[4]), .B(n10_adj_3003), .C(mic_divider[1]), 
         .D(mic_divider[2]), .Z(mic_tick_N_2463)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(495[21:42])
    defparam i14248_4_lut.init = 16'h1000;
    FD1P3IX ev_bit_i1 (.D(ev_bit[0]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i1.GSR = "DISABLED";
    FD1P3IX ev_bit_i27 (.D(ev_bit[26]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i27.GSR = "DISABLED";
    LUT4 i14095_4_lut (.A(spi_bitmap[57]), .B(n22682), .C(n22584), .D(spi_bitmap[77]), 
         .Z(n22738)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14095_4_lut.init = 16'h8000;
    LUT4 i14045_4_lut (.A(spi_bitmap[34]), .B(spi_bitmap[50]), .C(spi_bitmap[35]), 
         .D(spi_bitmap[52]), .Z(n22688)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14045_4_lut.init = 16'h8000;
    FD1P3IX ev_bit_i26 (.D(ev_bit[25]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i26.GSR = "DISABLED";
    FD1P3IX ev_clear_addr_i0 (.D(ev_clear_addr_7__N_2209[0]), .SP(pll_clk_enable_648), 
            .CD(pll_clk_enable_447), .CK(pll_clk), .Q(ev_clear_addr[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_clear_addr_i0.GSR = "DISABLED";
    FD1P3IX ev_bit_i25 (.D(ev_bit[24]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i25.GSR = "DISABLED";
    LUT4 mux_1209_i3_3_lut (.A(n9818), .B(n9819), .C(n9813), .Z(rd_data_15__N_2535[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i3_3_lut.init = 16'hcaca;
    FD1P3IX init_shadow_i0 (.D(n10843), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i0.GSR = "DISABLED";
    FD1P3IX ev_bit_i24 (.D(ev_bit[23]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i24.GSR = "DISABLED";
    FD1P3AX ev_state_i0 (.D(ev_state_3__N_581[0]), .SP(pll_clk_enable_652), 
            .CK(pll_clk), .Q(ev_state[0])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_state_i0.GSR = "DISABLED";
    FD1P3IX ev_bit_i23 (.D(ev_bit[22]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i23.GSR = "DISABLED";
    FD1S3AX mem_1192 (.D(staging_rd_addr_6__N_785[6]), .CK(pll_clk), .Q(n9810));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam mem_1192.GSR = "DISABLED";
    FD1P3IX ev_bit_i22 (.D(ev_bit[21]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i22.GSR = "DISABLED";
    LUT4 i1_3_lut_4_lut_adj_282 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[17]), 
         .D(ev_bit[17]), .Z(ev_wr_data[17])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_282.init = 16'hddd0;
    LUT4 mux_1209_i4_3_lut (.A(n9820), .B(n9821), .C(n9813), .Z(rd_data_15__N_2535[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i4_3_lut.init = 16'hcaca;
    LUT4 n15_bdd_4_lut_14351 (.A(ev_state_3__N_1900[2]), .B(ev_state[2]), 
         .C(ev_state[1]), .D(ev_state[0]), .Z(n23091)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;
    defparam n15_bdd_4_lut_14351.init = 16'h0020;
    CCU2D global_phase_7__I_0_438_8 (.A0(global_phase[6]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[7]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21784), .S0(next_phase[6]), 
          .S1(next_phase[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[34:67])
    defparam global_phase_7__I_0_438_8.INIT0 = 16'h5aaa;
    defparam global_phase_7__I_0_438_8.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_438_8.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_438_8.INJECT1_1 = "NO";
    LUT4 i4_4_lut_adj_283 (.A(mic_divider[6]), .B(mic_divider[5]), .C(mic_divider[0]), 
         .D(mic_divider[3]), .Z(n10_adj_3003)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut_adj_283.init = 16'hfffe;
    LUT4 mic_clk_I_0_2_lut (.A(mic_clk_c), .B(mic_tick), .Z(mic_clk_N_2428)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(507[18] 509[12])
    defparam mic_clk_I_0_2_lut.init = 16'h6666;
    LUT4 i1_3_lut_4_lut_adj_284 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[18]), 
         .D(ev_bit[18]), .Z(ev_wr_data[18])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_284.init = 16'hddd0;
    CCU2D global_phase_7__I_0_438_6 (.A0(global_phase[4]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(global_phase[5]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21783), .COUT(n21784), .S0(next_phase[4]), 
          .S1(next_phase[5]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(157[34:67])
    defparam global_phase_7__I_0_438_6.INIT0 = 16'h5aaa;
    defparam global_phase_7__I_0_438_6.INIT1 = 16'h5aaa;
    defparam global_phase_7__I_0_438_6.INJECT1_0 = "NO";
    defparam global_phase_7__I_0_438_6.INJECT1_1 = "NO";
    LUT4 i2_4_lut (.A(n22760), .B(n22409), .C(n22714), .D(n36_adj_3021), 
         .Z(n6_adj_3025)) /* synthesis lut_function=(!(A+((C+!(D))+!B))) */ ;
    defparam i2_4_lut.init = 16'h0400;
    LUT4 i14117_4_lut (.A(spi_version[5]), .B(n22716), .C(n22550), .D(spi_version[7]), 
         .Z(n22760)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14117_4_lut.init = 16'hfffe;
    LUT4 i14071_4_lut (.A(spi_version[1]), .B(spi_extension_length[7]), 
         .C(spi_rx_shift[6]), .D(spi_rx_shift[4]), .Z(n22714)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14071_4_lut.init = 16'hfffe;
    LUT4 i16_4_lut (.A(spi_rx_shift[0]), .B(n22762), .C(n22734), .D(spi_version[0]), 
         .Z(n36_adj_3021)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam i16_4_lut.init = 16'h0100;
    LUT4 i14073_4_lut (.A(spi_version[3]), .B(spi1_mosi_c_0), .C(spi_version[6]), 
         .D(spi_rx_shift[1]), .Z(n22716)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14073_4_lut.init = 16'hfffe;
    LUT4 mux_1209_i5_3_lut (.A(n9822), .B(n9823), .C(n9813), .Z(rd_data_15__N_2535[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i5_3_lut.init = 16'hcaca;
    LUT4 i13951_2_lut (.A(spi_bitmap[48]), .B(spi_bitmap[28]), .Z(n22594)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13951_2_lut.init = 16'h8888;
    LUT4 i13907_2_lut (.A(spi_rx_shift[2]), .B(spi_rx_shift[5]), .Z(n22550)) /* synthesis lut_function=(A+(B)) */ ;
    defparam i13907_2_lut.init = 16'heeee;
    LUT4 mux_1209_i6_3_lut (.A(n9824), .B(n9825), .C(n9813), .Z(rd_data_15__N_2535[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i6_3_lut.init = 16'hcaca;
    LUT4 i14079_4_lut (.A(spi_bitmap[15]), .B(spi_bitmap[36]), .C(spi_bitmap[29]), 
         .D(spi_bitmap[39]), .Z(n22722)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14079_4_lut.init = 16'h8000;
    LUT4 i14125_4_lut (.A(n22632), .B(n22746), .C(n22704), .D(n22630), 
         .Z(n22768)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14125_4_lut.init = 16'h8000;
    LUT4 i14107_4_lut (.A(spi_bitmap[79]), .B(n22708), .C(n22638), .D(spi_bitmap[4]), 
         .Z(n22750)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14107_4_lut.init = 16'h8000;
    LUT4 i14077_4_lut (.A(spi_bitmap[58]), .B(spi_bitmap[69]), .C(spi_bitmap[63]), 
         .D(spi_bitmap[78]), .Z(n22720)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14077_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_285 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[20]), 
         .D(ev_bit[20]), .Z(ev_wr_data[20])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_285.init = 16'hddd0;
    INV i14473 (.A(spi_mic_sck_c), .Z(sck_N_2937));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(70[24:35])
    LUT4 i14091_3_lut (.A(spi_version[2]), .B(n22467), .C(spi_extension_length[5]), 
         .Z(n22734)) /* synthesis lut_function=(A+(B (C))) */ ;
    defparam i14091_3_lut.init = 16'heaea;
    CCU2D expected_next_15__I_0_488_11 (.A0(spi_rx_shift[2]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[3]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21834), .COUT(n21835), .S0(expected_next[11]), 
          .S1(expected_next[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_11.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_488_11.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_488_11.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_11.INJECT1_1 = "NO";
    LUT4 i14065_4_lut (.A(spi_bitmap[33]), .B(spi_bitmap[49]), .C(spi_bitmap[46]), 
         .D(spi_bitmap[0]), .Z(n22708)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14065_4_lut.init = 16'h8000;
    VLO i1 (.Z(GND_net));
    LUT4 i1_3_lut_4_lut_adj_286 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[19]), 
         .D(ev_bit[19]), .Z(ev_wr_data[19])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_286.init = 16'hddd0;
    LUT4 i14119_4_lut (.A(spi_version[4]), .B(spi_extension_length[6]), 
         .C(spi_rx_shift[3]), .D(n22668), .Z(n22762)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14119_4_lut.init = 16'hfffe;
    TSALL TSALL_INST (.TSALL(GND_net));
    LUT4 mux_1209_i9_3_lut (.A(n9830), .B(n9831), .C(n9813), .Z(rd_data_15__N_2535[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i9_3_lut.init = 16'hcaca;
    CCU2D fpga_time_1091_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21853), .COUT(n21854), .S0(n148), .S1(n147));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_19.INJECT1_1 = "NO";
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    ws2812_stream ws2812_i (.GND_net(GND_net), .rgb_hold({rgb_hold}), .pll_clk(pll_clk), 
            .rgb_data_c(rgb_data_c)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(514[19] 521[6])
    LUT4 mux_1209_i7_3_lut (.A(n9826), .B(n9827), .C(n9813), .Z(rd_data_15__N_2535[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[41:53])
    defparam mux_1209_i7_3_lut.init = 16'hcaca;
    CCU2D add_493_4 (.A0(staging_q[10]), .B0(staging_q[2]), .C0(GND_net), 
          .D0(GND_net), .A1(staging_q[11]), .B1(staging_q[3]), .C1(GND_net), 
          .D1(GND_net), .CIN(n21787), .COUT(n21788), .S0(build_sum_8__N_2025[2]), 
          .S1(build_sum_8__N_2025[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(436[32:80])
    defparam add_493_4.INIT0 = 16'h5666;
    defparam add_493_4.INIT1 = 16'h5666;
    defparam add_493_4.INJECT1_0 = "NO";
    defparam add_493_4.INJECT1_1 = "NO";
    FD1S3AX spi_bit_count_1090__i2 (.D(n18), .CK(spi1_sck_c), .Q(spi_bit_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(340[34:54])
    defparam spi_bit_count_1090__i2.GSR = "ENABLED";
    FD1P3IX ev_bit_i21 (.D(ev_bit[20]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i21.GSR = "DISABLED";
    FD1P3IX ev_bit_i20 (.D(ev_bit[19]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i20.GSR = "DISABLED";
    FD1P3IX ev_bit_i19 (.D(ev_bit[18]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i19.GSR = "DISABLED";
    FD1P3IX ev_bit_i18 (.D(ev_bit[17]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i18.GSR = "DISABLED";
    FD1P3IX ev_bit_i17 (.D(ev_bit[16]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i17.GSR = "DISABLED";
    FD1P3IX ev_bit_i16 (.D(ev_bit[15]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i16.GSR = "DISABLED";
    FD1P3IX ev_bit_i15 (.D(ev_bit[14]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i15.GSR = "DISABLED";
    FD1P3IX ev_bit_i14 (.D(ev_bit[13]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i14.GSR = "DISABLED";
    LUT4 i2_4_lut_4_lut (.A(spi_byte_count[2]), .B(spi_byte_count[1]), .C(n4), 
         .D(spi_byte_count[3]), .Z(n1)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i2_4_lut_4_lut.init = 16'h1080;
    LUT4 i1_3_lut_4_lut_adj_287 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[22]), 
         .D(ev_bit[22]), .Z(ev_wr_data[22])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_287.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_288 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[23]), 
         .D(ev_bit[23]), .Z(ev_wr_data[23])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_288.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_289 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[24]), 
         .D(ev_bit[24]), .Z(ev_wr_data[24])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_289.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_290 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[25]), 
         .D(ev_bit[25]), .Z(ev_wr_data[25])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_290.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_291 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[26]), 
         .D(ev_bit[26]), .Z(ev_wr_data[26])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_291.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_292 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[27]), 
         .D(ev_bit[27]), .Z(ev_wr_data[27])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_292.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_293 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[28]), 
         .D(ev_bit[28]), .Z(ev_wr_data[28])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_293.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_294 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[29]), 
         .D(ev_bit[29]), .Z(ev_wr_data[29])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_294.init = 16'hddd0;
    LUT4 i14061_4_lut (.A(spi_bitmap[42]), .B(spi_bitmap[83]), .C(spi_bitmap[75]), 
         .D(spi_bitmap[10]), .Z(n22704)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14061_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_295 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[30]), 
         .D(ev_bit[30]), .Z(ev_wr_data[30])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_295.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_296 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[31]), 
         .D(ev_bit[31]), .Z(ev_wr_data[31])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_296.init = 16'hddd0;
    LUT4 i14284_3_lut_4_lut_4_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[0]), 
         .D(ev_state[1]), .Z(ev_we)) /* synthesis lut_function=(!(A (B+(C+!(D)))+!A ((D)+!C))) */ ;
    defparam i14284_3_lut_4_lut_4_lut.init = 16'h0250;
    LUT4 i1_3_lut_4_lut_adj_297 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[32]), 
         .D(ev_bit[32]), .Z(ev_wr_data[32])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_297.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_298 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[33]), 
         .D(ev_bit[33]), .Z(ev_wr_data[33])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_298.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_299 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[34]), 
         .D(ev_bit[34]), .Z(ev_wr_data[34])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_299.init = 16'hddd0;
    LUT4 i1_4_lut_else_4_lut (.A(status_bit_index[0]), .B(status_bit_index[2]), 
         .C(status_hold[76]), .D(status_bit_index[1]), .Z(n23206)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(126[17:33])
    defparam i1_4_lut_else_4_lut.init = 16'h2000;
    LUT4 i1_3_lut_4_lut_adj_300 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[35]), 
         .D(ev_bit[35]), .Z(ev_wr_data[35])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_300.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_301 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[75]), 
         .D(ev_bit[75]), .Z(ev_wr_data[75])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_301.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_302 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[76]), 
         .D(ev_bit[76]), .Z(ev_wr_data[76])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_302.init = 16'hddd0;
    LUT4 i13987_2_lut (.A(spi_bitmap[32]), .B(spi_bitmap[37]), .Z(n22630)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13987_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_303 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[77]), 
         .D(ev_bit[77]), .Z(ev_wr_data[77])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_303.init = 16'hddd0;
    LUT4 i14057_4_lut (.A(spi_bitmap[53]), .B(spi_bitmap[74]), .C(spi_bitmap[61]), 
         .D(spi_bitmap[38]), .Z(n22700)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14057_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_4_lut_adj_304 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[78]), 
         .D(ev_bit[78]), .Z(ev_wr_data[78])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_304.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_305 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[79]), 
         .D(ev_bit[79]), .Z(ev_wr_data[79])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_305.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_306 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[80]), 
         .D(ev_bit[80]), .Z(ev_wr_data[80])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_306.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_307 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[81]), 
         .D(ev_bit[81]), .Z(ev_wr_data[81])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_307.init = 16'hddd0;
    LUT4 i1_2_lut_3_lut_4_lut_rep_121 (.A(ev_state[2]), .B(n23198), .C(n23171), 
         .D(n23194), .Z(pll_clk_enable_759)) /* synthesis lut_function=(!(A (C+!(D))+!A !(B+!(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i1_2_lut_3_lut_4_lut_rep_121.init = 16'h4f44;
    LUT4 i1_3_lut_adj_308 (.A(n13915), .B(init_shadow[83]), .C(ev_bit[83]), 
         .Z(n12745)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_308.init = 16'hecec;
    LUT4 i1_3_lut_4_lut_adj_309 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[82]), 
         .D(ev_bit[82]), .Z(ev_wr_data[82])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_309.init = 16'hddd0;
    LUT4 i1_3_lut_4_lut_adj_310 (.A(ev_state[0]), .B(n23163), .C(ev_rd_hold[83]), 
         .D(ev_bit[83]), .Z(ev_wr_data[83])) /* synthesis lut_function=(A (B (C+(D)))+!A (C+(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(211[29:51])
    defparam i1_3_lut_4_lut_adj_310.init = 16'hddd0;
    LUT4 i2_4_lut_then_4_lut (.A(ev_state[2]), .B(ev_state_3__N_1912[1]), 
         .C(ev_state[3]), .D(ev_state[1]), .Z(n23210)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i2_4_lut_then_4_lut.init = 16'hfffb;
    LUT4 i2_4_lut_else_4_lut (.A(ev_state[2]), .B(ev_state[3]), .C(ev_state[1]), 
         .Z(n23209)) /* synthesis lut_function=(A+(B+!(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(407[9] 481[16])
    defparam i2_4_lut_else_4_lut.init = 16'hefef;
    LUT4 i3_4_lut_rep_123 (.A(n55), .B(n23152), .C(spi_byte_count[4]), 
         .D(n22580), .Z(spi1_sck_c_enable_139)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i3_4_lut_rep_123.init = 16'h0080;
    LUT4 i4_4_lut_rep_125 (.A(n7_adj_3024), .B(n23155), .C(expected_next_15__N_1342[3]), 
         .D(n25), .Z(spi1_sck_c_enable_226)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i4_4_lut_rep_125.init = 16'h8000;
    LUT4 i13979_2_lut (.A(spi_bitmap[31]), .B(spi_bitmap[51]), .Z(n22622)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13979_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_rep_71 (.A(spi_command[0]), .B(n22409), .C(spi_command[1]), 
         .Z(n23153)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B (C)))) */ ;
    defparam i1_3_lut_rep_71.init = 16'h4848;
    LUT4 i45_4_lut (.A(spi_bitmap[87]), .B(spi_bitmap[84]), .C(spi_bitmap[85]), 
         .D(spi_bitmap[14]), .Z(n133)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i45_4_lut.init = 16'hfeff;
    LUT4 n12_bdd_4_lut (.A(n12), .B(n21924), .C(spi_byte_count[6]), .D(spi_byte_count[7]), 
         .Z(n23205)) /* synthesis lut_function=(!(A (B (C (D)))+!A (B (C (D)+!C !(D))+!B !(C+(D))))) */ ;
    defparam n12_bdd_4_lut.init = 16'h3ffa;
    LUT4 i14260_2_lut_2_lut_4_lut_4_lut (.A(spi_command[0]), .B(n22409), 
         .C(spi_command[1]), .D(n6_adj_3025), .Z(invalid_frame_spi_N_2450)) /* synthesis lut_function=(A ((C)+!B)+!A !(B (C+(D))+!B !(C+!(D)))) */ ;
    defparam i14260_2_lut_2_lut_4_lut_4_lut.init = 16'hb2b7;
    LUT4 i1_2_lut_4_lut_adj_311 (.A(spi_command[0]), .B(n22409), .C(spi_command[1]), 
         .D(stop_toggle_spi), .Z(stop_toggle_spi_N_2443)) /* synthesis lut_function=(A (B (C (D)+!C !(D))+!B (D))+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;
    defparam i1_2_lut_4_lut_adj_311.init = 16'hb748;
    LUT4 i1_3_lut_4_lut_rep_127 (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .C(n23181), .D(frame_settle[0]), .Z(pll_clk_enable_172)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(382[13:51])
    defparam i1_3_lut_4_lut_rep_127.init = 16'h0900;
    LUT4 i2_4_lut_adj_312 (.A(phase_step_d1), .B(phase_step_reg), .C(n21947), 
         .D(n23160), .Z(n15_adj_3023)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i2_4_lut_adj_312.init = 16'hfeee;
    LUT4 i14111_4_lut (.A(spi_bitmap[82]), .B(n22726), .C(n22658), .D(spi_bitmap[7]), 
         .Z(n22754)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14111_4_lut.init = 16'h8000;
    LUT4 i3_2_lut (.A(spi_bitmap[86]), .B(spi_bitmap[62]), .Z(n91)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam i3_2_lut.init = 16'hbbbb;
    LUT4 i2_3_lut_4_lut_rep_129 (.A(status_flags_wire_15__N_1285[4]), .B(pll_locked), 
         .C(pll_clk_enable_1), .D(phase_step_d3), .Z(pll_clk_enable_268)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;
    defparam i2_3_lut_4_lut_rep_129.init = 16'hfff7;
    LUT4 i14019_2_lut (.A(spi_bitmap[41]), .B(spi_bitmap[72]), .Z(n22662)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14019_2_lut.init = 16'h8888;
    LUT4 ev_state_1__bdd_4_lut_rep_131 (.A(ev_state[1]), .B(ev_state[2]), 
         .C(ev_state[0]), .D(ev_state[3]), .Z(pll_clk_enable_382)) /* synthesis lut_function=(!(A+(B ((D)+!C)+!B (C+!(D))))) */ ;
    defparam ev_state_1__bdd_4_lut_rep_131.init = 16'h0140;
    LUT4 i14083_4_lut (.A(spi_bitmap[81]), .B(spi_bitmap[54]), .C(spi_bitmap[40]), 
         .D(spi_bitmap[59]), .Z(n22726)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14083_4_lut.init = 16'h8000;
    umh_toggle_ram84 event_ram (.pll_clk(pll_clk), .ev_we(ev_we), .VCC_net(VCC_net), 
            .GND_net(GND_net), .\ev_wr_addr[0] (ev_wr_addr[0]), .event_rd_addr({event_rd_addr}), 
            .\ev_wr_addr[1] (ev_wr_addr[1]), .\ev_wr_addr[2] (ev_wr_addr[2]), 
            .\ev_wr_addr[3] (ev_wr_addr[3]), .\ev_wr_addr[4] (ev_wr_addr[4]), 
            .\ev_wr_addr[5] (ev_wr_addr[5]), .\ev_wr_addr[6] (ev_wr_addr[6]), 
            .\ev_wr_addr[7] (ev_wr_addr[7]), .n23200(n23200), .ev_wr_data({ev_wr_data}), 
            .ev_rd_data({ev_rd_data})) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(219[22] 222[6])
    LUT4 i1_4_lut_rep_133 (.A(n23194), .B(ev_state[3]), .C(ev_state[2]), 
         .D(n23201), .Z(pll_clk_enable_715)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C)))) */ ;
    defparam i1_4_lut_rep_133.init = 16'h3032;
    LUT4 i1_2_lut_3_lut_4_lut_adj_313 (.A(spi1_sck_c_enable_287), .B(fpga_cs_n_c), 
         .C(n23142), .D(frame_end), .Z(spi1_sck_c_enable_318)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i1_2_lut_3_lut_4_lut_adj_313.init = 16'h2000;
    LUT4 i6541_2_lut_3_lut_4_lut_rep_135 (.A(ev_state[2]), .B(n23198), .C(n23171), 
         .D(n23194), .Z(n23447)) /* synthesis lut_function=(!(A (C+!(D))+!A (B+(C+!(D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam i6541_2_lut_3_lut_4_lut_rep_135.init = 16'h0b00;
    LUT4 i14015_2_lut (.A(spi_bitmap[9]), .B(spi_bitmap[55]), .Z(n22658)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14015_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_4_lut_rep_137 (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .C(pll_locked), .D(status_flags_wire_15__N_1285[4]), .Z(n23449)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(237[23:61])
    defparam i1_2_lut_3_lut_4_lut_rep_137.init = 16'h6fff;
    LUT4 i6456_2_lut_4_lut_rep_139 (.A(n23194), .B(ev_state[3]), .C(ev_state[2]), 
         .D(n23201), .Z(n23451)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i6456_2_lut_4_lut_rep_139.init = 16'h0002;
    LUT4 i14049_4_lut (.A(spi_bitmap[43]), .B(spi_bitmap[70]), .C(spi_bitmap[21]), 
         .D(spi_bitmap[73]), .Z(n22692)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14049_4_lut.init = 16'h8000;
    LUT4 i1_3_lut_adj_314 (.A(n13915), .B(init_shadow[82]), .C(ev_bit[82]), 
         .Z(n12739)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_314.init = 16'hecec;
    LUT4 i13961_2_lut (.A(spi_bitmap[22]), .B(spi_bitmap[44]), .Z(n22604)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13961_2_lut.init = 16'h8888;
    LUT4 i13221_3_lut_4_lut (.A(mic_sample_count[2]), .B(n23169), .C(mic_sample_count[3]), 
         .D(mic_sample_count[4]), .Z(n26)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam i13221_3_lut_4_lut.init = 16'h7f80;
    LUT4 i1_3_lut_adj_315 (.A(n13915), .B(init_shadow[81]), .C(ev_bit[81]), 
         .Z(n12733)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_315.init = 16'hecec;
    LUT4 i2_3_lut_rep_75 (.A(frame_req), .B(n23171), .C(swap_pending), 
         .Z(n23157)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut_rep_75.init = 16'hfefe;
    LUT4 i14039_4_lut (.A(spi_bitmap[23]), .B(spi_bitmap[2]), .C(spi_bitmap[1]), 
         .D(spi_bitmap[11]), .Z(n22682)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14039_4_lut.init = 16'h8000;
    LUT4 i10388_1_lut_3_lut (.A(frame_req), .B(n23171), .C(swap_pending), 
         .Z(fifo_credit_wire[0])) /* synthesis lut_function=(!(A+(B+(C)))) */ ;
    defparam i10388_1_lut_3_lut.init = 16'h0101;
    LUT4 i13941_2_lut (.A(spi_bitmap[3]), .B(spi_bitmap[8]), .Z(n22584)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13941_2_lut.init = 16'h8888;
    LUT4 i1_3_lut_adj_316 (.A(n13915), .B(init_shadow[80]), .C(ev_bit[80]), 
         .Z(n12727)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_316.init = 16'hecec;
    LUT4 i1_3_lut_adj_317 (.A(n13915), .B(init_shadow[79]), .C(ev_bit[79]), 
         .Z(n12721)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_317.init = 16'hecec;
    LUT4 ev_state_3__I_0_441_Mux_1_i7_4_lut_4_lut (.A(ev_state_3__N_1912[1]), 
         .B(ev_state[1]), .C(ev_state[0]), .D(ev_state[2]), .Z(n7_adj_2989)) /* synthesis lut_function=(!(A (B (C (D))+!B !(C))+!A (B (C (D))+!B !(C (D))))) */ ;
    defparam ev_state_3__I_0_441_Mux_1_i7_4_lut_4_lut.init = 16'h3cec;
    LUT4 build_phase_7__I_0_i3_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[2]), .D(build_sum[2]), .Z(ev_rd_slot[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 run_addr_reg_8__I_0_i8_3_lut (.A(run_addr_reg[7]), .B(ev_rd_slot[7]), 
         .C(n15_adj_3023), .Z(event_rd_addr[7])) /* synthesis lut_function=(A (B+(C))+!A !((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(209[33:75])
    defparam run_addr_reg_8__I_0_i8_3_lut.init = 16'hacac;
    LUT4 build_phase_7__I_0_i2_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[1]), .D(build_sum[1]), .Z(ev_rd_slot[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i2_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i53 (.D(n12486), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[53])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i53.GSR = "DISABLED";
    FD1P3IX init_shadow_i52 (.D(n12480), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[52])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i52.GSR = "DISABLED";
    FD1P3IX init_shadow_i51 (.D(n12474), .SP(pll_clk_enable_666), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[51])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i51.GSR = "DISABLED";
    FD1P3IX init_shadow_i50 (.D(n12468), .SP(pll_clk_enable_666), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[50])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i50.GSR = "DISABLED";
    FD1P3IX init_shadow_i49 (.D(n12462), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[49])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i49.GSR = "DISABLED";
    FD1P3IX init_shadow_i48 (.D(n12456), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[48])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i48.GSR = "DISABLED";
    FD1S3IX mic_divider_1094__i1 (.D(n39_adj_2992), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094__i1.GSR = "DISABLED";
    FD1S3IX mic_divider_1094__i2 (.D(n38_adj_2993), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094__i2.GSR = "DISABLED";
    FD1S3IX mic_divider_1094__i3 (.D(n37_adj_2994), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094__i3.GSR = "DISABLED";
    FD1S3IX mic_divider_1094__i4 (.D(n36_adj_2995), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094__i4.GSR = "DISABLED";
    FD1S3IX mic_divider_1094__i5 (.D(n35_adj_2996), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094__i5.GSR = "DISABLED";
    FD1S3IX mic_divider_1094__i6 (.D(n34_adj_2997), .CK(pll_clk), .CD(mic_tick), 
            .Q(mic_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(508[28:46])
    defparam mic_divider_1094__i6.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1093__i1 (.D(n29), .SP(mic_tick), .CK(pll_clk), 
            .Q(mic_sample_count[1]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam mic_sample_count_1093__i1.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1093__i2 (.D(n28), .SP(mic_tick), .CK(pll_clk), 
            .Q(mic_sample_count[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam mic_sample_count_1093__i2.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1093__i3 (.D(n27), .SP(mic_tick), .CK(pll_clk), 
            .Q(mic_sample_count[3]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam mic_sample_count_1093__i3.GSR = "DISABLED";
    FD1P3AX mic_sample_count_1093__i4 (.D(n26), .SP(mic_tick), .CK(pll_clk), 
            .Q(mic_sample_count[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(502[37:60])
    defparam mic_sample_count_1093__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1092__i1 (.D(n39), .CK(pll_clk), .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092__i1.GSR = "DISABLED";
    FD1S3AX time_divider_1092__i2 (.D(n38), .CK(pll_clk), .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092__i2.GSR = "DISABLED";
    FD1S3AX time_divider_1092__i3 (.D(n37), .CK(pll_clk), .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092__i3.GSR = "DISABLED";
    FD1S3AX time_divider_1092__i4 (.D(n36), .CK(pll_clk), .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092__i4.GSR = "DISABLED";
    FD1S3AX time_divider_1092__i5 (.D(n35), .CK(pll_clk), .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092__i5.GSR = "DISABLED";
    FD1S3AX time_divider_1092__i6 (.D(n34), .CK(pll_clk), .Q(time_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(368[29:48])
    defparam time_divider_1092__i6.GSR = "DISABLED";
    CCU2D fpga_time_1091_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21852), .COUT(n21853), .S0(n150), .S1(n149));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_17.INJECT1_1 = "NO";
    LUT4 build_phase_7__I_0_i1_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[0]), .D(build_sum[0]), .Z(ev_rd_slot[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 build_phase_7__I_0_i4_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[3]), .D(build_sum[3]), .Z(ev_rd_slot[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 build_phase_7__I_0_i5_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[4]), .D(build_sum[4]), .Z(ev_rd_slot[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i5_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i47 (.D(n12450), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[47])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i47.GSR = "DISABLED";
    FD1P3IX init_shadow_i46 (.D(n12444), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[46])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i46.GSR = "DISABLED";
    FD1P3IX init_shadow_i45 (.D(n12438), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[45])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i45.GSR = "DISABLED";
    FD1P3IX init_shadow_i44 (.D(n12432), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[44])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i44.GSR = "DISABLED";
    FD1P3IX init_shadow_i43 (.D(n12426), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[43])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i43.GSR = "DISABLED";
    FD1P3IX init_shadow_i42 (.D(n12420), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[42])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i42.GSR = "DISABLED";
    FD1P3IX init_shadow_i41 (.D(n12414), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[41])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i41.GSR = "DISABLED";
    FD1P3IX init_shadow_i40 (.D(n12408), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[40])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i40.GSR = "DISABLED";
    FD1P3IX init_shadow_i39 (.D(n12402), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[39])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i39.GSR = "DISABLED";
    FD1P3IX init_shadow_i38 (.D(n12396), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[38])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i38.GSR = "DISABLED";
    FD1P3IX init_shadow_i37 (.D(n12390), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[37])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i37.GSR = "DISABLED";
    FD1P3IX init_shadow_i36 (.D(n12384), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[36])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i36.GSR = "DISABLED";
    FD1P3IX init_shadow_i35 (.D(n12378), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[35])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i35.GSR = "DISABLED";
    FD1P3IX init_shadow_i34 (.D(n12372), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[34])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i34.GSR = "DISABLED";
    PFUMX i14374 (.BLUT(n23209), .ALUT(n23210), .C0(ev_state[0]), .Z(n15103));
    FD1P3IX init_shadow_i33 (.D(n12366), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[33])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i33.GSR = "DISABLED";
    FD1P3IX init_shadow_i32 (.D(n12360), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[32])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i32.GSR = "DISABLED";
    FD1P3IX init_shadow_i31 (.D(n12354), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[31])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i31.GSR = "DISABLED";
    FD1P3IX init_shadow_i30 (.D(n12348), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[30])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i30.GSR = "DISABLED";
    FD1P3IX init_shadow_i29 (.D(n12342), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[29])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i29.GSR = "DISABLED";
    FD1P3IX init_shadow_i28 (.D(n12336), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[28])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i28.GSR = "DISABLED";
    FD1P3IX init_shadow_i27 (.D(n12330), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[27])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i27.GSR = "DISABLED";
    FD1P3IX init_shadow_i26 (.D(n12324), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[26])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i26.GSR = "DISABLED";
    FD1P3IX init_shadow_i25 (.D(n12318), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[25])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i25.GSR = "DISABLED";
    FD1P3IX init_shadow_i24 (.D(n12312), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[24])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i24.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i6_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[5]), .D(build_sum[5]), .Z(ev_rd_slot[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i6_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i23 (.D(n12306), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[23])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i23.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i7_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[6]), .D(build_sum[6]), .Z(ev_rd_slot[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i7_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i22 (.D(n12300), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[22])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i22.GSR = "DISABLED";
    FD1P3IX init_shadow_i21 (.D(n12294), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[21])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i21.GSR = "DISABLED";
    FD1P3IX init_shadow_i20 (.D(n12288), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[20])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i20.GSR = "DISABLED";
    FD1P3IX init_shadow_i19 (.D(n12282), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[19])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i19.GSR = "DISABLED";
    FD1P3IX init_shadow_i18 (.D(n12276), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[18])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i18.GSR = "DISABLED";
    FD1P3IX init_shadow_i17 (.D(n12270), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[17])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i17.GSR = "DISABLED";
    FD1P3IX init_shadow_i16 (.D(n12264), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[16])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i16.GSR = "DISABLED";
    FD1P3IX init_shadow_i15 (.D(n12258), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[15])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i15.GSR = "DISABLED";
    FD1P3IX init_shadow_i14 (.D(n12252), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[14])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i14.GSR = "DISABLED";
    FD1P3IX init_shadow_i13 (.D(n12246), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i13.GSR = "DISABLED";
    FD1P3IX init_shadow_i12 (.D(n12240), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[12])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i12.GSR = "DISABLED";
    FD1P3IX init_shadow_i11 (.D(n12234), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[11])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i11.GSR = "DISABLED";
    FD1P3IX init_shadow_i10 (.D(n12228), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[10])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i10.GSR = "DISABLED";
    FD1P3IX init_shadow_i9 (.D(n12222), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[9])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i9.GSR = "DISABLED";
    FD1P3IX init_shadow_i8 (.D(n12216), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[8])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i8.GSR = "DISABLED";
    FD1P3IX init_shadow_i7 (.D(n12210), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[7])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i7.GSR = "DISABLED";
    FD1P3IX init_shadow_i6 (.D(n12204), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i6.GSR = "DISABLED";
    FD1P3IX init_shadow_i5 (.D(n12198), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i5.GSR = "DISABLED";
    FD1P3IX init_shadow_i4 (.D(n12192), .SP(pll_clk_enable_715), .CD(n15297), 
            .CK(pll_clk), .Q(init_shadow[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i4.GSR = "DISABLED";
    LUT4 build_phase_7__I_0_i8_3_lut_4_lut (.A(ev_state[0]), .B(n23175), 
         .C(build_phase[7]), .D(build_sum[7]), .Z(ev_rd_slot[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(205[33:53])
    defparam build_phase_7__I_0_i8_3_lut_4_lut.init = 16'hf2d0;
    FD1P3IX init_shadow_i3 (.D(n12186), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i3.GSR = "DISABLED";
    FD1P3IX init_shadow_i2 (.D(n12180), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i2.GSR = "DISABLED";
    FD1P3IX init_shadow_i1 (.D(n12174), .SP(pll_clk_enable_715), .CD(n23451), 
            .CK(pll_clk), .Q(init_shadow[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam init_shadow_i1.GSR = "DISABLED";
    FD1P3IX ev_ch_i6 (.D(ev_ch_6__N_1920[6]), .SP(pll_clk_enable_721), .CD(n18687), 
            .CK(pll_clk), .Q(ev_ch[6])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_ch_i6.GSR = "DISABLED";
    LUT4 frame_end_I_0_4_lut (.A(n22466), .B(frame_end_N_2499), .C(n9), 
         .D(n10), .Z(frame_end)) /* synthesis lut_function=(A (B (C+(D)))+!A (B+!(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(275[29] 276[95])
    defparam frame_end_I_0_4_lut.init = 16'hccc5;
    FD1P3IX ev_ch_i5 (.D(ev_ch_6__N_1920[5]), .SP(pll_clk_enable_721), .CD(n18687), 
            .CK(pll_clk), .Q(ev_ch[5])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_ch_i5.GSR = "DISABLED";
    FD1P3IX ev_ch_i4 (.D(ev_ch_6__N_1920[4]), .SP(pll_clk_enable_721), .CD(n18687), 
            .CK(pll_clk), .Q(ev_ch[4])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_ch_i4.GSR = "DISABLED";
    FD1P3IX ev_ch_i3 (.D(ev_ch_6__N_1920[3]), .SP(pll_clk_enable_721), .CD(n18687), 
            .CK(pll_clk), .Q(ev_ch[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_ch_i3.GSR = "DISABLED";
    FD1P3IX ev_ch_i2 (.D(ev_ch_6__N_1920[2]), .SP(pll_clk_enable_721), .CD(n18687), 
            .CK(pll_clk), .Q(ev_ch[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_ch_i2.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_318 (.A(n13915), .B(init_shadow[78]), .C(ev_bit[78]), 
         .Z(n12715)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_318.init = 16'hecec;
    FD1P3IX ev_ch_i1 (.D(ev_ch_6__N_1920[1]), .SP(pll_clk_enable_721), .CD(n18687), 
            .CK(pll_clk), .Q(ev_ch[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_ch_i1.GSR = "DISABLED";
    LUT4 i13_4_lut (.A(n22582), .B(n26_adj_3026), .C(n22), .D(n22465), 
         .Z(n22466)) /* synthesis lut_function=((B+(C+(D)))+!A) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i13_4_lut.init = 16'hfffd;
    FD1P3AX ev_state_i3 (.D(ev_state_3__N_581[3]), .SP(pll_clk_enable_724), 
            .CK(pll_clk), .Q(ev_state[3])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_state_i3.GSR = "DISABLED";
    FD1P3AX ev_state_i2 (.D(ev_state_3__N_581[2]), .SP(pll_clk_enable_724), 
            .CK(pll_clk), .Q(ev_state[2])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_state_i2.GSR = "DISABLED";
    FD1P3AX ev_state_i1 (.D(ev_state_3__N_581[1]), .SP(pll_clk_enable_724), 
            .CK(pll_clk), .Q(ev_state[1])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_state_i1.GSR = "DISABLED";
    FD1P3IX spi_channel_field_1089__i0 (.D(n15), .SP(spi1_sck_c_enable_325), 
            .CD(spi1_sck_c_enable_324), .CK(spi1_sck_c), .Q(spi_channel_field[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(316[78:102])
    defparam spi_channel_field_1089__i0.GSR = "ENABLED";
    FD1P3AX stop_toggle_spi_376 (.D(stop_toggle_spi_N_2443), .SP(spi1_sck_c_enable_326), 
            .CK(spi1_sck_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam stop_toggle_spi_376.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_319 (.A(n13915), .B(init_shadow[77]), .C(ev_bit[77]), 
         .Z(n12700)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_319.init = 16'hecec;
    FD1P3IX ev_bit_i83 (.D(ev_bit[82]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[83])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i83.GSR = "DISABLED";
    LUT4 i1271_2_lut_3_lut_4_lut (.A(ev_ch[3]), .B(n23162), .C(ev_ch[5]), 
         .D(ev_ch[4]), .Z(ev_ch_6__N_1920[5])) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(C))+!A !(C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(447[27:39])
    defparam i1271_2_lut_3_lut_4_lut.init = 16'h78f0;
    FD1P3IX ev_bit_i82 (.D(ev_bit[81]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[82])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i82.GSR = "DISABLED";
    CCU2D expected_next_15__I_0_488_9 (.A0(spi_rx_shift[0]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(spi_rx_shift[1]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n21833), .COUT(n21834), .S0(expected_next[9]), 
          .S1(expected_next[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(271[33] 273[86])
    defparam expected_next_15__I_0_488_9.INIT0 = 16'hfaaa;
    defparam expected_next_15__I_0_488_9.INIT1 = 16'hfaaa;
    defparam expected_next_15__I_0_488_9.INJECT1_0 = "NO";
    defparam expected_next_15__I_0_488_9.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i81 (.D(ev_bit[80]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[81])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i81.GSR = "DISABLED";
    FD1P3IX ev_bit_i80 (.D(ev_bit[79]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[80])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i80.GSR = "DISABLED";
    LUT4 i1_4_lut_rep_79 (.A(n23194), .B(ev_state[3]), .C(ev_state[2]), 
         .D(n23201), .Z(pll_clk_enable_666)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C)))) */ ;
    defparam i1_4_lut_rep_79.init = 16'h3032;
    FD1P3IX ev_bit_i79 (.D(ev_bit[78]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[79])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i79.GSR = "DISABLED";
    CCU2D add_491_9 (.A0(phase_frac[7]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_frac[8]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21813), .COUT(n21814), .S0(phase_frac_sum[7]), .S1(phase_frac_sum[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(155[34:66])
    defparam add_491_9.INIT0 = 16'h5555;
    defparam add_491_9.INIT1 = 16'h5aaa;
    defparam add_491_9.INJECT1_0 = "NO";
    defparam add_491_9.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i78 (.D(ev_bit[77]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[78])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i78.GSR = "DISABLED";
    FD1P3IX ev_bit_i77 (.D(ev_bit[76]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[77])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i77.GSR = "DISABLED";
    FD1P3IX ev_bit_i76 (.D(ev_bit[75]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[76])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i76.GSR = "DISABLED";
    FD1P3IX ev_bit_i75 (.D(ev_bit[74]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[75])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i75.GSR = "DISABLED";
    FD1P3IX ev_bit_i74 (.D(ev_bit[73]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[74])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i74.GSR = "DISABLED";
    FD1P3IX ev_bit_i73 (.D(ev_bit[72]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[73])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i73.GSR = "DISABLED";
    PFUMX i14372 (.BLUT(n23206), .ALUT(n23207), .C0(status_hold[74]), 
          .Z(n23208));
    FD1P3IX ev_bit_i72 (.D(ev_bit[71]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[72])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i72.GSR = "DISABLED";
    LUT4 i6456_2_lut_4_lut (.A(n23194), .B(ev_state[3]), .C(ev_state[2]), 
         .D(n23201), .Z(n15297)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i6456_2_lut_4_lut.init = 16'h0002;
    FD1P3IX ev_bit_i71 (.D(ev_bit[70]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[71])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i71.GSR = "DISABLED";
    FD1P3IX ev_bit_i70 (.D(ev_bit[69]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[70])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i70.GSR = "DISABLED";
    LUT4 m1_lut (.Z(n23428)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    FD1P3IX ev_bit_i69 (.D(ev_bit[68]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[69])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i69.GSR = "DISABLED";
    FD1P3IX ev_bit_i68 (.D(ev_bit[67]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[68])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i68.GSR = "DISABLED";
    FD1P3IX ev_bit_i67 (.D(ev_bit[66]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[67])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i67.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_320 (.A(n13915), .B(init_shadow[76]), .C(ev_bit[76]), 
         .Z(n12680)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_320.init = 16'hecec;
    FD1P3IX ev_bit_i66 (.D(ev_bit[65]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[66])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i66.GSR = "DISABLED";
    FD1P3IX ev_bit_i65 (.D(ev_bit[64]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[65])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i65.GSR = "DISABLED";
    CCU2D fpga_time_1091_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21851), .COUT(n21852), .S0(n152), .S1(n151));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_15.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i64 (.D(ev_bit[63]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[64])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i64.GSR = "DISABLED";
    LUT4 i4_4_lut_adj_321 (.A(n23174), .B(n23186), .C(spi_byte_count[5]), 
         .D(n23168), .Z(n10)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;
    defparam i4_4_lut_adj_321.init = 16'hffef;
    FD1P3IX ev_bit_i63 (.D(ev_bit[62]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[63])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i63.GSR = "DISABLED";
    FD1P3IX ev_bit_i62 (.D(ev_bit[61]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[62])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i62.GSR = "DISABLED";
    CCU2D fpga_time_1091_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21850), .COUT(n21851), .S0(n154), .S1(n153));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(366[29:46])
    defparam fpga_time_1091_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_1091_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_1091_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_1091_add_4_13.INJECT1_1 = "NO";
    FD1P3IX ev_bit_i61 (.D(ev_bit[60]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[61])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i61.GSR = "DISABLED";
    FD1P3IX ev_bit_i60 (.D(ev_bit[59]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[60])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i60.GSR = "DISABLED";
    FD1P3IX ev_bit_i59 (.D(ev_bit[58]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[59])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i59.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_322 (.A(n13915), .B(init_shadow[75]), .C(ev_bit[75]), 
         .Z(n12666)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_322.init = 16'hecec;
    FD1P3IX ev_bit_i58 (.D(ev_bit[57]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[58])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i58.GSR = "DISABLED";
    FD1P3IX ev_bit_i57 (.D(ev_bit[56]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[57])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i57.GSR = "DISABLED";
    LUT4 i1_3_lut_adj_323 (.A(n13915), .B(init_shadow[74]), .C(ev_bit[74]), 
         .Z(n12652)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_323.init = 16'hecec;
    FD1P3IX ev_bit_i56 (.D(ev_bit[55]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[56])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i56.GSR = "DISABLED";
    FD1P3IX ev_bit_i55 (.D(ev_bit[54]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[55])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i55.GSR = "DISABLED";
    FD1P3IX ev_bit_i54 (.D(ev_bit[53]), .SP(pll_clk_enable_758), .CD(n15201), 
            .CK(pll_clk), .Q(ev_bit[54])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i54.GSR = "DISABLED";
    LUT4 i13939_2_lut (.A(expected_next[5]), .B(expected_next[2]), .Z(n22582)) /* synthesis lut_function=(A (B)) */ ;
    defparam i13939_2_lut.init = 16'h8888;
    umh_channel_ram18 staging_ram (.n9803(n9803), .spi1_sck_c(spi1_sck_c), 
            .spi_channel_index({spi_channel_index}), .n9809(n9809), .staging_q({staging_q}), 
            .pll_clk(pll_clk), .rd_data_15__N_2535({rd_data_15__N_2535}), 
            .n9799(n9799), .n9797(n9797), .n9805(n9805), .n9807(n9807), 
            .spi_write(spi_write), .VCC_net(VCC_net), .GND_net(GND_net), 
            .staging_rd_addr_6__N_785({staging_rd_addr_6__N_785}), .spi1_mosi_c_0(spi1_mosi_c_0), 
            .\spi_rx_shift[0] (spi_rx_shift[0]), .\spi_rx_shift[1] (spi_rx_shift[1]), 
            .\spi_rx_shift[2] (spi_rx_shift[2]), .\spi_rx_shift[3] (spi_rx_shift[3]), 
            .\spi_rx_shift[4] (spi_rx_shift[4]), .\spi_rx_shift[5] (spi_rx_shift[5]), 
            .\spi_rx_shift[6] (spi_rx_shift[6]), .spi_phase_pending({spi_phase_pending}), 
            .n9814(n9814), .n9816(n9816), .n9818(n9818), .n9820(n9820), 
            .n9822(n9822), .n9824(n9824), .n9826(n9826), .n9828(n9828), 
            .n9830(n9830), .n9832(n9832), .n9834(n9834), .n9836(n9836), 
            .n9838(n9838), .n9840(n9840), .n9842(n9842), .n9844(n9844), 
            .n9801(n9801)) /* synthesis syn_module_defined=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(193[23] 196[6])
    LUT4 i1_3_lut_adj_324 (.A(n13915), .B(init_shadow[73]), .C(ev_bit[73]), 
         .Z(n12636)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_324.init = 16'hecec;
    LUT4 i1_3_lut_adj_325 (.A(n13915), .B(init_shadow[72]), .C(ev_bit[72]), 
         .Z(n12620)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_325.init = 16'hecec;
    LUT4 i1_3_lut_adj_326 (.A(n13915), .B(init_shadow[71]), .C(ev_bit[71]), 
         .Z(n12599)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_326.init = 16'hecec;
    GSR GSR_INST (.GSR(fpga_cs_n_N_2458));
    FD1P3IX ev_bit_i13 (.D(ev_bit[12]), .SP(pll_clk_enable_759), .CD(n23447), 
            .CK(pll_clk), .Q(ev_bit[13])) /* synthesis lse_init_val=0 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(348[12] 510[8])
    defparam ev_bit_i13.GSR = "DISABLED";
    PFUMX i14355 (.BLUT(n23095), .ALUT(n23094), .C0(ev_state[3]), .Z(ev_state_3__N_581[3]));
    LUT4 i12_4_lut (.A(expected_next[3]), .B(n24), .C(n18_adj_3027), .D(expected_next[6]), 
         .Z(n26_adj_3026)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i12_4_lut.init = 16'hfffe;
    CCU2D equal_1582_0 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(frame_end_N_2500[16]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21701));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(276[48:95])
    defparam equal_1582_0.INIT0 = 16'hF000;
    defparam equal_1582_0.INIT1 = 16'h5555;
    defparam equal_1582_0.INJECT1_0 = "NO";
    defparam equal_1582_0.INJECT1_1 = "YES";
    LUT4 i1_3_lut_adj_327 (.A(n13915), .B(init_shadow[70]), .C(ev_bit[70]), 
         .Z(n12590)) /* synthesis lut_function=(A (B+(C))+!A (B)) */ ;
    defparam i1_3_lut_adj_327.init = 16'hecec;
    LUT4 i8_4_lut (.A(expected_next[4]), .B(expected_next[10]), .C(expected_next[9]), 
         .D(expected_next[13]), .Z(n22)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(287[18] 342[12])
    defparam i8_4_lut.init = 16'hfffe;
    
endmodule
//
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (mic_latest, sck_N_2937, spi_mic_cs_n_c, spi_mic_miso_c) /* synthesis syn_module_defined=1 */ ;
    input [31:0]mic_latest;
    input sck_N_2937;
    input spi_mic_cs_n_c;
    output spi_mic_miso_c;
    
    wire sck_N_2937 /* synthesis is_inv_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    wire [31:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(11[12:26])
    
    wire n14065, n14124, n14126;
    wire [5:0]bit_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(12[11:20])
    wire [5:0]bit_count_5__N_2938;
    
    wire sck_N_2937_enable_35, n15419, n15417, n15415, n15412, n23154, 
        n23183, n23164, n14066, n14068, n14070, n14072, n14074, 
        n14076, n14078, n14080, n6, n14082, n14084, n14086, n14088, 
        n14090, n14092, n14094, n14096, n14098, n14100, n14102, 
        n14104, n14106, n14108, n14110, n14112, n14114, n14116, 
        n14118, n14120, n14122;
    
    LUT4 i5388_3_lut (.A(mic_latest[29]), .B(shift_register[29]), .C(n14065), 
         .Z(n14124)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5388_3_lut.init = 16'hcaca;
    LUT4 i5390_3_lut (.A(mic_latest[30]), .B(shift_register[30]), .C(n14065), 
         .Z(n14126)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5390_3_lut.init = 16'hcaca;
    FD1S3DX bit_count_i0 (.D(bit_count_5__N_2938[0]), .CK(sck_N_2937), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i0.GSR = "DISABLED";
    FD1P3DX bit_count_i4 (.D(n15419), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i4.GSR = "DISABLED";
    FD1P3DX bit_count_i3 (.D(n15417), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i3.GSR = "DISABLED";
    FD1P3DX bit_count_i2 (.D(n15415), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i2.GSR = "DISABLED";
    FD1P3DX bit_count_i1 (.D(n15412), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i1.GSR = "DISABLED";
    LUT4 i9987_4_lut (.A(mic_latest[31]), .B(spi_mic_cs_n_c), .C(shift_register[31]), 
         .D(n23154), .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C (D))))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(18[15] 19[73])
    defparam i9987_4_lut.init = 16'h3022;
    LUT4 i1327_2_lut_rep_101 (.A(bit_count[1]), .B(bit_count[0]), .Z(n23183)) /* synthesis lut_function=(A (B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[22:38])
    defparam i1327_2_lut_rep_101.init = 16'h8888;
    LUT4 i1334_2_lut_rep_82_3_lut (.A(bit_count[1]), .B(bit_count[0]), .C(bit_count[2]), 
         .Z(n23164)) /* synthesis lut_function=(A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[22:38])
    defparam i1334_2_lut_rep_82_3_lut.init = 16'h8080;
    LUT4 i10073_2_lut (.A(mic_latest[0]), .B(n14065), .Z(n14066)) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i10073_2_lut.init = 16'h2222;
    LUT4 i5332_3_lut (.A(mic_latest[1]), .B(shift_register[1]), .C(n14065), 
         .Z(n14068)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5332_3_lut.init = 16'hcaca;
    LUT4 i5334_3_lut (.A(mic_latest[2]), .B(shift_register[2]), .C(n14065), 
         .Z(n14070)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5334_3_lut.init = 16'hcaca;
    LUT4 i5336_3_lut (.A(mic_latest[3]), .B(shift_register[3]), .C(n14065), 
         .Z(n14072)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5336_3_lut.init = 16'hcaca;
    LUT4 i5338_3_lut (.A(mic_latest[4]), .B(shift_register[4]), .C(n14065), 
         .Z(n14074)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5338_3_lut.init = 16'hcaca;
    LUT4 i5340_3_lut (.A(mic_latest[5]), .B(shift_register[5]), .C(n14065), 
         .Z(n14076)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5340_3_lut.init = 16'hcaca;
    LUT4 i5342_3_lut (.A(mic_latest[6]), .B(shift_register[6]), .C(n14065), 
         .Z(n14078)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5342_3_lut.init = 16'hcaca;
    LUT4 i5344_3_lut (.A(mic_latest[7]), .B(shift_register[7]), .C(n14065), 
         .Z(n14080)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5344_3_lut.init = 16'hcaca;
    LUT4 i4_4_lut (.A(bit_count[1]), .B(bit_count[4]), .C(bit_count[3]), 
         .D(n6), .Z(n14065)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(19[16:33])
    defparam i4_4_lut.init = 16'hfffe;
    LUT4 i1_2_lut (.A(bit_count[2]), .B(bit_count[0]), .Z(n6)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(19[16:33])
    defparam i1_2_lut.init = 16'heeee;
    LUT4 i5346_3_lut (.A(mic_latest[8]), .B(shift_register[8]), .C(n14065), 
         .Z(n14082)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5346_3_lut.init = 16'hcaca;
    LUT4 i5348_3_lut (.A(mic_latest[9]), .B(shift_register[9]), .C(n14065), 
         .Z(n14084)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5348_3_lut.init = 16'hcaca;
    LUT4 i5350_3_lut (.A(mic_latest[10]), .B(shift_register[10]), .C(n14065), 
         .Z(n14086)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5350_3_lut.init = 16'hcaca;
    LUT4 i5352_3_lut (.A(mic_latest[11]), .B(shift_register[11]), .C(n14065), 
         .Z(n14088)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5352_3_lut.init = 16'hcaca;
    LUT4 i5354_3_lut (.A(mic_latest[12]), .B(shift_register[12]), .C(n14065), 
         .Z(n14090)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5354_3_lut.init = 16'hcaca;
    LUT4 i5356_3_lut (.A(mic_latest[13]), .B(shift_register[13]), .C(n14065), 
         .Z(n14092)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5356_3_lut.init = 16'hcaca;
    LUT4 i5358_3_lut (.A(mic_latest[14]), .B(shift_register[14]), .C(n14065), 
         .Z(n14094)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5358_3_lut.init = 16'hcaca;
    LUT4 i5360_3_lut (.A(mic_latest[15]), .B(shift_register[15]), .C(n14065), 
         .Z(n14096)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5360_3_lut.init = 16'hcaca;
    LUT4 i5362_3_lut (.A(mic_latest[16]), .B(shift_register[16]), .C(n14065), 
         .Z(n14098)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5362_3_lut.init = 16'hcaca;
    FD1S3DX bit_count_i5 (.D(bit_count_5__N_2938[5]), .CK(sck_N_2937), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i5.GSR = "DISABLED";
    FD1P3DX shift_register_i1 (.D(n14066), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3DX shift_register_i2 (.D(n14068), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3DX shift_register_i3 (.D(n14070), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3DX shift_register_i4 (.D(n14072), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3DX shift_register_i5 (.D(n14074), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3DX shift_register_i6 (.D(n14076), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3DX shift_register_i7 (.D(n14078), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3DX shift_register_i8 (.D(n14080), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3DX shift_register_i9 (.D(n14082), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3DX shift_register_i10 (.D(n14084), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3DX shift_register_i11 (.D(n14086), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3DX shift_register_i12 (.D(n14088), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3DX shift_register_i13 (.D(n14090), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3DX shift_register_i14 (.D(n14092), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3DX shift_register_i15 (.D(n14094), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3DX shift_register_i16 (.D(n14096), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[16])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3DX shift_register_i17 (.D(n14098), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[17])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3DX shift_register_i18 (.D(n14100), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[18])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3DX shift_register_i19 (.D(n14102), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[19])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i19.GSR = "DISABLED";
    FD1P3DX shift_register_i20 (.D(n14104), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[20])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3DX shift_register_i21 (.D(n14106), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[21])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3DX shift_register_i22 (.D(n14108), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[22])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3DX shift_register_i23 (.D(n14110), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[23])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i23.GSR = "DISABLED";
    FD1P3DX shift_register_i24 (.D(n14112), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[24])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i24.GSR = "DISABLED";
    FD1P3DX shift_register_i25 (.D(n14114), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[25])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i25.GSR = "DISABLED";
    FD1P3DX shift_register_i26 (.D(n14116), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[26])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i26.GSR = "DISABLED";
    FD1P3DX shift_register_i27 (.D(n14118), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[27])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i27.GSR = "DISABLED";
    FD1P3DX shift_register_i28 (.D(n14120), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[28])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i28.GSR = "DISABLED";
    FD1P3DX shift_register_i29 (.D(n14122), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[29])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i29.GSR = "DISABLED";
    FD1P3DX shift_register_i30 (.D(n14124), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[30])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i30.GSR = "DISABLED";
    FD1P3DX shift_register_i31 (.D(n14126), .SP(sck_N_2937_enable_35), .CK(sck_N_2937), 
            .CD(spi_mic_cs_n_c), .Q(shift_register[31])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=20, LSE_RCOL=6, LSE_LLINE=522, LSE_RLINE=525 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i31.GSR = "DISABLED";
    LUT4 i5364_3_lut (.A(mic_latest[17]), .B(shift_register[17]), .C(n14065), 
         .Z(n14100)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5364_3_lut.init = 16'hcaca;
    LUT4 i5366_3_lut (.A(mic_latest[18]), .B(shift_register[18]), .C(n14065), 
         .Z(n14102)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5366_3_lut.init = 16'hcaca;
    LUT4 i5368_3_lut (.A(mic_latest[19]), .B(shift_register[19]), .C(n14065), 
         .Z(n14104)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5368_3_lut.init = 16'hcaca;
    LUT4 i5370_3_lut (.A(mic_latest[20]), .B(shift_register[20]), .C(n14065), 
         .Z(n14106)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5370_3_lut.init = 16'hcaca;
    LUT4 i5372_3_lut (.A(mic_latest[21]), .B(shift_register[21]), .C(n14065), 
         .Z(n14108)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5372_3_lut.init = 16'hcaca;
    LUT4 i5374_3_lut (.A(mic_latest[22]), .B(shift_register[22]), .C(n14065), 
         .Z(n14110)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5374_3_lut.init = 16'hcaca;
    LUT4 i5376_3_lut (.A(mic_latest[23]), .B(shift_register[23]), .C(n14065), 
         .Z(n14112)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5376_3_lut.init = 16'hcaca;
    LUT4 i5378_3_lut (.A(mic_latest[24]), .B(shift_register[24]), .C(n14065), 
         .Z(n14114)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5378_3_lut.init = 16'hcaca;
    LUT4 i5380_3_lut (.A(mic_latest[25]), .B(shift_register[25]), .C(n14065), 
         .Z(n14116)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5380_3_lut.init = 16'hcaca;
    LUT4 i5382_3_lut (.A(mic_latest[26]), .B(shift_register[26]), .C(n14065), 
         .Z(n14118)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5382_3_lut.init = 16'hcaca;
    LUT4 i5384_3_lut (.A(mic_latest[27]), .B(shift_register[27]), .C(n14065), 
         .Z(n14120)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5384_3_lut.init = 16'hcaca;
    LUT4 i5386_3_lut (.A(mic_latest[28]), .B(shift_register[28]), .C(n14065), 
         .Z(n14122)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(28[14] 31[8])
    defparam i5386_3_lut.init = 16'hcaca;
    LUT4 i6303_1_lut (.A(bit_count[5]), .Z(sck_N_2937_enable_35)) /* synthesis lut_function=(!(A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(25[14] 31[8])
    defparam i6303_1_lut.init = 16'h5555;
    LUT4 i10057_3_lut_4_lut (.A(bit_count[3]), .B(n23164), .C(n23154), 
         .D(bit_count[4]), .Z(n15419)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C (D)))+!A !(C (D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[22:38])
    defparam i10057_3_lut_4_lut.init = 16'h7080;
    LUT4 i10276_3_lut_4_lut (.A(bit_count[3]), .B(n23164), .C(bit_count[4]), 
         .D(bit_count[5]), .Z(bit_count_5__N_2938[5])) /* synthesis lut_function=(A (B (C+(D))+!B (D))+!A (D)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(30[22:38])
    defparam i10276_3_lut_4_lut.init = 16'hff80;
    LUT4 i3_2_lut_rep_72 (.A(n14065), .B(bit_count[5]), .Z(n23154)) /* synthesis lut_function=(A+(B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(19[16:33])
    defparam i3_2_lut_rep_72.init = 16'heeee;
    LUT4 i9988_3_lut_3_lut (.A(n14065), .B(bit_count[5]), .C(bit_count[0]), 
         .Z(bit_count_5__N_2938[0])) /* synthesis lut_function=(A (B (C)+!B !(C))+!A ((C)+!B)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(19[16:33])
    defparam i9988_3_lut_3_lut.init = 16'hd3d3;
    LUT4 i10053_3_lut_4_lut (.A(n14065), .B(bit_count[5]), .C(bit_count[0]), 
         .D(bit_count[1]), .Z(n15412)) /* synthesis lut_function=(!(A (C (D)+!C !(D))+!A ((C (D)+!C !(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(19[16:33])
    defparam i10053_3_lut_4_lut.init = 16'h0ee0;
    LUT4 i10055_3_lut_4_lut (.A(n14065), .B(bit_count[5]), .C(n23183), 
         .D(bit_count[2]), .Z(n15415)) /* synthesis lut_function=(!(A (C (D)+!C !(D))+!A ((C (D)+!C !(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(19[16:33])
    defparam i10055_3_lut_4_lut.init = 16'h0ee0;
    LUT4 i10056_3_lut_4_lut (.A(n14065), .B(bit_count[5]), .C(n23164), 
         .D(bit_count[3]), .Z(n15417)) /* synthesis lut_function=(!(A (C (D)+!C !(D))+!A ((C (D)+!C !(D))+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/spi_mic_stream.v(19[16:33])
    defparam i10056_3_lut_4_lut.init = 16'h0ee0;
    
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

module ws2812_stream (GND_net, rgb_hold, pll_clk, rgb_data_c) /* synthesis syn_module_defined=1 */ ;
    input GND_net;
    input [95:0]rgb_hold;
    input pll_clk;
    output rgb_data_c;
    
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[10:17])
    
    wire n21794;
    wire [12:0]reset_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(17[12:23])
    wire [12:0]reset_count_12__N_2882;
    
    wire n21795, n21793;
    wire [95:0]shift_register;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(20[12:26])
    wire [1:0]state;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(16[11:16])
    wire [95:0]shift_register_95__N_2665;
    
    wire n18987, n137;
    wire [6:0]bit_number;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(19[11:21])
    
    wire n23150, n21933, n23179, n16155;
    wire [7:0]bit_cell_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(18[11:25])
    
    wire n4, n119, n22432, n132, n21792, pll_clk_enable_728;
    wire [1:0]state_1__N_2635;
    wire [6:0]bit_number_6__N_2895;
    
    wire pll_clk_enable_726, n15394;
    wire [7:0]bit_cell_count_7__N_2650;
    
    wire n23178, n75, n23158, n7, pll_clk_enable_597, n23191, n19059, 
        n17, n16, pll_clk_enable_727;
    wire [7:0]high_count;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(21[12:22])
    
    wire n21828, n21827, n21826, n21825;
    wire [7:0]n84;
    
    wire n21824, n21791, n21823, n21822, n10, n10_adj_2984, n21964, 
        n21796;
    
    CCU2D add_909_9 (.A0(reset_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21794), .COUT(n21795), .S0(reset_count_12__N_2882[7]), 
          .S1(reset_count_12__N_2882[8]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[22] 31[16])
    defparam add_909_9.INIT0 = 16'h5aaa;
    defparam add_909_9.INIT1 = 16'h5aaa;
    defparam add_909_9.INJECT1_0 = "NO";
    defparam add_909_9.INJECT1_1 = "NO";
    CCU2D add_909_7 (.A0(reset_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21793), .COUT(n21794), .S0(reset_count_12__N_2882[5]), 
          .S1(reset_count_12__N_2882[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[22] 31[16])
    defparam add_909_7.INIT0 = 16'h5aaa;
    defparam add_909_7.INIT1 = 16'h5aaa;
    defparam add_909_7.INJECT1_0 = "NO";
    defparam add_909_7.INJECT1_1 = "NO";
    LUT4 state_1__I_0_41_Mux_1_i3_3_lut (.A(rgb_hold[1]), .B(shift_register[0]), 
         .C(state[1]), .Z(shift_register_95__N_2665[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_1_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_2_i3_3_lut (.A(rgb_hold[2]), .B(shift_register[1]), 
         .C(state[1]), .Z(shift_register_95__N_2665[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_2_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_3_i3_3_lut (.A(rgb_hold[3]), .B(shift_register[2]), 
         .C(state[1]), .Z(shift_register_95__N_2665[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_3_i3_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_rep_68 (.A(n18987), .B(n137), .C(bit_number[5]), .Z(n23150)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i2_3_lut_rep_68.init = 16'h0808;
    LUT4 state_1__I_0_41_Mux_4_i3_3_lut (.A(rgb_hold[4]), .B(shift_register[3]), 
         .C(state[1]), .Z(shift_register_95__N_2665[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_4_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_5_i3_3_lut (.A(rgb_hold[5]), .B(shift_register[4]), 
         .C(state[1]), .Z(shift_register_95__N_2665[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_5_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_6_i3_3_lut (.A(rgb_hold[6]), .B(shift_register[5]), 
         .C(state[1]), .Z(shift_register_95__N_2665[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_6_i3_3_lut.init = 16'hcaca;
    FD1S3AX state_i0 (.D(n21933), .CK(pll_clk), .Q(state[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam state_i0.GSR = "DISABLED";
    LUT4 state_1__I_0_41_Mux_7_i3_3_lut (.A(rgb_hold[7]), .B(shift_register[6]), 
         .C(state[1]), .Z(shift_register_95__N_2665[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_7_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_8_i3_3_lut (.A(rgb_hold[16]), .B(shift_register[7]), 
         .C(state[1]), .Z(shift_register_95__N_2665[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_8_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_9_i3_3_lut (.A(rgb_hold[17]), .B(shift_register[8]), 
         .C(state[1]), .Z(shift_register_95__N_2665[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_9_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_10_i3_3_lut (.A(rgb_hold[18]), .B(shift_register[9]), 
         .C(state[1]), .Z(shift_register_95__N_2665[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_10_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_4_lut (.A(n18987), .B(n137), .C(bit_number[5]), .D(n23179), 
         .Z(n16155)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i1_2_lut_4_lut.init = 16'h0800;
    LUT4 state_1__I_0_41_Mux_11_i3_3_lut (.A(rgb_hold[19]), .B(shift_register[10]), 
         .C(state[1]), .Z(shift_register_95__N_2665[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_11_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_12_i3_3_lut (.A(rgb_hold[20]), .B(shift_register[11]), 
         .C(state[1]), .Z(shift_register_95__N_2665[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_12_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_13_i3_3_lut (.A(rgb_hold[21]), .B(shift_register[12]), 
         .C(state[1]), .Z(shift_register_95__N_2665[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_13_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_14_i3_3_lut (.A(rgb_hold[22]), .B(shift_register[13]), 
         .C(state[1]), .Z(shift_register_95__N_2665[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_14_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_15_i3_3_lut (.A(rgb_hold[23]), .B(shift_register[14]), 
         .C(state[1]), .Z(shift_register_95__N_2665[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_15_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_16_i3_3_lut (.A(rgb_hold[8]), .B(shift_register[15]), 
         .C(state[1]), .Z(shift_register_95__N_2665[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_16_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_17_i3_3_lut (.A(rgb_hold[9]), .B(shift_register[16]), 
         .C(state[1]), .Z(shift_register_95__N_2665[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_17_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_18_i3_3_lut (.A(rgb_hold[10]), .B(shift_register[17]), 
         .C(state[1]), .Z(shift_register_95__N_2665[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_18_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_19_i3_3_lut (.A(rgb_hold[11]), .B(shift_register[18]), 
         .C(state[1]), .Z(shift_register_95__N_2665[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_19_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_20_i3_3_lut (.A(rgb_hold[12]), .B(shift_register[19]), 
         .C(state[1]), .Z(shift_register_95__N_2665[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_20_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_21_i3_3_lut (.A(rgb_hold[13]), .B(shift_register[20]), 
         .C(state[1]), .Z(shift_register_95__N_2665[21])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_21_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_22_i3_3_lut (.A(rgb_hold[14]), .B(shift_register[21]), 
         .C(state[1]), .Z(shift_register_95__N_2665[22])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_22_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_23_i3_3_lut (.A(rgb_hold[15]), .B(shift_register[22]), 
         .C(state[1]), .Z(shift_register_95__N_2665[23])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_23_i3_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut (.A(bit_cell_count[6]), .B(bit_cell_count[4]), .C(bit_cell_count[5]), 
         .D(n4), .Z(n119)) /* synthesis lut_function=(A (B (C+(D))+!B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(18[11:25])
    defparam i1_4_lut.init = 16'ha8a0;
    LUT4 i1_4_lut_adj_20 (.A(bit_cell_count[6]), .B(bit_cell_count[5]), 
         .C(n22432), .D(bit_cell_count[4]), .Z(n132)) /* synthesis lut_function=(A+(B (C+(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(18[11:25])
    defparam i1_4_lut_adj_20.init = 16'heeea;
    LUT4 state_1__I_0_41_Mux_24_i3_3_lut (.A(rgb_hold[24]), .B(shift_register[23]), 
         .C(state[1]), .Z(shift_register_95__N_2665[24])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_24_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_25_i3_3_lut (.A(rgb_hold[25]), .B(shift_register[24]), 
         .C(state[1]), .Z(shift_register_95__N_2665[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_25_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_26_i3_3_lut (.A(rgb_hold[26]), .B(shift_register[25]), 
         .C(state[1]), .Z(shift_register_95__N_2665[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_26_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_27_i3_3_lut (.A(rgb_hold[27]), .B(shift_register[26]), 
         .C(state[1]), .Z(shift_register_95__N_2665[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_27_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_28_i3_3_lut (.A(rgb_hold[28]), .B(shift_register[27]), 
         .C(state[1]), .Z(shift_register_95__N_2665[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_28_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_29_i3_3_lut (.A(rgb_hold[29]), .B(shift_register[28]), 
         .C(state[1]), .Z(shift_register_95__N_2665[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_29_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_30_i3_3_lut (.A(rgb_hold[30]), .B(shift_register[29]), 
         .C(state[1]), .Z(shift_register_95__N_2665[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_30_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_31_i3_3_lut (.A(rgb_hold[31]), .B(shift_register[30]), 
         .C(state[1]), .Z(shift_register_95__N_2665[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_31_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_32_i3_3_lut (.A(rgb_hold[40]), .B(shift_register[31]), 
         .C(state[1]), .Z(shift_register_95__N_2665[32])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_32_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_33_i3_3_lut (.A(rgb_hold[41]), .B(shift_register[32]), 
         .C(state[1]), .Z(shift_register_95__N_2665[33])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_33_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_34_i3_3_lut (.A(rgb_hold[42]), .B(shift_register[33]), 
         .C(state[1]), .Z(shift_register_95__N_2665[34])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_34_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_35_i3_3_lut (.A(rgb_hold[43]), .B(shift_register[34]), 
         .C(state[1]), .Z(shift_register_95__N_2665[35])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_35_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_36_i3_3_lut (.A(rgb_hold[44]), .B(shift_register[35]), 
         .C(state[1]), .Z(shift_register_95__N_2665[36])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_36_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_37_i3_3_lut (.A(rgb_hold[45]), .B(shift_register[36]), 
         .C(state[1]), .Z(shift_register_95__N_2665[37])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_37_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_38_i3_3_lut (.A(rgb_hold[46]), .B(shift_register[37]), 
         .C(state[1]), .Z(shift_register_95__N_2665[38])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_38_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_39_i3_3_lut (.A(rgb_hold[47]), .B(shift_register[38]), 
         .C(state[1]), .Z(shift_register_95__N_2665[39])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_39_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut (.A(bit_cell_count[1]), .B(bit_cell_count[3]), .C(bit_cell_count[2]), 
         .Z(n4)) /* synthesis lut_function=(A (B)+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i1_3_lut.init = 16'hc8c8;
    LUT4 state_1__I_0_41_Mux_40_i3_3_lut (.A(rgb_hold[32]), .B(shift_register[39]), 
         .C(state[1]), .Z(shift_register_95__N_2665[40])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_40_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_41_i3_3_lut (.A(rgb_hold[33]), .B(shift_register[40]), 
         .C(state[1]), .Z(shift_register_95__N_2665[41])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_41_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_42_i3_3_lut (.A(rgb_hold[34]), .B(shift_register[41]), 
         .C(state[1]), .Z(shift_register_95__N_2665[42])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_42_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_43_i3_3_lut (.A(rgb_hold[35]), .B(shift_register[42]), 
         .C(state[1]), .Z(shift_register_95__N_2665[43])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_43_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_44_i3_3_lut (.A(rgb_hold[36]), .B(shift_register[43]), 
         .C(state[1]), .Z(shift_register_95__N_2665[44])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_44_i3_3_lut.init = 16'hcaca;
    CCU2D add_909_5 (.A0(reset_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21792), .COUT(n21793), .S0(reset_count_12__N_2882[3]), 
          .S1(reset_count_12__N_2882[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[22] 31[16])
    defparam add_909_5.INIT0 = 16'h5aaa;
    defparam add_909_5.INIT1 = 16'h5aaa;
    defparam add_909_5.INJECT1_0 = "NO";
    defparam add_909_5.INJECT1_1 = "NO";
    LUT4 state_1__I_0_41_Mux_45_i3_3_lut (.A(rgb_hold[37]), .B(shift_register[44]), 
         .C(state[1]), .Z(shift_register_95__N_2665[45])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_45_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_46_i3_3_lut (.A(rgb_hold[38]), .B(shift_register[45]), 
         .C(state[1]), .Z(shift_register_95__N_2665[46])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_46_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_47_i3_3_lut (.A(rgb_hold[39]), .B(shift_register[46]), 
         .C(state[1]), .Z(shift_register_95__N_2665[47])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_47_i3_3_lut.init = 16'hcaca;
    FD1P3IX bit_number_i6 (.D(bit_number_6__N_2895[6]), .SP(pll_clk_enable_728), 
            .CD(state_1__N_2635[1]), .CK(pll_clk), .Q(bit_number[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_number_i6.GSR = "DISABLED";
    FD1P3IX bit_number_i5 (.D(bit_number_6__N_2895[5]), .SP(pll_clk_enable_728), 
            .CD(state_1__N_2635[1]), .CK(pll_clk), .Q(bit_number[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_number_i5.GSR = "DISABLED";
    FD1P3IX bit_number_i4 (.D(bit_number_6__N_2895[4]), .SP(pll_clk_enable_728), 
            .CD(state_1__N_2635[1]), .CK(pll_clk), .Q(bit_number[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_number_i4.GSR = "DISABLED";
    FD1P3IX bit_number_i3 (.D(bit_number_6__N_2895[3]), .SP(pll_clk_enable_728), 
            .CD(state_1__N_2635[1]), .CK(pll_clk), .Q(bit_number[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_number_i3.GSR = "DISABLED";
    FD1P3IX bit_number_i2 (.D(bit_number_6__N_2895[2]), .SP(pll_clk_enable_728), 
            .CD(state_1__N_2635[1]), .CK(pll_clk), .Q(bit_number[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_number_i2.GSR = "DISABLED";
    FD1P3IX bit_number_i1 (.D(bit_number_6__N_2895[1]), .SP(pll_clk_enable_728), 
            .CD(state_1__N_2635[1]), .CK(pll_clk), .Q(bit_number[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_number_i1.GSR = "DISABLED";
    FD1P3IX reset_count_i12 (.D(reset_count_12__N_2882[12]), .SP(pll_clk_enable_726), 
            .CD(n16155), .CK(pll_clk), .Q(reset_count[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i12.GSR = "DISABLED";
    FD1P3IX reset_count_i11 (.D(reset_count_12__N_2882[11]), .SP(pll_clk_enable_726), 
            .CD(n16155), .CK(pll_clk), .Q(reset_count[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i11.GSR = "DISABLED";
    FD1P3IX reset_count_i10 (.D(reset_count_12__N_2882[10]), .SP(pll_clk_enable_726), 
            .CD(n16155), .CK(pll_clk), .Q(reset_count[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i10.GSR = "DISABLED";
    FD1P3IX reset_count_i9 (.D(reset_count_12__N_2882[9]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i9.GSR = "DISABLED";
    FD1P3IX reset_count_i8 (.D(reset_count_12__N_2882[8]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i8.GSR = "DISABLED";
    FD1P3IX reset_count_i7 (.D(reset_count_12__N_2882[7]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i7.GSR = "DISABLED";
    FD1P3IX reset_count_i6 (.D(reset_count_12__N_2882[6]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i6.GSR = "DISABLED";
    FD1P3IX reset_count_i5 (.D(reset_count_12__N_2882[5]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i5.GSR = "DISABLED";
    FD1P3IX reset_count_i4 (.D(reset_count_12__N_2882[4]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i4.GSR = "DISABLED";
    FD1P3IX reset_count_i3 (.D(reset_count_12__N_2882[3]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i3.GSR = "DISABLED";
    FD1P3IX reset_count_i2 (.D(reset_count_12__N_2882[2]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i2.GSR = "DISABLED";
    FD1P3IX reset_count_i1 (.D(reset_count_12__N_2882[1]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i1.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i7 (.D(bit_cell_count_7__N_2650[7]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i7.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i6 (.D(bit_cell_count_7__N_2650[6]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i6.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i5 (.D(bit_cell_count_7__N_2650[5]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i5.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i4 (.D(bit_cell_count_7__N_2650[4]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i4.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i3 (.D(bit_cell_count_7__N_2650[3]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i3.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i2 (.D(bit_cell_count_7__N_2650[2]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i2.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i1 (.D(bit_cell_count_7__N_2650[1]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i1.GSR = "DISABLED";
    LUT4 state_1__I_0_41_Mux_48_i3_3_lut (.A(rgb_hold[48]), .B(shift_register[47]), 
         .C(state[1]), .Z(shift_register_95__N_2665[48])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_48_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_49_i3_3_lut (.A(rgb_hold[49]), .B(shift_register[48]), 
         .C(state[1]), .Z(shift_register_95__N_2665[49])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_49_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_50_i3_3_lut (.A(rgb_hold[50]), .B(shift_register[49]), 
         .C(state[1]), .Z(shift_register_95__N_2665[50])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_50_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_51_i3_3_lut (.A(rgb_hold[51]), .B(shift_register[50]), 
         .C(state[1]), .Z(shift_register_95__N_2665[51])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_51_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_52_i3_3_lut (.A(rgb_hold[52]), .B(shift_register[51]), 
         .C(state[1]), .Z(shift_register_95__N_2665[52])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_52_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_53_i3_3_lut (.A(rgb_hold[53]), .B(shift_register[52]), 
         .C(state[1]), .Z(shift_register_95__N_2665[53])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_53_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_54_i3_3_lut (.A(rgb_hold[54]), .B(shift_register[53]), 
         .C(state[1]), .Z(shift_register_95__N_2665[54])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_54_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_55_i3_3_lut (.A(rgb_hold[55]), .B(shift_register[54]), 
         .C(state[1]), .Z(shift_register_95__N_2665[55])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_55_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_96 (.A(bit_cell_count[3]), .B(bit_cell_count[2]), 
         .Z(n23178)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_rep_96.init = 16'h8888;
    LUT4 i1_3_lut_4_lut (.A(bit_cell_count[3]), .B(bit_cell_count[2]), .C(bit_cell_count[0]), 
         .D(bit_cell_count[1]), .Z(n22432)) /* synthesis lut_function=(A (B (C+(D)))) */ ;
    defparam i1_3_lut_4_lut.init = 16'h8880;
    LUT4 state_1__I_0_41_Mux_56_i3_3_lut (.A(rgb_hold[64]), .B(shift_register[55]), 
         .C(state[1]), .Z(shift_register_95__N_2665[56])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_56_i3_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_rep_97 (.A(state[1]), .B(state[0]), .Z(n23179)) /* synthesis lut_function=(!((B)+!A)) */ ;
    defparam i1_2_lut_rep_97.init = 16'h2222;
    LUT4 state_1__I_0_41_Mux_57_i3_3_lut (.A(rgb_hold[65]), .B(shift_register[56]), 
         .C(state[1]), .Z(shift_register_95__N_2665[57])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_57_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_58_i3_3_lut (.A(rgb_hold[66]), .B(shift_register[57]), 
         .C(state[1]), .Z(shift_register_95__N_2665[58])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_58_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_59_i3_3_lut (.A(rgb_hold[67]), .B(shift_register[58]), 
         .C(state[1]), .Z(shift_register_95__N_2665[59])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_59_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_60_i3_3_lut (.A(rgb_hold[68]), .B(shift_register[59]), 
         .C(state[1]), .Z(shift_register_95__N_2665[60])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_60_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_61_i3_3_lut (.A(rgb_hold[69]), .B(shift_register[60]), 
         .C(state[1]), .Z(shift_register_95__N_2665[61])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_61_i3_3_lut.init = 16'hcaca;
    LUT4 i1_3_lut_4_lut_adj_21 (.A(state[1]), .B(state[0]), .C(bit_cell_count[7]), 
         .D(n75), .Z(rgb_data_c)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;
    defparam i1_3_lut_4_lut_adj_21.init = 16'h0002;
    LUT4 state_1__I_0_41_Mux_62_i3_3_lut (.A(rgb_hold[70]), .B(shift_register[61]), 
         .C(state[1]), .Z(shift_register_95__N_2665[62])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_62_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_63_i3_3_lut (.A(rgb_hold[71]), .B(shift_register[62]), 
         .C(state[1]), .Z(shift_register_95__N_2665[63])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_63_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_64_i3_3_lut (.A(rgb_hold[56]), .B(shift_register[63]), 
         .C(state[1]), .Z(shift_register_95__N_2665[64])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_64_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_65_i3_3_lut (.A(rgb_hold[57]), .B(shift_register[64]), 
         .C(state[1]), .Z(shift_register_95__N_2665[65])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_65_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_66_i3_3_lut (.A(rgb_hold[58]), .B(shift_register[65]), 
         .C(state[1]), .Z(shift_register_95__N_2665[66])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_66_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_67_i3_3_lut (.A(rgb_hold[59]), .B(shift_register[66]), 
         .C(state[1]), .Z(shift_register_95__N_2665[67])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_67_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_68_i3_3_lut (.A(rgb_hold[60]), .B(shift_register[67]), 
         .C(state[1]), .Z(shift_register_95__N_2665[68])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_68_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_69_i3_3_lut (.A(rgb_hold[61]), .B(shift_register[68]), 
         .C(state[1]), .Z(shift_register_95__N_2665[69])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_69_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_70_i3_3_lut (.A(rgb_hold[62]), .B(shift_register[69]), 
         .C(state[1]), .Z(shift_register_95__N_2665[70])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_70_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_71_i3_3_lut (.A(rgb_hold[63]), .B(shift_register[70]), 
         .C(state[1]), .Z(shift_register_95__N_2665[71])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_71_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_72_i3_3_lut (.A(rgb_hold[72]), .B(shift_register[71]), 
         .C(state[1]), .Z(shift_register_95__N_2665[72])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_72_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_73_i3_3_lut (.A(rgb_hold[73]), .B(shift_register[72]), 
         .C(state[1]), .Z(shift_register_95__N_2665[73])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_73_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_74_i3_3_lut (.A(rgb_hold[74]), .B(shift_register[73]), 
         .C(state[1]), .Z(shift_register_95__N_2665[74])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_74_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_75_i3_3_lut (.A(rgb_hold[75]), .B(shift_register[74]), 
         .C(state[1]), .Z(shift_register_95__N_2665[75])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_75_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_76_i3_3_lut (.A(rgb_hold[76]), .B(shift_register[75]), 
         .C(state[1]), .Z(shift_register_95__N_2665[76])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_76_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_77_i3_3_lut (.A(rgb_hold[77]), .B(shift_register[76]), 
         .C(state[1]), .Z(shift_register_95__N_2665[77])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_77_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_78_i3_3_lut (.A(rgb_hold[78]), .B(shift_register[77]), 
         .C(state[1]), .Z(shift_register_95__N_2665[78])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_78_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_79_i3_3_lut (.A(rgb_hold[79]), .B(shift_register[78]), 
         .C(state[1]), .Z(shift_register_95__N_2665[79])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_79_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_80_i3_3_lut (.A(rgb_hold[88]), .B(shift_register[79]), 
         .C(state[1]), .Z(shift_register_95__N_2665[80])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_80_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_81_i3_3_lut (.A(rgb_hold[89]), .B(shift_register[80]), 
         .C(state[1]), .Z(shift_register_95__N_2665[81])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_81_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_82_i3_3_lut (.A(rgb_hold[90]), .B(shift_register[81]), 
         .C(state[1]), .Z(shift_register_95__N_2665[82])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_82_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_83_i3_3_lut (.A(rgb_hold[91]), .B(shift_register[82]), 
         .C(state[1]), .Z(shift_register_95__N_2665[83])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_83_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_84_i3_3_lut (.A(rgb_hold[92]), .B(shift_register[83]), 
         .C(state[1]), .Z(shift_register_95__N_2665[84])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_84_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_85_i3_3_lut (.A(rgb_hold[93]), .B(shift_register[84]), 
         .C(state[1]), .Z(shift_register_95__N_2665[85])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_85_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_86_i3_3_lut (.A(rgb_hold[94]), .B(shift_register[85]), 
         .C(state[1]), .Z(shift_register_95__N_2665[86])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_86_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_87_i3_3_lut (.A(rgb_hold[95]), .B(shift_register[86]), 
         .C(state[1]), .Z(shift_register_95__N_2665[87])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_87_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_88_i3_3_lut (.A(rgb_hold[80]), .B(shift_register[87]), 
         .C(state[1]), .Z(shift_register_95__N_2665[88])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_88_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_89_i3_3_lut (.A(rgb_hold[81]), .B(shift_register[88]), 
         .C(state[1]), .Z(shift_register_95__N_2665[89])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_89_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_90_i3_3_lut (.A(rgb_hold[82]), .B(shift_register[89]), 
         .C(state[1]), .Z(shift_register_95__N_2665[90])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_90_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_91_i3_3_lut (.A(rgb_hold[83]), .B(shift_register[90]), 
         .C(state[1]), .Z(shift_register_95__N_2665[91])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_91_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_92_i3_3_lut (.A(rgb_hold[84]), .B(shift_register[91]), 
         .C(state[1]), .Z(shift_register_95__N_2665[92])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_92_i3_3_lut.init = 16'hcaca;
    LUT4 state_1__I_0_41_Mux_93_i3_4_lut (.A(rgb_hold[85]), .B(shift_register[92]), 
         .C(state[1]), .D(n23158), .Z(shift_register_95__N_2665[93])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_93_i3_4_lut.init = 16'h0aca;
    LUT4 state_1__I_0_41_Mux_94_i3_4_lut (.A(rgb_hold[86]), .B(shift_register[93]), 
         .C(state[1]), .D(n23158), .Z(shift_register_95__N_2665[94])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_94_i3_4_lut.init = 16'h0aca;
    LUT4 state_1__I_0_41_Mux_95_i3_4_lut (.A(rgb_hold[87]), .B(shift_register[94]), 
         .C(state[1]), .D(n23158), .Z(shift_register_95__N_2665[95])) /* synthesis lut_function=(!(A (B (C (D))+!B (C))+!A (((D)+!C)+!B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(25[5] 54[12])
    defparam state_1__I_0_41_Mux_95_i3_4_lut.init = 16'h0aca;
    LUT4 i1_4_lut_adj_22 (.A(state[0]), .B(n7), .C(n23150), .D(state[1]), 
         .Z(pll_clk_enable_597)) /* synthesis lut_function=(A+(B (C (D))+!B (C+!(D)))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(16[11:16])
    defparam i1_4_lut_adj_22.init = 16'hfabb;
    LUT4 i10133_2_lut_rep_109 (.A(reset_count[5]), .B(reset_count[12]), 
         .Z(n23191)) /* synthesis lut_function=(A (B)) */ ;
    defparam i10133_2_lut_rep_109.init = 16'h8888;
    LUT4 i1_3_lut_4_lut_adj_23 (.A(reset_count[5]), .B(reset_count[12]), 
         .C(n19059), .D(reset_count[9]), .Z(n7)) /* synthesis lut_function=((((D)+!C)+!B)+!A) */ ;
    defparam i1_3_lut_4_lut_adj_23.init = 16'hff7f;
    LUT4 i14273_3_lut (.A(state[0]), .B(n7), .C(state[1]), .Z(n21933)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;
    defparam i14273_3_lut.init = 16'h0101;
    LUT4 i9_4_lut (.A(n17), .B(reset_count[1]), .C(n16), .D(reset_count[11]), 
         .Z(n19059)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i9_4_lut.init = 16'h8000;
    LUT4 i7_4_lut (.A(reset_count[3]), .B(reset_count[2]), .C(reset_count[6]), 
         .D(reset_count[4]), .Z(n17)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i7_4_lut.init = 16'h8000;
    LUT4 i6_4_lut (.A(reset_count[10]), .B(reset_count[8]), .C(reset_count[0]), 
         .D(reset_count[7]), .Z(n16)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i6_4_lut.init = 16'h8000;
    FD1P3AX shift_register_i1 (.D(shift_register_95__N_2665[1]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i1.GSR = "DISABLED";
    FD1P3AX shift_register_i2 (.D(shift_register_95__N_2665[2]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i2.GSR = "DISABLED";
    FD1P3AX shift_register_i3 (.D(shift_register_95__N_2665[3]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i3.GSR = "DISABLED";
    FD1P3AX shift_register_i4 (.D(shift_register_95__N_2665[4]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i4.GSR = "DISABLED";
    FD1P3AX shift_register_i5 (.D(shift_register_95__N_2665[5]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i5.GSR = "DISABLED";
    FD1P3AX shift_register_i6 (.D(shift_register_95__N_2665[6]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i6.GSR = "DISABLED";
    FD1P3AX shift_register_i7 (.D(shift_register_95__N_2665[7]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i7.GSR = "DISABLED";
    FD1P3AX shift_register_i8 (.D(shift_register_95__N_2665[8]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i8.GSR = "DISABLED";
    FD1P3AX shift_register_i9 (.D(shift_register_95__N_2665[9]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i9.GSR = "DISABLED";
    FD1P3AX shift_register_i10 (.D(shift_register_95__N_2665[10]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i10.GSR = "DISABLED";
    FD1P3AX shift_register_i11 (.D(shift_register_95__N_2665[11]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i11.GSR = "DISABLED";
    FD1P3AX shift_register_i12 (.D(shift_register_95__N_2665[12]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i12.GSR = "DISABLED";
    FD1P3AX shift_register_i13 (.D(shift_register_95__N_2665[13]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i13.GSR = "DISABLED";
    FD1P3AX shift_register_i14 (.D(shift_register_95__N_2665[14]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i14.GSR = "DISABLED";
    FD1P3AX shift_register_i15 (.D(shift_register_95__N_2665[15]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[15])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i15.GSR = "DISABLED";
    FD1P3AX shift_register_i16 (.D(shift_register_95__N_2665[16]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[16])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i16.GSR = "DISABLED";
    FD1P3AX shift_register_i17 (.D(shift_register_95__N_2665[17]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[17])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i17.GSR = "DISABLED";
    FD1P3AX shift_register_i18 (.D(shift_register_95__N_2665[18]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[18])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i18.GSR = "DISABLED";
    FD1P3AX shift_register_i19 (.D(shift_register_95__N_2665[19]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[19])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i19.GSR = "DISABLED";
    FD1P3AX shift_register_i20 (.D(shift_register_95__N_2665[20]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[20])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i20.GSR = "DISABLED";
    FD1P3AX shift_register_i21 (.D(shift_register_95__N_2665[21]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[21])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i21.GSR = "DISABLED";
    FD1P3AX shift_register_i22 (.D(shift_register_95__N_2665[22]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[22])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i22.GSR = "DISABLED";
    FD1P3AX shift_register_i23 (.D(shift_register_95__N_2665[23]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i23.GSR = "DISABLED";
    FD1P3AX shift_register_i24 (.D(shift_register_95__N_2665[24]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[24])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i24.GSR = "DISABLED";
    FD1P3AX shift_register_i25 (.D(shift_register_95__N_2665[25]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[25])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i25.GSR = "DISABLED";
    FD1P3AX shift_register_i26 (.D(shift_register_95__N_2665[26]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[26])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i26.GSR = "DISABLED";
    FD1P3AX shift_register_i27 (.D(shift_register_95__N_2665[27]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[27])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i27.GSR = "DISABLED";
    FD1P3AX shift_register_i28 (.D(shift_register_95__N_2665[28]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[28])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i28.GSR = "DISABLED";
    FD1P3AX shift_register_i29 (.D(shift_register_95__N_2665[29]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[29])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i29.GSR = "DISABLED";
    FD1P3AX shift_register_i30 (.D(shift_register_95__N_2665[30]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[30])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i30.GSR = "DISABLED";
    FD1P3AX shift_register_i31 (.D(shift_register_95__N_2665[31]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[31])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i31.GSR = "DISABLED";
    FD1P3AX shift_register_i32 (.D(shift_register_95__N_2665[32]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[32])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i32.GSR = "DISABLED";
    FD1P3AX shift_register_i33 (.D(shift_register_95__N_2665[33]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[33])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i33.GSR = "DISABLED";
    FD1P3AX shift_register_i34 (.D(shift_register_95__N_2665[34]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[34])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i34.GSR = "DISABLED";
    FD1P3AX shift_register_i35 (.D(shift_register_95__N_2665[35]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[35])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i35.GSR = "DISABLED";
    FD1P3AX shift_register_i36 (.D(shift_register_95__N_2665[36]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[36])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i36.GSR = "DISABLED";
    FD1P3AX shift_register_i37 (.D(shift_register_95__N_2665[37]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[37])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i37.GSR = "DISABLED";
    FD1P3AX shift_register_i38 (.D(shift_register_95__N_2665[38]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[38])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i38.GSR = "DISABLED";
    FD1P3AX shift_register_i39 (.D(shift_register_95__N_2665[39]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[39])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i39.GSR = "DISABLED";
    FD1P3AX shift_register_i40 (.D(shift_register_95__N_2665[40]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[40])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i40.GSR = "DISABLED";
    FD1P3AX shift_register_i41 (.D(shift_register_95__N_2665[41]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[41])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i41.GSR = "DISABLED";
    FD1P3AX shift_register_i42 (.D(shift_register_95__N_2665[42]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[42])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i42.GSR = "DISABLED";
    FD1P3AX shift_register_i43 (.D(shift_register_95__N_2665[43]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[43])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i43.GSR = "DISABLED";
    FD1P3AX shift_register_i44 (.D(shift_register_95__N_2665[44]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[44])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i44.GSR = "DISABLED";
    FD1P3AX shift_register_i45 (.D(shift_register_95__N_2665[45]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[45])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i45.GSR = "DISABLED";
    FD1P3AX shift_register_i46 (.D(shift_register_95__N_2665[46]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[46])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i46.GSR = "DISABLED";
    FD1P3AX shift_register_i47 (.D(shift_register_95__N_2665[47]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[47])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i47.GSR = "DISABLED";
    FD1P3AX shift_register_i48 (.D(shift_register_95__N_2665[48]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[48])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i48.GSR = "DISABLED";
    FD1P3AX shift_register_i49 (.D(shift_register_95__N_2665[49]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[49])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i49.GSR = "DISABLED";
    FD1P3AX shift_register_i50 (.D(shift_register_95__N_2665[50]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[50])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i50.GSR = "DISABLED";
    FD1P3AX shift_register_i51 (.D(shift_register_95__N_2665[51]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[51])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i51.GSR = "DISABLED";
    FD1P3AX shift_register_i52 (.D(shift_register_95__N_2665[52]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[52])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i52.GSR = "DISABLED";
    FD1P3AX shift_register_i53 (.D(shift_register_95__N_2665[53]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[53])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i53.GSR = "DISABLED";
    FD1P3AX shift_register_i54 (.D(shift_register_95__N_2665[54]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[54])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i54.GSR = "DISABLED";
    FD1P3AX shift_register_i55 (.D(shift_register_95__N_2665[55]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[55])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i55.GSR = "DISABLED";
    FD1P3AX shift_register_i56 (.D(shift_register_95__N_2665[56]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[56])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i56.GSR = "DISABLED";
    FD1P3AX shift_register_i57 (.D(shift_register_95__N_2665[57]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[57])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i57.GSR = "DISABLED";
    FD1P3AX shift_register_i58 (.D(shift_register_95__N_2665[58]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[58])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i58.GSR = "DISABLED";
    FD1P3AX shift_register_i59 (.D(shift_register_95__N_2665[59]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[59])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i59.GSR = "DISABLED";
    FD1P3AX shift_register_i60 (.D(shift_register_95__N_2665[60]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[60])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i60.GSR = "DISABLED";
    FD1P3AX shift_register_i61 (.D(shift_register_95__N_2665[61]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[61])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i61.GSR = "DISABLED";
    FD1P3AX shift_register_i62 (.D(shift_register_95__N_2665[62]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[62])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i62.GSR = "DISABLED";
    FD1P3AX shift_register_i63 (.D(shift_register_95__N_2665[63]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[63])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i63.GSR = "DISABLED";
    FD1P3AX shift_register_i64 (.D(shift_register_95__N_2665[64]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[64])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i64.GSR = "DISABLED";
    FD1P3AX shift_register_i65 (.D(shift_register_95__N_2665[65]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[65])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i65.GSR = "DISABLED";
    FD1P3AX shift_register_i66 (.D(shift_register_95__N_2665[66]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[66])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i66.GSR = "DISABLED";
    FD1P3AX shift_register_i67 (.D(shift_register_95__N_2665[67]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[67])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i67.GSR = "DISABLED";
    FD1P3AX shift_register_i68 (.D(shift_register_95__N_2665[68]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[68])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i68.GSR = "DISABLED";
    FD1P3AX shift_register_i69 (.D(shift_register_95__N_2665[69]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[69])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i69.GSR = "DISABLED";
    FD1P3AX shift_register_i70 (.D(shift_register_95__N_2665[70]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[70])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i70.GSR = "DISABLED";
    FD1P3AX shift_register_i71 (.D(shift_register_95__N_2665[71]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[71])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i71.GSR = "DISABLED";
    FD1P3AX shift_register_i72 (.D(shift_register_95__N_2665[72]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[72])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i72.GSR = "DISABLED";
    FD1P3AX shift_register_i73 (.D(shift_register_95__N_2665[73]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[73])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i73.GSR = "DISABLED";
    FD1P3AX shift_register_i74 (.D(shift_register_95__N_2665[74]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[74])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i74.GSR = "DISABLED";
    FD1P3AX shift_register_i75 (.D(shift_register_95__N_2665[75]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[75])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i75.GSR = "DISABLED";
    FD1P3AX shift_register_i76 (.D(shift_register_95__N_2665[76]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[76])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i76.GSR = "DISABLED";
    FD1P3AX shift_register_i77 (.D(shift_register_95__N_2665[77]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[77])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i77.GSR = "DISABLED";
    FD1P3AX shift_register_i78 (.D(shift_register_95__N_2665[78]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[78])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i78.GSR = "DISABLED";
    FD1P3AX shift_register_i79 (.D(shift_register_95__N_2665[79]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[79])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i79.GSR = "DISABLED";
    FD1P3AX shift_register_i80 (.D(shift_register_95__N_2665[80]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[80])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i80.GSR = "DISABLED";
    FD1P3AX shift_register_i81 (.D(shift_register_95__N_2665[81]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[81])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i81.GSR = "DISABLED";
    FD1P3AX shift_register_i82 (.D(shift_register_95__N_2665[82]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[82])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i82.GSR = "DISABLED";
    FD1P3AX shift_register_i83 (.D(shift_register_95__N_2665[83]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[83])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i83.GSR = "DISABLED";
    FD1P3AX shift_register_i84 (.D(shift_register_95__N_2665[84]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[84])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i84.GSR = "DISABLED";
    FD1P3AX shift_register_i85 (.D(shift_register_95__N_2665[85]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[85])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i85.GSR = "DISABLED";
    FD1P3AX shift_register_i86 (.D(shift_register_95__N_2665[86]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[86])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i86.GSR = "DISABLED";
    FD1P3AX shift_register_i87 (.D(shift_register_95__N_2665[87]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[87])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i87.GSR = "DISABLED";
    FD1P3AX shift_register_i88 (.D(shift_register_95__N_2665[88]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[88])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i88.GSR = "DISABLED";
    FD1P3AX shift_register_i89 (.D(shift_register_95__N_2665[89]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[89])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i89.GSR = "DISABLED";
    FD1P3AX shift_register_i90 (.D(shift_register_95__N_2665[90]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[90])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i90.GSR = "DISABLED";
    FD1P3AX shift_register_i91 (.D(shift_register_95__N_2665[91]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[91])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i91.GSR = "DISABLED";
    FD1P3AX shift_register_i92 (.D(shift_register_95__N_2665[92]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[92])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i92.GSR = "DISABLED";
    FD1P3AX shift_register_i93 (.D(shift_register_95__N_2665[93]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[93])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i93.GSR = "DISABLED";
    FD1P3AX shift_register_i94 (.D(shift_register_95__N_2665[94]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(shift_register[94])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i94.GSR = "DISABLED";
    FD1P3AX shift_register_i95 (.D(shift_register_95__N_2665[95]), .SP(pll_clk_enable_727), 
            .CK(pll_clk), .Q(high_count[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i95.GSR = "DISABLED";
    FD1P3AX state_i1 (.D(state_1__N_2635[1]), .SP(pll_clk_enable_597), .CK(pll_clk), 
            .Q(state[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam state_i1.GSR = "DISABLED";
    CCU2D add_911_7 (.A0(bit_number[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_number[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21828), .S0(bit_number_6__N_2895[5]), .S1(bit_number_6__N_2895[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26] 48[20])
    defparam add_911_7.INIT0 = 16'h5aaa;
    defparam add_911_7.INIT1 = 16'h5aaa;
    defparam add_911_7.INJECT1_0 = "NO";
    defparam add_911_7.INJECT1_1 = "NO";
    CCU2D add_911_5 (.A0(bit_number[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_number[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21827), .COUT(n21828), .S0(bit_number_6__N_2895[3]), .S1(bit_number_6__N_2895[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26] 48[20])
    defparam add_911_5.INIT0 = 16'h5aaa;
    defparam add_911_5.INIT1 = 16'h5aaa;
    defparam add_911_5.INJECT1_0 = "NO";
    defparam add_911_5.INJECT1_1 = "NO";
    CCU2D add_911_3 (.A0(bit_number[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_number[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21826), .COUT(n21827), .S0(bit_number_6__N_2895[1]), .S1(bit_number_6__N_2895[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26] 48[20])
    defparam add_911_3.INIT0 = 16'h5aaa;
    defparam add_911_3.INIT1 = 16'h5aaa;
    defparam add_911_3.INJECT1_0 = "NO";
    defparam add_911_3.INJECT1_1 = "NO";
    CCU2D add_911_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_number[0]), .B1(n18987), .C1(bit_number[5]), .D1(n137), 
          .COUT(n21826), .S1(bit_number_6__N_2895[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(46[26] 48[20])
    defparam add_911_1.INIT0 = 16'hF000;
    defparam add_911_1.INIT1 = 16'h59aa;
    defparam add_911_1.INJECT1_0 = "NO";
    defparam add_911_1.INJECT1_1 = "NO";
    CCU2D add_17_9 (.A0(bit_cell_count[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n21825), .S0(n84[7]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(50[35:56])
    defparam add_17_9.INIT0 = 16'h5aaa;
    defparam add_17_9.INIT1 = 16'h0000;
    defparam add_17_9.INJECT1_0 = "NO";
    defparam add_17_9.INJECT1_1 = "NO";
    PFUMX i7478 (.BLUT(n132), .ALUT(n119), .C0(high_count[6]), .Z(n75));
    CCU2D add_17_7 (.A0(bit_cell_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21824), .COUT(n21825), .S0(n84[5]), .S1(n84[6]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(50[35:56])
    defparam add_17_7.INIT0 = 16'h5aaa;
    defparam add_17_7.INIT1 = 16'h5aaa;
    defparam add_17_7.INJECT1_0 = "NO";
    defparam add_17_7.INJECT1_1 = "NO";
    CCU2D add_909_3 (.A0(reset_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21791), .COUT(n21792), .S0(reset_count_12__N_2882[1]), 
          .S1(reset_count_12__N_2882[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[22] 31[16])
    defparam add_909_3.INIT0 = 16'h5aaa;
    defparam add_909_3.INIT1 = 16'h5aaa;
    defparam add_909_3.INJECT1_0 = "NO";
    defparam add_909_3.INJECT1_1 = "NO";
    CCU2D add_17_5 (.A0(bit_cell_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21823), .COUT(n21824), .S0(n84[3]), .S1(n84[4]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(50[35:56])
    defparam add_17_5.INIT0 = 16'h5aaa;
    defparam add_17_5.INIT1 = 16'h5aaa;
    defparam add_17_5.INJECT1_0 = "NO";
    defparam add_17_5.INJECT1_1 = "NO";
    CCU2D add_909_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[0]), .B1(n23191), .C1(n19059), .D1(reset_count[9]), 
          .COUT(n21791), .S1(reset_count_12__N_2882[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[22] 31[16])
    defparam add_909_1.INIT0 = 16'hF000;
    defparam add_909_1.INIT1 = 16'h5595;
    defparam add_909_1.INJECT1_0 = "NO";
    defparam add_909_1.INJECT1_1 = "NO";
    CCU2D add_17_3 (.A0(bit_cell_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21822), .COUT(n21823), .S0(n84[1]), .S1(n84[2]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(50[35:56])
    defparam add_17_3.INIT0 = 16'h5aaa;
    defparam add_17_3.INIT1 = 16'h5aaa;
    defparam add_17_3.INJECT1_0 = "NO";
    defparam add_17_3.INJECT1_1 = "NO";
    CCU2D add_17_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n21822), .S1(n84[0]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(50[35:56])
    defparam add_17_1.INIT0 = 16'hF000;
    defparam add_17_1.INIT1 = 16'h5555;
    defparam add_17_1.INJECT1_0 = "NO";
    defparam add_17_1.INJECT1_1 = "NO";
    LUT4 i15_2_lut (.A(state[1]), .B(state[0]), .Z(pll_clk_enable_728)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i15_2_lut.init = 16'h6666;
    LUT4 i6545_2_lut (.A(state[0]), .B(state[1]), .Z(state_1__N_2635[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i6545_2_lut.init = 16'h2222;
    LUT4 i10052_2_lut_3_lut (.A(state[1]), .B(n137), .C(n84[0]), .Z(bit_cell_count_7__N_2650[0])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i10052_2_lut_3_lut.init = 16'h2020;
    LUT4 i10029_2_lut_3_lut (.A(state[1]), .B(n137), .C(n84[1]), .Z(bit_cell_count_7__N_2650[1])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i10029_2_lut_3_lut.init = 16'h2020;
    LUT4 i5_3_lut (.A(bit_number[1]), .B(n10), .C(bit_number[4]), .Z(n18987)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i5_3_lut.init = 16'h8080;
    LUT4 i4_4_lut (.A(bit_number[2]), .B(bit_number[6]), .C(bit_number[3]), 
         .D(bit_number[0]), .Z(n10)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_3_lut (.A(state[1]), .B(n137), .C(n84[5]), .Z(bit_cell_count_7__N_2650[5])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i1_2_lut_3_lut.init = 16'h2020;
    LUT4 i5_4_lut (.A(bit_cell_count[5]), .B(n10_adj_2984), .C(n23178), 
         .D(bit_cell_count[6]), .Z(n137)) /* synthesis lut_function=(!(A+(((D)+!C)+!B))) */ ;
    defparam i5_4_lut.init = 16'h0040;
    LUT4 i10026_2_lut_3_lut (.A(state[1]), .B(n137), .C(n84[4]), .Z(bit_cell_count_7__N_2650[4])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i10026_2_lut_3_lut.init = 16'h2020;
    LUT4 i4_4_lut_adj_24 (.A(bit_cell_count[1]), .B(bit_cell_count[0]), 
         .C(bit_cell_count[7]), .D(bit_cell_count[4]), .Z(n10_adj_2984)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i4_4_lut_adj_24.init = 16'h8000;
    LUT4 i10027_2_lut_3_lut (.A(state[1]), .B(n137), .C(n84[3]), .Z(bit_cell_count_7__N_2650[3])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i10027_2_lut_3_lut.init = 16'h2020;
    LUT4 i1_2_lut_3_lut_adj_25 (.A(state[1]), .B(n137), .C(n84[7]), .Z(bit_cell_count_7__N_2650[7])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i1_2_lut_3_lut_adj_25.init = 16'h2020;
    LUT4 i10028_2_lut_3_lut (.A(state[1]), .B(n137), .C(n84[2]), .Z(bit_cell_count_7__N_2650[2])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i10028_2_lut_3_lut.init = 16'h2020;
    LUT4 i10025_2_lut_3_lut (.A(state[1]), .B(n137), .C(n84[6]), .Z(bit_cell_count_7__N_2650[6])) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam i10025_2_lut_3_lut.init = 16'h2020;
    LUT4 i1_3_lut_rep_62 (.A(state[1]), .B(state[0]), .C(n23150), .Z(pll_clk_enable_726)) /* synthesis lut_function=(!(A (B+!(C))+!A (B))) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(16[11:16])
    defparam i1_3_lut_rep_62.init = 16'h3131;
    LUT4 i6544_2_lut_3_lut (.A(state[1]), .B(state[0]), .C(n23150), .Z(n15394)) /* synthesis lut_function=(!((B+!(C))+!A)) */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(16[11:16])
    defparam i6544_2_lut_3_lut.init = 16'h2020;
    LUT4 state_0__bdd_3_lut (.A(state[0]), .B(n137), .C(state[1]), .Z(pll_clk_enable_727)) /* synthesis lut_function=(!(A (C)+!A !(B (C)))) */ ;
    defparam state_0__bdd_3_lut.init = 16'h4a4a;
    LUT4 i1_2_lut_rep_76 (.A(state[0]), .B(n137), .Z(n23158)) /* synthesis lut_function=(A+!(B)) */ ;
    defparam i1_2_lut_rep_76.init = 16'hbbbb;
    LUT4 i1_2_lut_3_lut_adj_26 (.A(state[0]), .B(n137), .C(state[1]), 
         .Z(n21964)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;
    defparam i1_2_lut_3_lut_adj_26.init = 16'h4040;
    FD1P3IX bit_number_i0 (.D(bit_number_6__N_2895[0]), .SP(pll_clk_enable_728), 
            .CD(state_1__N_2635[1]), .CK(pll_clk), .Q(bit_number[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_number_i0.GSR = "DISABLED";
    FD1P3IX reset_count_i0 (.D(reset_count_12__N_2882[0]), .SP(pll_clk_enable_726), 
            .CD(n15394), .CK(pll_clk), .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam reset_count_i0.GSR = "DISABLED";
    CCU2D add_909_13 (.A0(reset_count[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21796), .S0(reset_count_12__N_2882[11]), 
          .S1(reset_count_12__N_2882[12]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[22] 31[16])
    defparam add_909_13.INIT0 = 16'h5aaa;
    defparam add_909_13.INIT1 = 16'h5aaa;
    defparam add_909_13.INJECT1_0 = "NO";
    defparam add_909_13.INJECT1_1 = "NO";
    FD1P3IX shift_register_i0 (.D(rgb_hold[0]), .SP(pll_clk_enable_727), 
            .CD(n21964), .CK(pll_clk), .Q(shift_register[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam shift_register_i0.GSR = "DISABLED";
    FD1P3AX bit_cell_count_i0 (.D(bit_cell_count_7__N_2650[0]), .SP(pll_clk_enable_728), 
            .CK(pll_clk), .Q(bit_cell_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=1, LSE_LCOL=19, LSE_RCOL=6, LSE_LLINE=514, LSE_RLINE=521 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(24[8] 55[4])
    defparam bit_cell_count_i0.GSR = "DISABLED";
    CCU2D add_909_11 (.A0(reset_count[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n21795), .COUT(n21796), .S0(reset_count_12__N_2882[9]), 
          .S1(reset_count_12__N_2882[10]));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/ws2812_stream.v(29[22] 31[16])
    defparam add_909_11.INIT0 = 16'h5aaa;
    defparam add_909_11.INIT1 = 16'h5aaa;
    defparam add_909_11.INJECT1_0 = "NO";
    defparam add_909_11.INJECT1_1 = "NO";
    
endmodule
//
// Verilog Description of module umh_toggle_ram84
//

module umh_toggle_ram84 (pll_clk, ev_we, VCC_net, GND_net, \ev_wr_addr[0] , 
            event_rd_addr, \ev_wr_addr[1] , \ev_wr_addr[2] , \ev_wr_addr[3] , 
            \ev_wr_addr[4] , \ev_wr_addr[5] , \ev_wr_addr[6] , \ev_wr_addr[7] , 
            n23200, ev_wr_data, ev_rd_data) /* synthesis syn_module_defined=1 */ ;
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
    input n23200;
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23200), 
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
            .ADW8(n23200), .BE0(VCC_net), .BE1(VCC_net), .CEW(ev_we), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23200), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23200), 
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
            .ADW6(\ev_wr_addr[6] ), .ADW7(\ev_wr_addr[7] ), .ADW8(n23200), 
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

module umh_channel_ram18 (n9803, spi1_sck_c, spi_channel_index, n9809, 
            staging_q, pll_clk, rd_data_15__N_2535, n9799, n9797, 
            n9805, n9807, spi_write, VCC_net, GND_net, staging_rd_addr_6__N_785, 
            spi1_mosi_c_0, \spi_rx_shift[0] , \spi_rx_shift[1] , \spi_rx_shift[2] , 
            \spi_rx_shift[3] , \spi_rx_shift[4] , \spi_rx_shift[5] , \spi_rx_shift[6] , 
            spi_phase_pending, n9814, n9816, n9818, n9820, n9822, 
            n9824, n9826, n9828, n9830, n9832, n9834, n9836, n9838, 
            n9840, n9842, n9844, n9801) /* synthesis syn_module_defined=1 */ ;
    output n9803;
    input spi1_sck_c;
    input [6:0]spi_channel_index;
    output n9809;
    output [15:0]staging_q;
    input pll_clk;
    input [15:0]rd_data_15__N_2535;
    output n9799;
    output n9797;
    output n9805;
    output n9807;
    input spi_write;
    input VCC_net;
    input GND_net;
    input [6:0]staging_rd_addr_6__N_785;
    input spi1_mosi_c_0;
    input \spi_rx_shift[0] ;
    input \spi_rx_shift[1] ;
    input \spi_rx_shift[2] ;
    input \spi_rx_shift[3] ;
    input \spi_rx_shift[4] ;
    input \spi_rx_shift[5] ;
    input \spi_rx_shift[6] ;
    input [7:0]spi_phase_pending;
    output n9814;
    output n9816;
    output n9818;
    output n9820;
    output n9822;
    output n9824;
    output n9826;
    output n9828;
    output n9830;
    output n9832;
    output n9834;
    output n9836;
    output n9838;
    output n9840;
    output n9842;
    output n9844;
    output n9801;
    
    wire spi1_sck_c /* synthesis SET_AS_NETWORK=spi1_sck_c, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(61[24:32])
    wire pll_clk /* synthesis SET_AS_NETWORK=pll_clk, is_clock=1 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(93[10:17])
    
    FD1S3AX mem_1184 (.D(spi_channel_index[3]), .CK(spi1_sck_c), .Q(n9803));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1184.GSR = "DISABLED";
    FD1S3AX mem_1190 (.D(spi_channel_index[6]), .CK(spi1_sck_c), .Q(n9809));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1190.GSR = "DISABLED";
    FD1S3AX rd_data_i0 (.D(rd_data_15__N_2535[0]), .CK(pll_clk), .Q(staging_q[0])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i0.GSR = "DISABLED";
    FD1S3AX mem_1180 (.D(spi_channel_index[1]), .CK(spi1_sck_c), .Q(n9799));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1180.GSR = "DISABLED";
    FD1S3AX mem_1178 (.D(spi_channel_index[0]), .CK(spi1_sck_c), .Q(n9797));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1178.GSR = "DISABLED";
    FD1S3AX mem_1186 (.D(spi_channel_index[4]), .CK(spi1_sck_c), .Q(n9805));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1186.GSR = "DISABLED";
    FD1S3AX mem_1188 (.D(spi_channel_index[5]), .CK(spi1_sck_c), .Q(n9807));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1188.GSR = "DISABLED";
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
            .ADR1(GND_net), .ADR2(GND_net), .ADR3(GND_net), .ADR4(staging_rd_addr_6__N_785[0]), 
            .ADR5(staging_rd_addr_6__N_785[1]), .ADR6(staging_rd_addr_6__N_785[2]), 
            .ADR7(staging_rd_addr_6__N_785[3]), .ADR8(staging_rd_addr_6__N_785[4]), 
            .ADR9(staging_rd_addr_6__N_785[5]), .ADR10(staging_rd_addr_6__N_785[6]), 
            .ADR11(GND_net), .ADR12(GND_net), .CER(VCC_net), .OCER(VCC_net), 
            .CLKR(pll_clk), .CSR0(GND_net), .CSR1(GND_net), .CSR2(GND_net), 
            .RST(GND_net), .DO0(n9832), .DO1(n9834), .DO2(n9836), .DO3(n9838), 
            .DO4(n9840), .DO5(n9842), .DO6(n9844), .DO9(n9814), .DO10(n9816), 
            .DO11(n9818), .DO12(n9820), .DO13(n9822), .DO14(n9824), 
            .DO15(n9826), .DO16(n9828), .DO17(n9830));
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
    FD1S3AX mem_1182 (.D(spi_channel_index[2]), .CK(spi1_sck_c), .Q(n9801));   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(35[41:53])
    defparam mem_1182.GSR = "DISABLED";
    FD1S3AX rd_data_i1 (.D(rd_data_15__N_2535[1]), .CK(pll_clk), .Q(staging_q[1])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i1.GSR = "DISABLED";
    FD1S3AX rd_data_i2 (.D(rd_data_15__N_2535[2]), .CK(pll_clk), .Q(staging_q[2])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i2.GSR = "DISABLED";
    FD1S3AX rd_data_i3 (.D(rd_data_15__N_2535[3]), .CK(pll_clk), .Q(staging_q[3])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i3.GSR = "DISABLED";
    FD1S3AX rd_data_i4 (.D(rd_data_15__N_2535[4]), .CK(pll_clk), .Q(staging_q[4])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i4.GSR = "DISABLED";
    FD1S3AX rd_data_i5 (.D(rd_data_15__N_2535[5]), .CK(pll_clk), .Q(staging_q[5])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i5.GSR = "DISABLED";
    FD1S3AX rd_data_i6 (.D(rd_data_15__N_2535[6]), .CK(pll_clk), .Q(staging_q[6])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i6.GSR = "DISABLED";
    FD1S3AX rd_data_i7 (.D(rd_data_15__N_2535[7]), .CK(pll_clk), .Q(staging_q[7])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i7.GSR = "DISABLED";
    FD1S3AX rd_data_i8 (.D(rd_data_15__N_2535[8]), .CK(pll_clk), .Q(staging_q[8])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i8.GSR = "DISABLED";
    FD1S3AX rd_data_i9 (.D(rd_data_15__N_2535[9]), .CK(pll_clk), .Q(staging_q[9])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i9.GSR = "DISABLED";
    FD1S3AX rd_data_i10 (.D(rd_data_15__N_2535[10]), .CK(pll_clk), .Q(staging_q[10])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i10.GSR = "DISABLED";
    FD1S3AX rd_data_i11 (.D(rd_data_15__N_2535[11]), .CK(pll_clk), .Q(staging_q[11])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i11.GSR = "DISABLED";
    FD1S3AX rd_data_i12 (.D(rd_data_15__N_2535[12]), .CK(pll_clk), .Q(staging_q[12])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i12.GSR = "DISABLED";
    FD1S3AX rd_data_i13 (.D(rd_data_15__N_2535[13]), .CK(pll_clk), .Q(staging_q[13])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i13.GSR = "DISABLED";
    FD1S3AX rd_data_i14 (.D(rd_data_15__N_2535[14]), .CK(pll_clk), .Q(staging_q[14])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i14.GSR = "DISABLED";
    FD1S3AX rd_data_i15 (.D(rd_data_15__N_2535[15]), .CK(pll_clk), .Q(staging_q[15])) /* synthesis LSE_LINE_FILE_ID=1, LSE_LCOL=23, LSE_RCOL=6, LSE_LLINE=193, LSE_RLINE=196 */ ;   // d:/data/onedrive/projects/umh/software/umh controller/fpga/src/umh_fpga_top.v(36[12:54])
    defparam rd_data_i15.GSR = "DISABLED";
    
endmodule
