// Verilog netlist produced by program LSE :  version Diamond (64-bit) 3.13.0.56.2
// Netlist written on Tue Sep 08 18:58:50 2026
//
// Verilog Description of module umh_fpga_top
//

module umh_fpga_top (fpga_clk, fpga_cs_n, spi1_sck, spi1_mosi, spi1_miso, 
            us_tx, rgb_data, mic_clk, mic_data_0, mic_data_1, spi_mic_cs_n, 
            spi_mic_sck, spi_mic_miso) /* synthesis syn_module_defined=1 */ ;   // src/umh_fpga_top.v(5[8:20])
    input fpga_clk;   // src/umh_fpga_top.v(6[24:32])
    input fpga_cs_n;   // src/umh_fpga_top.v(7[24:33])
    input spi1_sck;   // src/umh_fpga_top.v(8[24:32])
    input spi1_mosi;   // src/umh_fpga_top.v(9[24:33])
    output spi1_miso;   // src/umh_fpga_top.v(10[24:33])
    output [83:0]us_tx;   // src/umh_fpga_top.v(11[24:29])
    output rgb_data;   // src/umh_fpga_top.v(12[24:32])
    output mic_clk;   // src/umh_fpga_top.v(13[24:31])
    input mic_data_0;   // src/umh_fpga_top.v(14[24:34])
    input mic_data_1;   // src/umh_fpga_top.v(15[24:34])
    input spi_mic_cs_n;   // src/umh_fpga_top.v(16[24:36])
    input spi_mic_sck;   // src/umh_fpga_top.v(17[24:35])
    output spi_mic_miso;   // src/umh_fpga_top.v(18[24:36])
    
    wire fpga_clk_c /* synthesis SET_AS_NETWORK=fpga_clk_c, is_clock=1 */ ;   // src/umh_fpga_top.v(6[24:32])
    wire fpga_cs_n_c /* synthesis SET_AS_NETWORK=fpga_cs_n_c, is_clock=1 */ ;   // src/umh_fpga_top.v(7[24:33])
    wire spi1_sck_c /* synthesis is_clock=1, SET_AS_NETWORK=spi1_sck_c */ ;   // src/umh_fpga_top.v(8[24:32])
    wire spi_mic_sck_c /* synthesis is_clock=1 */ ;   // src/umh_fpga_top.v(17[24:35])
    wire [0:83]phase_active /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(30[44:56])
    wire [1:0]\level_active[0]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[1]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[2]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[3]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[4]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[5]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[6]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[7]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[8]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[9]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[10]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[11]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[12]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[13]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[14]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[15]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[16]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[17]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[18]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[19]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[20]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[21]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[22]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[23]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[24]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[25]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[26]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[27]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[28]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[29]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[30]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[31]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[32]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[33]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[34]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[35]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[36]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[37]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[38]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[39]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[40]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[41]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[42]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[43]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[44]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[45]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[46]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[47]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[48]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[49]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[50]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[51]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[52]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[53]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[54]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[55]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[56]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[57]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[58]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[59]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[60]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[61]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[62]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[63]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[64]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[65]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[66]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[67]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[68]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[69]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[70]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[71]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[72]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[73]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[74]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[75]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[76]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[77]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[78]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[79]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[80]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[81]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[82]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire [1:0]\level_active[83]  /* synthesis syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(32[44:56])
    wire spi1_sck_N_1872 /* synthesis is_inv_clock=1 */ ;   // src/umh_fpga_top.v(51[11:27])
    wire sck_N_3318 /* synthesis is_inv_clock=1 */ ;   // src/spi_mic_stream.v(11[12:26])
    
    wire GND_net, VCC_net, spi1_mosi_c, spi1_miso_c, us_tx_c_83, us_tx_c_82, 
        us_tx_c_81, us_tx_c_80, us_tx_c_79, us_tx_c_78, us_tx_c_77, 
        us_tx_c_76, us_tx_c_75, us_tx_c_74, us_tx_c_73, us_tx_c_72, 
        us_tx_c_71, us_tx_c_70, us_tx_c_69, us_tx_c_68, us_tx_c_67, 
        us_tx_c_66, us_tx_c_65, us_tx_c_64, us_tx_c_63, us_tx_c_62, 
        us_tx_c_61, us_tx_c_60, us_tx_c_59, us_tx_c_58, us_tx_c_57, 
        us_tx_c_56, us_tx_c_55, us_tx_c_54, us_tx_c_53, us_tx_c_52, 
        us_tx_c_51, us_tx_c_50, us_tx_c_49, us_tx_c_48, us_tx_c_47, 
        us_tx_c_46, us_tx_c_45, us_tx_c_44, us_tx_c_43, us_tx_c_42, 
        us_tx_c_41, us_tx_c_40, us_tx_c_39, us_tx_c_38, us_tx_c_37, 
        us_tx_c_36, us_tx_c_35, us_tx_c_34, us_tx_c_33, us_tx_c_32, 
        us_tx_c_31, us_tx_c_30, us_tx_c_29, us_tx_c_28, us_tx_c_27, 
        us_tx_c_26, us_tx_c_25, us_tx_c_24, us_tx_c_23, us_tx_c_22, 
        us_tx_c_21, us_tx_c_20, us_tx_c_19, us_tx_c_18, us_tx_c_17, 
        us_tx_c_16, us_tx_c_15, us_tx_c_14, us_tx_c_13, us_tx_c_12, 
        us_tx_c_11, us_tx_c_10, us_tx_c_9, us_tx_c_8, us_tx_c_7, us_tx_c_6, 
        us_tx_c_5, us_tx_c_4, us_tx_c_3, us_tx_c_2, us_tx_c_1, us_tx_c_0, 
        rgb_data_c, mic_clk_c, mic_data_0_c, mic_data_1_c, spi_mic_cs_n_c, 
        spi_mic_miso_c;
    wire [95:0]rgb_values;   // src/umh_fpga_top.v(33[13:23])
    wire [7:0]spi_rx_shift;   // src/umh_fpga_top.v(35[11:23])
    wire [2:0]spi_bit_count;   // src/umh_fpga_top.v(36[11:24])
    wire [15:0]spi_byte_count;   // src/umh_fpga_top.v(37[12:26])
    wire [7:0]spi_command;   // src/umh_fpga_top.v(38[11:22])
    wire [7:0]spi_version;   // src/umh_fpga_top.v(39[11:22])
    
    wire n9529, n26, n9484, n9528, n25;
    wire [15:0]spi_update_flags;   // src/umh_fpga_top.v(40[12:28])
    wire [15:0]spi_extension_length;   // src/umh_fpga_top.v(41[12:32])
    wire [31:0]spi_frame_sequence;   // src/umh_fpga_top.v(42[12:30])
    wire [31:0]spi_expected_length;   // src/umh_fpga_top.v(43[12:31])
    wire [6:0]spi_channel_index;   // src/umh_fpga_top.v(44[11:28])
    wire [1:0]spi_channel_field;   // src/umh_fpga_top.v(45[11:28])
    wire [7:0]spi_level_pending;   // src/umh_fpga_top.v(46[11:28])
    wire [31:0]accepted_sequence_spi;   // src/umh_fpga_top.v(47[12:33])
    
    wire frame_toggle_spi, stop_toggle_spi;
    wire [6:0]status_bit_index;   // src/umh_fpga_top.v(51[11:27])
    
    wire frame_toggle_meta, frame_toggle_sync, frame_toggle_seen, stop_toggle_meta, 
        stop_toggle_sync, stop_toggle_seen;
    wire [31:0]accepted_sequence;   // src/umh_fpga_top.v(61[12:29])
    wire [31:0]phase_acc;   // src/umh_fpga_top.v(62[12:21])
    wire [31:0]fpga_time;   // src/umh_fpga_top.v(63[12:21])
    wire [5:0]time_divider;   // src/umh_fpga_top.v(64[11:23])
    
    wire n31;
    wire [7:0]amplitude_phase;   // src/umh_fpga_top.v(66[11:26])
    
    wire n18637, n158, n9481;
    wire [6:0]mic_divider;   // src/umh_fpga_top.v(67[11:22])
    wire [15:0]mic_shift_0;   // src/umh_fpga_top.v(69[12:23])
    wire [15:0]mic_shift_1;   // src/umh_fpga_top.v(70[12:23])
    wire [4:0]mic_sample_count;   // src/umh_fpga_top.v(71[11:27])
    wire [31:0]mic_latest;   // src/umh_fpga_top.v(72[12:22])
    wire [6:0]load_index;   // src/umh_fpga_top.v(73[11:21])
    
    wire load_active, rgb_values_95__N_787, rgb_values_95__N_445, rgb_values_95__N_781, 
        rgb_values_95__N_441, rgb_values_95__N_775, spi_extension_length_15__N_1319, 
        spi_extension_length_15__N_1264, spi_extension_length_15__N_1313, 
        spi_extension_length_15__N_1250, spi_extension_length_15__N_1282;
    wire [8:0]level_mem_din;   // src/umh_fpga_top.v(80[12:25])
    
    wire rgb_values_95__N_769, rgb_values_95__N_433, rgb_values_95__N_763, 
        rgb_values_95__N_429, rgb_values_95__N_757;
    wire [8:0]phase_mem_dout;   // src/umh_fpga_top.v(81[12:26])
    wire [8:0]level_mem_dout;   // src/umh_fpga_top.v(82[12:26])
    
    wire mem_write_phase, mem_write_level;
    wire [31:0]phase_acc_next;   // src/umh_fpga_top.v(139[13:27])
    
    wire n9480, carrier_wrap;
    wire [15:0]fifo_depth;   // src/umh_fpga_top.v(158[13:23])
    
    wire n30, n160, fpga_clk_c_enable_344, n9477, n162, spi_version_7__N_1151, 
        spi_version_7__N_1120, spi_version_7__N_1145, spi_version_7__N_1116, 
        spi_version_7__N_1139, spi_version_7__N_1112, spi_frame_sequence_31__N_1356, 
        spi_frame_sequence_31__N_1451, spi_frame_sequence_31__N_1352, spi_frame_sequence_31__N_1346, 
        spi_frame_sequence_31__N_1410, spi_frame_sequence_31__N_1350, spi_command_7__N_1068, 
        spi_command_7__N_1091, spi_command_7__N_1064, spi_command_7__N_1060, 
        spi_command_7__N_1085, spi_version_7__N_1106, spi_version_7__N_1122, 
        spi_version_7__N_1110, spi_command_7__N_1103, spi_command_7__N_1072, 
        spi_command_7__N_1097, spi_rx_shift_6__N_916, spi_rx_shift_6__N_923, 
        spi_bit_count_2__N_956, spi_bit_count_2__N_946, spi_rx_shift_0__N_941, 
        spi_rx_shift_6__N_922, spi_rx_shift_2__N_935, spi_rx_shift_6__N_920, 
        spi_rx_shift_4__N_929, spi_rx_shift_6__N_918, spi_expected_length_31__N_1712, 
        spi_expected_length_31__N_1590, n17131, spi_expected_length_31__N_1706, 
        spi_expected_length_31__N_1586, n17141, spi_expected_length_31__N_1700, 
        spi_level_pending_7__N_1814, n17163, spi_expected_length_31__N_1724, 
        spi_expected_length_31__N_1598, spi_expected_length_31__N_1718, 
        spi_expected_length_31__N_1594, spi_channel_index_6__N_1740, spi_channel_index_6__N_1766, 
        spi_level_pending_7__N_1788, spi_level_pending_7__N_1800, spi_level_pending_7__N_1784, 
        spi_level_pending_7__N_1817, spi_channel_index_6__N_1732, spi_channel_index_6__N_1754, 
        spi_channel_index_6__N_1736, spi_channel_index_6__N_1760, rgb_values_95__N_655, 
        rgb_values_95__N_357, rgb_values_95__N_649, rgb_values_95__N_353, 
        rgb_values_95__N_643, rgb_values_95__N_673, rgb_values_95__N_369, 
        rgb_values_95__N_667, rgb_values_95__N_365, rgb_values_95__N_661, 
        rgb_values_95__N_361, rgb_values_95__N_381, rgb_values_95__N_685, 
        rgb_values_95__N_377, rgb_values_95__N_679, rgb_values_95__N_373, 
        rgb_values_95__N_703, rgb_values_95__N_389, rgb_values_95__N_697, 
        rgb_values_95__N_385, rgb_values_95__N_691, spi_extension_length_15__N_1254, 
        n9476, n17140, fpga_clk_c_enable_345, fpga_clk_c_enable_346, 
        n9473, spi_extension_length_15__N_1304, spi_extension_length_15__N_1256, 
        n17129, spi_extension_length_15__N_1307, spi_extension_length_15__N_1258, 
        spi_extension_length_15__N_1310, spi_extension_length_15__N_1260, 
        spi_update_flags_15__N_1247, spi_update_flags_15__N_1184, spi_update_flags_15__N_1244, 
        spi_update_flags_15__N_1182, n9472, n9469, n9468, n25_adj_3369, 
        n9465, n9261, n7629, n7626, n9464, n9461, n9460, spi_frame_sequence_31__N_1535, 
        us_tx_83__N_2854, spi_frame_sequence_31__N_1408, n9457, spi_frame_sequence_31__N_1532, 
        n9456, n8, spi_frame_sequence_31__N_1406, spi_frame_sequence_31__N_1529, 
        n9453, n9452, spi_frame_sequence_31__N_1404, spi_frame_sequence_31__N_1526, 
        n9449, spi_frame_sequence_31__N_1402, n9448, spi_frame_sequence_31__N_1523, 
        spi_frame_sequence_31__N_1400, n9445, spi_frame_sequence_31__N_1520, 
        n9444, spi_frame_sequence_31__N_1398, spi_frame_sequence_31__N_1517, 
        n9441, n9440, spi_frame_sequence_31__N_1396, spi_frame_sequence_31__N_1514, 
        n9437, spi_frame_sequence_31__N_1394, n9436, spi_frame_sequence_31__N_1511, 
        spi_frame_sequence_31__N_1392, n9433, spi_frame_sequence_31__N_1508, 
        n9432, n25_adj_3370, spi_frame_sequence_31__N_1390, spi_frame_sequence_31__N_1505, 
        n9429, n9428, spi_frame_sequence_31__N_1388, spi_frame_sequence_31__N_1502, 
        n9425, spi_frame_sequence_31__N_1386, n9424, spi_frame_sequence_31__N_1499, 
        n13789, spi_frame_sequence_31__N_1384, n9421, spi_frame_sequence_31__N_1496, 
        n9420, spi_frame_sequence_31__N_1382, spi_frame_sequence_31__N_1493, 
        n9417, n9416, spi_frame_sequence_31__N_1380, spi_frame_sequence_31__N_1490, 
        n9413, spi_frame_sequence_31__N_1378, n9412, spi_frame_sequence_31__N_1487, 
        n13787, spi_frame_sequence_31__N_1376, n9409, spi_frame_sequence_31__N_1484, 
        n9408, n29, spi_frame_sequence_31__N_1374, spi_frame_sequence_31__N_1481, 
        n9405, n9404, spi_frame_sequence_31__N_1372, spi_frame_sequence_31__N_1478, 
        n9401, spi_frame_sequence_31__N_1370, n9400, spi_frame_sequence_31__N_1475, 
        spi_frame_sequence_31__N_1368, n9397, spi_frame_sequence_31__N_1472, 
        n9396, spi_frame_sequence_31__N_1366, spi_frame_sequence_31__N_1469, 
        n9393, n9392, spi_frame_sequence_31__N_1364, spi_frame_sequence_31__N_1466, 
        n9389, spi_frame_sequence_31__N_1362, n9388, spi_frame_sequence_31__N_1463, 
        spi_frame_sequence_31__N_1360, n4, n9385, spi_frame_sequence_31__N_1460, 
        n9384, spi_frame_sequence_31__N_1358, spi_frame_sequence_31__N_1457, 
        n9381, n9380, spi_frame_sequence_31__N_1454, spi_frame_sequence_31__N_1354, 
        spi_frame_sequence_31__N_1448, spi_frame_sequence_31__N_1348, spi_frame_sequence_31__N_1445, 
        spi_version_7__N_1148, spi_version_7__N_1118, spi_version_7__N_1142, 
        spi_version_7__N_1114, spi_version_7__N_1136, spi_version_7__N_1108, 
        spi_version_7__N_1133, spi_command_7__N_1100, spi_command_7__N_1070, 
        spi_command_7__N_1094, spi_command_7__N_1066, spi_command_7__N_1088, 
        spi_command_7__N_1058, spi_command_7__N_1074, spi_command_7__N_1062, 
        n9909;
    wire [15:0]status_flags_wire_15__N_2034;
    
    wire n26_adj_3371, n12, n13921;
    wire [15:0]status_flags_wire_15__N_2050;
    
    wire n9377, n6814, spi_rx_shift_1__N_938, spi_rx_shift_6__N_921, 
        spi_rx_shift_3__N_932, spi_rx_shift_6__N_919, spi_rx_shift_5__N_926, 
        spi_rx_shift_6__N_917, spi_bit_count_2__N_959, spi_bit_count_2__N_948, 
        spi_bit_count_2__N_944, spi_bit_count_2__N_950;
    wire [7:0]spi1_miso_N_2860;
    
    wire n10, n9376, spi1_sck_c_enable_55, rgb_values_95__N_721, rgb_values_95__N_401, 
        rgb_values_95__N_718, rgb_values_95__N_399, rgb_values_95__N_715, 
        rgb_values_95__N_630, n9906, n25_adj_3372, n9905, n9260, spi1_sck_c_enable_56, 
        n9361, n29_adj_3373, fpga_clk_c_enable_274, fpga_clk_c_enable_377, 
        fpga_clk_c_enable_246, fpga_clk_c_enable_347, n40, fpga_clk_c_enable_407, 
        n9525, n9373, n8873, n9372, n12_adj_3374, n25_adj_3375, 
        fpga_clk_c_enable_441, n25_adj_3376, n9524, n9902, n9901, 
        spi1_sck_c_enable_57, n9898, n9897, spi1_sck_c_enable_58, n9894, 
        n9893, spi1_sck_c_enable_59, n9890, n9889, spi1_sck_c_enable_60, 
        n9886;
    wire [31:0]spi_expected_length_31__N_2085;
    
    wire n9253, n8872, n9521, fpga_clk_c_enable_371, n37, n9520, 
        n17162, n9264, n28, n9885, spi1_sck_c_enable_61, n9310, 
        n26_adj_3377, n12_adj_3378, fpga_clk_c_enable_410, n25_adj_3379, 
        n7, n8_adj_3380, n9308, n9305, n9297, n36, n38, n40_adj_3381, 
        n42, n9296, n9301, n7859, n22, n20, n18, n9288, n9300, 
        n5, n9517, n26_adj_3382, n9516, n159, fpga_clk_c_enable_403, 
        fpga_clk_c_enable_372, n17161, n9513, n56, n9512, n54, n52, 
        n9509, n18643, n12_adj_3383, n34, n35, n36_adj_3384, n37_adj_3385, 
        n62, n38_adj_3386, n39, n60, n40_adj_3387, n58, n17139, 
        n9508, n12_adj_3388, n17056, n17124, n17138, n16, n17160, 
        n17137, n17159, n9882, n9881, n17136, fpga_clk_c_enable_215, 
        spi1_sck_c_enable_62;
    wire [15:0]spi_byte_count_15__N_997;
    wire [2:0]spi_bit_count_2__N_953;
    wire [7:0]spi_version_7__N_1107;
    
    wire n9878, fpga_clk_c_enable_375, n7607, fpga_cs_n_c_enable_4;
    wire [15:0]spi_extension_length_15__N_1251;
    
    wire n21164, n17072, n17070, n17068, n17066, n17064, n17062;
    wire [6:0]spi_channel_index_6__N_1731;
    wire [7:0]spi_level_pending_7__N_1785;
    wire [1:0]spi_channel_field_1__N_1773;
    
    wire n8_adj_3389, fpga_cs_n_c_enable_34, rgb_values_95__N_521, rgb_values_95__N_898, 
        rgb_values_95__N_519, rgb_values_95__N_895, rgb_values_95__N_517, 
        rgb_values_95__N_892, rgb_values_95__N_515, rgb_values_95__N_889, 
        rgb_values_95__N_513, rgb_values_95__N_886, rgb_values_95__N_489, 
        rgb_values_95__N_850, rgb_values_95__N_487, rgb_values_95__N_847, 
        rgb_values_95__N_485, rgb_values_95__N_844, rgb_values_95__N_483, 
        rgb_values_95__N_841, rgb_values_95__N_481, rgb_values_95__N_838, 
        rgb_values_95__N_479, rgb_values_95__N_835, rgb_values_95__N_477, 
        rgb_values_95__N_832, rgb_values_95__N_475, rgb_values_95__N_829, 
        rgb_values_95__N_473, rgb_values_95__N_826, rgb_values_95__N_471, 
        rgb_values_95__N_823, rgb_values_95__N_469, rgb_values_95__N_511, 
        rgb_values_95__N_883, rgb_values_95__N_509, rgb_values_95__N_880, 
        rgb_values_95__N_507, rgb_values_95__N_877, rgb_values_95__N_505, 
        rgb_values_95__N_874, rgb_values_95__N_503, rgb_values_95__N_871, 
        n17135, rgb_values_95__N_501, rgb_values_95__N_868, rgb_values_95__N_499, 
        rgb_values_95__N_865, rgb_values_95__N_497, rgb_values_95__N_862, 
        n17157, rgb_values_95__N_495, rgb_values_95__N_859, n17134, 
        rgb_values_95__N_493, rgb_values_95__N_856, rgb_values_95__N_491, 
        rgb_values_95__N_853, rgb_values_95__N_820, n17156, rgb_values_95__N_467, 
        rgb_values_95__N_817, rgb_values_95__N_465, rgb_values_95__N_814, 
        rgb_values_95__N_463, rgb_values_95__N_811, rgb_values_95__N_461, 
        rgb_values_95__N_808, rgb_values_95__N_459, rgb_values_95__N_805, 
        rgb_values_95__N_457, rgb_values_95__N_802, rgb_values_95__N_455, 
        rgb_values_95__N_799, rgb_values_95__N_453, rgb_values_95__N_796, 
        rgb_values_95__N_451, rgb_values_95__N_793, rgb_values_95__N_449, 
        rgb_values_95__N_790, n17125, rgb_values_95__N_447, n17155, 
        rgb_values_95__N_784, rgb_values_95__N_443, rgb_values_95__N_778, 
        rgb_values_95__N_439, rgb_values_95__N_772, rgb_values_95__N_435, 
        n17086, rgb_values_95__N_766, rgb_values_95__N_431, rgb_values_95__N_760, 
        rgb_values_95__N_427, rgb_values_95__N_437, spi_byte_count_15__N_962, 
        spi_byte_count_15__N_994, spi_byte_count_15__N_964, spi_byte_count_15__N_1013, 
        spi_byte_count_15__N_966, spi_byte_count_15__N_1016, spi_byte_count_15__N_968, 
        spi_byte_count_15__N_1019, spi_byte_count_15__N_970, spi_byte_count_15__N_1022, 
        spi_byte_count_15__N_972, spi_byte_count_15__N_1025, spi_byte_count_15__N_974, 
        spi_byte_count_15__N_1028, spi_byte_count_15__N_976, spi_byte_count_15__N_1031, 
        spi_byte_count_15__N_978, spi_byte_count_15__N_1034, spi_byte_count_15__N_980, 
        spi_byte_count_15__N_1037, spi_byte_count_15__N_982, spi_byte_count_15__N_1040, 
        spi_byte_count_15__N_984, spi_byte_count_15__N_1043, spi_byte_count_15__N_986, 
        spi_byte_count_15__N_1046, spi_byte_count_15__N_988, spi_byte_count_15__N_1049, 
        spi_byte_count_15__N_990, spi_byte_count_15__N_1052, spi_byte_count_15__N_992, 
        spi_byte_count_15__N_1055, rgb_values_95__N_913, rgb_values_95__N_529, 
        rgb_values_95__N_910, rgb_values_95__N_527, rgb_values_95__N_907, 
        rgb_values_95__N_525, rgb_values_95__N_904, rgb_values_95__N_523, 
        rgb_values_95__N_901, accepted_sequence_spi_31__N_2262, n9877, 
        fpga_clk_c_enable_378, frame_toggle_spi_N_2870, stop_toggle_spi_N_2878, 
        n9292, n9325, stop_toggle_spi_N_2877, spi1_sck_c_enable_63, 
        frame_toggle_spi_N_2869, n50, n7832, n26_adj_3390, fpga_clk_c_enable_374, 
        n9304, n9317, n9874, fpga_clk_c_enable_411, n9873, invalid_frame_spi_N_2881, 
        rgb_values_95__N_724, rgb_values_95__N_425, rgb_values_95__N_754, 
        rgb_values_95__N_423, rgb_values_95__N_751, rgb_values_95__N_421, 
        rgb_values_95__N_748, rgb_values_95__N_419, rgb_values_95__N_745, 
        fpga_cs_n_c_enable_1, rgb_values_95__N_417, rgb_values_95__N_742, 
        n15, rgb_values_95__N_415, rgb_values_95__N_739, n18642, rgb_values_95__N_413, 
        rgb_values_95__N_736, n11, rgb_values_95__N_411, rgb_values_95__N_733, 
        rgb_values_95__N_409, rgb_values_95__N_730, n18531, rgb_values_95__N_407, 
        rgb_values_95__N_727, rgb_values_95__N_405, rgb_values_95__N_403, 
        n6;
    wire [5:0]time_divider_5__N_2270;
    
    wire fpga_clk_c_enable_379, time_divider_5__N_2269, n8565, n165, 
        n14, n10_adj_3391, n128, n150, n151, n123, n17055, n10_adj_3392, 
        n154, n153, n152, n38_adj_3393, n37_adj_3394, n9289, n155, 
        spi1_sck_c_enable_64, time_half_N_2907, n9870, n7_adj_3395, 
        n9285, n17074, n9281, n9869, n9284, spi1_sck_c_enable_65, 
        n36_adj_3396, n34_adj_3397, n9866, n9865, frame_toggle_seen_N_2889, 
        n9365, n9364, n9369, spi1_sck_c_enable_66, stop_toggle_seen_N_2891, 
        n9324, n25_adj_3398, n17054, n17126, n17059, n17127, n9862, 
        n9861, spi1_sck_c_enable_67, n9858, n9857, spi1_sck_c_enable_68, 
        n9854, n9853, spi1_sck_c_enable_69, n9850, n9849, spi1_sck_c_enable_70, 
        n9846, n9845, spi1_sck_c_enable_71, n9842, n9841, spi1_sck_c_enable_72, 
        n9838, n9837, spi1_sck_c_enable_73, n9834, n9833, spi1_sck_c_enable_74, 
        n9830, n9829, spi1_sck_c_enable_75, n9826, n9825, spi1_sck_c_enable_76, 
        n9822, n9821, spi1_sck_c_enable_77, n9817, n9816, n9813, 
        n9812, n9809, n9808, n9805, n9804, n9801, n9800, n9797, 
        n9796, n9793, n9792, n9789, n9788, n9785, n9784, n9781, 
        n9780, n2315, n2316, n9777, n9776, n9773, n9772, n9769, 
        n9768, n9765, n9764, n9761, n9760, n9757, n9756, n9753, 
        n9752, n9749, n9748, n9745, n9744, n9741, n9740, n9737, 
        n9736, n9733, n9732, n9729, n9728, n9725, n9724, n9721, 
        n9720, n9717, n9716, n9713, n9712, n9709, n9708, n9705, 
        n9704, n9701, n9700, n9697, n9696, n17058, n9693, n9692, 
        n9689, n9688, spi1_sck_c_enable_78, n9685, n9684, spi1_sck_c_enable_79, 
        n9681, n9680, spi1_sck_c_enable_80, n9677, n9676, spi1_sck_c_enable_81, 
        n9673, n9672, spi1_sck_c_enable_82, n14028, n9669, n9668, 
        spi1_sck_c_enable_83, n9665, n9664, spi1_sck_c_enable_84, n13994, 
        n9661, n9660, spi1_sck_c_enable_85, n9657, n9656, spi1_sck_c_enable_86, 
        n9653, n9652, spi1_sck_c_enable_87, n9649, n9648, spi1_sck_c_enable_88, 
        n9645, n9644, spi1_sck_c_enable_89, n9641, n9640, spi1_sck_c_enable_90, 
        n9637, n9636, spi1_sck_c_enable_91, n9633, n9632, spi1_sck_c_enable_92, 
        n9629, n9628, spi1_sck_c_enable_93, n9625, n9624, spi1_sck_c_enable_94, 
        n9621, n9620, spi1_sck_c_enable_95, n9617, n9616, spi1_sck_c_enable_96, 
        n9613, n9612, n14_adj_3399, n37_adj_3400, n13820, n17057, 
        n62_adj_3401, fpga_clk_c_enable_405, n12_adj_3402, n17128, n134, 
        n135, n9609, n136, n9608, n12_adj_3403, n11180, n137, 
        n9505, n11181, fpga_clk_c_enable_348, n9504, n138, n9501, 
        n9500, n139, n17130, n17100, n26_adj_3404, n17213, n27, 
        n28_adj_3405, n29_adj_3406, n140, fpga_clk_c_enable_380, n10198, 
        fpga_clk_c_enable_349, n10197, n17212, n141, n12_adj_3407, 
        n17211, n3, n4_adj_3408, n5_adj_3409, n6_adj_3410, n7_adj_3411, 
        n8_adj_3412, spi1_sck_c_enable_121, n142, n9277, n17210, n14030, 
        n26_adj_3413, n10194, n9280, n10193, n17209, spi1_sck_c_enable_175, 
        n9293, n17208, n9276, n17207, n10190, fpga_clk_c_enable_381, 
        n9605, n9604, n17441, fpga_clk_c_enable_406, n10189, spi1_sck_c_enable_176, 
        n17206, n143, fpga_clk_c_enable_350, n10186, fpga_clk_c_enable_382, 
        n17205, n17204, fpga_clk_c_enable_351, running_N_2903, mic_divider_6__N_2705, 
        fpga_clk_c_enable_383, n10185, n10305, n9273, n9272, spi1_sck_c_enable_177, 
        n30_adj_3414, n32, n34_adj_3415, n17203, n48, n8610, n9488, 
        n17202, n10182, n17201, n9497, n10181, n58_adj_3416, n60_adj_3417, 
        n9496, spi1_sck_c_enable_178, n9601, n9600, n13987, n9597, 
        n9596, n9593, n9592, n9589, n9588, n9585, n157, n163, 
        fpga_clk_c_enable_408, n7113, fpga_clk_c_enable_373, n9268, 
        mic_clk_N_2867, n10178, n17200, n144, fpga_clk_c_enable_352, 
        n10177, spi1_sck_c_enable_179, n17199, fpga_clk_c_enable_384, 
        n9584, n145, n9265, n12_adj_3418, n38_adj_3419, n39_adj_3420, 
        n40_adj_3421, n17198, n41, n42_adj_3422, n43, n44, n45, 
        n146, rgb_values_95__N_349, rgb_values_95__N_640, rgb_values_95__N_347, 
        rgb_values_95__N_637, rgb_values_95__N_345, rgb_values_95__N_634, 
        rgb_values_95__N_343, rgb_values_95__N_631, rgb_values_95__N_341, 
        rgb_values_95__N_531, spi_channel_field_1__N_1772, spi_channel_field_1__N_1776, 
        spi_channel_field_1__N_1774, spi_channel_field_1__N_1781, rgb_values_95__N_337, 
        spi_level_pending_7__N_1829, spi_level_pending_7__N_1798, spi_level_pending_7__N_1826, 
        spi_level_pending_7__N_1796, spi_level_pending_7__N_1823, spi_level_pending_7__N_1794, 
        spi_expected_length_31__N_1582, spi_expected_length_31__N_1697, 
        spi_expected_length_31__N_1580, spi_expected_length_31__N_1694, 
        spi_level_pending_7__N_1820, spi_level_pending_7__N_1792, spi_channel_index_6__N_1730, 
        spi_channel_index_6__N_1744, spi_channel_index_6__N_1734, spi_channel_index_6__N_1757, 
        spi_channel_index_6__N_1738, spi_channel_index_6__N_1763, spi_channel_index_6__N_1742, 
        spi_channel_index_6__N_1769, spi_level_pending_7__N_1811, spi_level_pending_7__N_1786, 
        spi_level_pending_7__N_1790, spi_expected_length_31__N_1727, spi_expected_length_31__N_1600, 
        spi_expected_length_31__N_1721, spi_expected_length_31__N_1596, 
        spi_expected_length_31__N_1715, spi_expected_length_31__N_1592, 
        spi_expected_length_31__N_1709, spi_expected_length_31__N_1588, 
        spi_expected_length_31__N_1703, spi_expected_length_31__N_1584, 
        rgb_values_95__N_397, rgb_values_95__N_395, rgb_values_95__N_712, 
        spi_extension_length_15__N_1301, spi_extension_length_15__N_1266, 
        spi_extension_length_15__N_1316, spi_extension_length_15__N_1262, 
        spi_extension_length_15__N_1252, rgb_values_95__N_393, rgb_values_95__N_709, 
        spi_expected_length_31__N_1578, spi_expected_length_31__N_1691, 
        spi_expected_length_31__N_1576, spi_expected_length_31__N_1688, 
        spi_expected_length_31__N_1574, spi_expected_length_31__N_1685, 
        rgb_values_95__N_706, rgb_values_95__N_391, rgb_values_95__N_700, 
        rgb_values_95__N_387, rgb_values_95__N_694, rgb_values_95__N_383, 
        rgb_values_95__N_688, rgb_values_95__N_379, rgb_values_95__N_682, 
        rgb_values_95__N_375, rgb_values_95__N_676, rgb_values_95__N_371, 
        rgb_values_95__N_670, rgb_values_95__N_367, rgb_values_95__N_664, 
        rgb_values_95__N_363, rgb_values_95__N_658, rgb_values_95__N_359, 
        rgb_values_95__N_652, rgb_values_95__N_355, rgb_values_95__N_646, 
        rgb_values_95__N_351, spi_expected_length_31__N_1572, spi_expected_length_31__N_1682, 
        n18792, spi_expected_length_31__N_1570, spi_expected_length_31__N_1679, 
        spi_expected_length_31__N_1568, spi_expected_length_31__N_1676, 
        spi_expected_length_31__N_1566, spi_expected_length_31__N_1673, 
        spi_expected_length_31__N_1564, spi_expected_length_31__N_1670, 
        spi_expected_length_31__N_1562, spi_expected_length_31__N_1667, 
        spi_expected_length_31__N_1560, spi_expected_length_31__N_1664, 
        spi_expected_length_31__N_1558, spi_expected_length_31__N_1661, 
        spi_expected_length_31__N_1556, spi_expected_length_31__N_1658, 
        spi_expected_length_31__N_1554, spi_expected_length_31__N_1655, 
        spi_expected_length_31__N_1552, spi_expected_length_31__N_1652, 
        spi_expected_length_31__N_1550, spi_expected_length_31__N_1649, 
        spi_expected_length_31__N_1538, spi_expected_length_31__N_1602, 
        spi_expected_length_31__N_1540, spi_expected_length_31__N_1637, 
        spi_expected_length_31__N_1542, spi_expected_length_31__N_1640, 
        spi_expected_length_31__N_1544, spi_expected_length_31__N_1643, 
        spi_expected_length_31__N_1546, spi_expected_length_31__N_1646, 
        spi_expected_length_31__N_1548, spi_extension_length_15__N_1343, 
        spi_extension_length_15__N_1280, spi_extension_length_15__N_1340, 
        spi_extension_length_15__N_1278, spi_extension_length_15__N_1337, 
        spi_extension_length_15__N_1276, spi_extension_length_15__N_1334, 
        spi_extension_length_15__N_1274, spi_extension_length_15__N_1331, 
        spi_extension_length_15__N_1272, spi_extension_length_15__N_1328, 
        spi_extension_length_15__N_1270, spi_extension_length_15__N_1325, 
        spi_extension_length_15__N_1268, spi_extension_length_15__N_1322;
    wire [12:0]reset_count;   // src/ws2812_stream.v(14[12:23])
    
    wire n9581, n9580, n9577, n9316, n9313, n9312, n9576, n9573, 
        n9572, n24, fpga_clk_c_enable_412, n9569, n9568, n18524, 
        n9565, n9564, n9561, n9560, n9557, n9556, fpga_clk_c_enable_413, 
        fpga_clk_c_enable_414, fpga_clk_c_enable_415, n9322, spi1_sck_c_enable_191, 
        n17196, n10174, fpga_clk_c_enable_333, n10173, n164, fpga_clk_c_enable_334, 
        n17195, spi1_sck_c_enable_180, n9493, n12_adj_3423, n9492, 
        n10170, n10169, fpga_clk_c_enable_335, n66, fpga_clk_c_enable_23, 
        n40_adj_3424, n39_adj_3425, n44_adj_3426, n17194, n38_adj_3427, 
        n37_adj_3428, n36_adj_3429, fpga_clk_c_enable_336, fpga_clk_c_enable_337, 
        n39_adj_3430, n36_adj_3431, fpga_clk_c_enable_338, n9553, n9552, 
        n9549, n9548, spi1_sck_c_enable_97, n9545, n9544, n9541, 
        n9540, spi1_sck_c_enable_98, spi1_sck_c_enable_181, n35_adj_3432, 
        n12_adj_3433, fpga_clk_c_enable_339, fpga_clk_c_enable_340, n6931, 
        n13, fpga_clk_c_enable_341, fpga_clk_c_enable_342, fpga_clk_c_enable_404, 
        n34_adj_3434, n9537, n9536, n7008, n7007, spi1_sck_c_enable_99, 
        spi1_sck_c_enable_100, n7166, n46, n10166, n10165, spi1_sck_c_enable_182, 
        spi1_sck_c_enable_101, n10162, n10161, spi1_sck_c_enable_102, 
        n9256, n9257, n9320, spi1_sck_c_enable_103, n9357, spi1_sck_c_enable_104, 
        n9360, n13957, spi1_sck_c_enable_183, spi1_sck_c_enable_105, 
        n8652, fpga_clk_c_enable_353, fpga_clk_c_enable_385, n161, n10158, 
        n147, n8_adj_3435, n10157, spi1_sck_c_enable_184, spi1_sck_c_enable_106, 
        n10154, n10153, spi1_sck_c_enable_185, fpga_clk_c_enable_354, 
        n148, n17188, spi1_sck_c_enable_107, fpga_clk_c_enable_386, 
        n10150, n8_adj_3436, n10149, spi1_sck_c_enable_186, fpga_clk_c_enable_355, 
        n149, n8_adj_3437, spi1_sck_c_enable_108, n10146, n10145, 
        n9489, n12_adj_3438, n156, fpga_clk_c_enable_387, n8632, n9269, 
        n17187, n9003, n9368, spi1_sck_c_enable_109, n9328, n9329, 
        n9332, n9333, n9336, n9337, spi1_sck_c_enable_110, n9340, 
        n9341, n9344, n9345, n9348, n9349, spi1_sck_c_enable_111, 
        n9352, n9353, spi1_sck_c_enable_112, n9356, n17186, n35_adj_3439, 
        spi1_sck_c_enable_187, n32_adj_3440, n33, n34_adj_3441, n35_adj_3442, 
        spi1_sck_c_enable_113, fpga_clk_c_enable_356, n10142, n9533, 
        fpga_clk_c_enable_188, n10141, spi1_sck_c_enable_188, spi1_sck_c_enable_114, 
        n10138, n17185, n10137, spi1_sck_c_enable_189, spi1_sck_c_enable_115, 
        n10134, n10133, spi1_sck_c_enable_190, spi1_sck_c_enable_116, 
        n10130, n10129, spi1_sck_c_enable_192, spi1_sck_c_enable_117, 
        n10126, n10125, spi1_sck_c_enable_1, spi1_sck_c_enable_118, 
        n10122, n10121, spi1_sck_c_enable_2, spi1_sck_c_enable_119, 
        n10118, n17184, n10117, spi1_sck_c_enable_3, spi1_sck_c_enable_120, 
        n10114, n10113, spi1_sck_c_enable_4, spi1_sck_c_enable_122, 
        n10110, n10109, spi1_sck_c_enable_5, spi1_sck_c_enable_123, 
        n10106, n10105, spi1_sck_c_enable_6, spi1_sck_c_enable_124, 
        n10102, n10101, spi1_sck_c_enable_7, spi1_sck_c_enable_125, 
        n10098, n10097, spi1_sck_c_enable_8, spi1_sck_c_enable_126, 
        n10094, n10093, spi1_sck_c_enable_9, spi1_sck_c_enable_127, 
        n10090, n10089, spi1_sck_c_enable_10, spi1_sck_c_enable_128, 
        n10086, n10085, spi1_sck_c_enable_11, spi1_sck_c_enable_129, 
        n10082, n10081, spi1_sck_c_enable_12, spi1_sck_c_enable_130, 
        n10078, n10077, spi1_sck_c_enable_13, n8870, spi1_sck_c_enable_131, 
        n10074, n10073, spi1_sck_c_enable_14, spi1_sck_c_enable_132, 
        spi1_sck_c_enable_133, n10070, n9254, n10069, spi1_sck_c_enable_15, 
        n9019, spi1_sck_c_enable_134, n10066, n10065, spi1_sck_c_enable_16, 
        spi1_sck_c_enable_135, n10062, n9009, n10061, spi1_sck_c_enable_17, 
        spi1_sck_c_enable_136, n10058, n18527, n10057, spi1_sck_c_enable_18, 
        spi1_sck_c_enable_137, n10054, n10053, spi1_sck_c_enable_19, 
        spi1_sck_c_enable_138, n10050, n10049, spi1_sck_c_enable_20, 
        spi1_sck_c_enable_139, n10046, n10045, spi1_sck_c_enable_21, 
        spi1_sck_c_enable_140, n10042, n10041, spi1_sck_c_enable_22, 
        spi1_sck_c_enable_141, n10038, n10037, spi1_sck_c_enable_23, 
        spi1_sck_c_enable_142, n10034, n10033, spi1_sck_c_enable_24, 
        spi1_sck_c_enable_143, n10030, n10029, spi1_sck_c_enable_25, 
        spi1_sck_c_enable_144, n10026, n10025, spi1_sck_c_enable_26, 
        spi1_sck_c_enable_145, n10022, n10021, spi1_sck_c_enable_27, 
        spi1_sck_c_enable_146, n17183, n10018, n10017, spi1_sck_c_enable_28, 
        spi1_sck_c_enable_147, n10014, n10013, spi1_sck_c_enable_29, 
        spi1_sck_c_enable_148, n10010, n10009, spi1_sck_c_enable_30, 
        spi1_sck_c_enable_149, n10006, n10005, spi1_sck_c_enable_31, 
        spi1_sck_c_enable_150, n10002, n10001, n4_adj_3443, spi1_sck_c_enable_32, 
        spi1_sck_c_enable_151, n9998, n9997, spi1_sck_c_enable_33, spi1_sck_c_enable_152, 
        n9994, n9993, spi1_sck_c_enable_34, spi1_sck_c_enable_153, n9990, 
        n9989, spi1_sck_c_enable_35, spi1_sck_c_enable_154, n9986, n9985, 
        spi1_sck_c_enable_36, spi1_sck_c_enable_155, n9982, n9981, spi1_sck_c_enable_37, 
        n17182, spi1_sck_c_enable_156, n9978, n9977, spi1_sck_c_enable_38, 
        spi1_sck_c_enable_157, n9974, n9973, spi1_sck_c_enable_39, spi1_sck_c_enable_158, 
        n9970, n9969, spi1_sck_c_enable_40, spi1_sck_c_enable_159, n9966, 
        n9965, spi1_sck_c_enable_41, spi1_sck_c_enable_160, n9962, n9961, 
        spi1_sck_c_enable_42, spi1_sck_c_enable_161, n17180, n9958, 
        n9957, spi1_sck_c_enable_43, spi1_sck_c_enable_162, n9954, n9953, 
        spi1_sck_c_enable_44, spi1_sck_c_enable_163, n9950, n9949, n8881, 
        spi1_sck_c_enable_45, spi1_sck_c_enable_164, n9946, n9945, spi1_sck_c_enable_46, 
        fpga_clk_c_enable_409, spi1_sck_c_enable_165, n9942, n9941, 
        spi1_sck_c_enable_47, spi1_sck_c_enable_166, n9938, n9937, spi1_sck_c_enable_48, 
        spi1_sck_c_enable_167, n9934, fpga_clk_c_enable_444, n9933, 
        fpga_clk_c_enable_388, fpga_clk_c_enable_357, spi1_sck_c_enable_49, 
        fpga_clk_c_enable_389, fpga_clk_c_enable_358, spi1_sck_c_enable_168, 
        fpga_clk_c_enable_390, fpga_clk_c_enable_359, n9930, fpga_clk_c_enable_391, 
        fpga_clk_c_enable_360, n9929, fpga_clk_c_enable_392, fpga_clk_c_enable_361, 
        spi1_sck_c_enable_50, fpga_clk_c_enable_393, fpga_clk_c_enable_362, 
        spi1_sck_c_enable_169, fpga_clk_c_enable_394, fpga_clk_c_enable_363, 
        n9926, fpga_clk_c_enable_395, fpga_clk_c_enable_364, n9925, 
        n9485, spi1_sck_c_enable_51, n9532, spi1_sck_c_enable_170, spi1_sck_c_enable_171, 
        n9922, n9921, fpga_clk_c_enable_396, fpga_clk_c_enable_365, 
        spi1_sck_c_enable_52, fpga_clk_c_enable_397, fpga_clk_c_enable_366, 
        spi1_sck_c_enable_172, n8892, n9918, fpga_clk_c_enable_398, 
        fpga_clk_c_enable_367, n9917, fpga_clk_c_enable_399, fpga_clk_c_enable_368, 
        spi1_sck_c_enable_53, fpga_clk_c_enable_400, fpga_clk_c_enable_369, 
        spi1_sck_c_enable_173, n9914, fpga_clk_c_enable_370, fpga_clk_c_enable_401, 
        n9913, fpga_clk_c_enable_402, fpga_clk_c_enable_343, spi1_sck_c_enable_54, 
        n17179, spi1_sck_c_enable_174, n9910, n17178, n25_adj_3444, 
        n26_adj_3445, n26_adj_3446, n25_adj_3447, n26_adj_3448, n17177, 
        n25_adj_3449, n26_adj_3450, n25_adj_3451, n25_adj_3452, n26_adj_3453, 
        n26_adj_3454, n17176, n25_adj_3455, n25_adj_3456, n26_adj_3457, 
        n26_adj_3458, n17175, n25_adj_3459, n25_adj_3460, n26_adj_3461, 
        n26_adj_3462, n25_adj_3463, n25_adj_3464, n26_adj_3465, n26_adj_3466, 
        n18888, n18548, n25_adj_3467, n25_adj_3468, n26_adj_3469, 
        n17173, n26_adj_3470, n25_adj_3471, n25_adj_3472, n26_adj_3473, 
        n26_adj_3474, n17172, n25_adj_3475, n25_adj_3476, n26_adj_3477, 
        n26_adj_3478, n17171, n25_adj_3479, n25_adj_3480, n26_adj_3481, 
        n26_adj_3482, n25_adj_3483, n17170, n25_adj_3484, n26_adj_3485, 
        n26_adj_3486, n25_adj_3487, n25_adj_3488, n26_adj_3489, n17169, 
        n26_adj_3490, n25_adj_3491, n25_adj_3492, n26_adj_3493, n17168, 
        n25_adj_3494, n26_adj_3495, n26_adj_3496, n25_adj_3497, n26_adj_3498, 
        n17167, n25_adj_3499, n26_adj_3500, n25_adj_3501, n26_adj_3502, 
        n25_adj_3503, n26_adj_3504, n17166, n25_adj_3505, n26_adj_3506, 
        n26_adj_3507, n17165, n26_adj_3508, n25_adj_3509, n26_adj_3510, 
        n26_adj_3511, n17436, n17164, n10_adj_3512, n18641, n18634, 
        n18846, n18844, n18842, n18838, n18834, n18818, n18802, 
        n20623, n18782, n20622, n20621, n20868, n20867, n20866, 
        n20865, n20620, n18762, n18752, n21165, n10_adj_3513, n20785, 
        n20784, n14_adj_3514, n20783, n10_adj_3515, n6_adj_3516, n8_adj_3517, 
        n20744;
    
    VHI i2 (.Z(VCC_net));
    INV i16393 (.A(spi_mic_sck_c), .Z(sck_N_3318));   // src/umh_fpga_top.v(17[24:35])
    FD1P3AX level_active_83___i159 (.D(n2316), .SP(fpga_clk_c_enable_411), 
            .CK(fpga_clk_c), .Q(\level_active[4] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i159.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i12_5349_5350_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1384), .Q(n9561)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i12_5349_5350_set.GSR = "ENABLED";
    LUT4 i15692_2_lut_4_lut (.A(rgb_values[30]), .B(rgb_values[22]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_823)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15692_2_lut_4_lut.init = 16'h0035;
    FD1P3AX level_active_83___i89 (.D(n2316), .SP(fpga_clk_c_enable_444), 
            .CK(fpga_clk_c), .Q(\level_active[39] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i89.GSR = "ENABLED";
    LUT4 i9720_2_lut (.A(n9785), .B(n9784), .Z(spi_byte_count[7])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9720_2_lut.init = 16'h8888;
    FD1P3AX level_active_83___i158 (.D(n2315), .SP(fpga_clk_c_enable_410), 
            .CK(fpga_clk_c), .Q(\level_active[5] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i158.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1948_2_lut_3_lut (.A(spi_expected_length[25]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1550)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1948_2_lut_3_lut.init = 16'h0808;
    LUT4 i9718_2_lut (.A(n9793), .B(n9792), .Z(spi_byte_count[9])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9718_2_lut.init = 16'h8888;
    CCU2D add_2849_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(spi_channel_index[0]), .B1(n18531), .C1(n5), .D1(n8873), 
          .COUT(n17155), .S1(spi_channel_index_6__N_1731[0]));   // src/umh_fpga_top.v(188[13] 233[20])
    defparam add_2849_1.INIT0 = 16'hF000;
    defparam add_2849_1.INIT1 = 16'h56aa;
    defparam add_2849_1.INJECT1_0 = "NO";
    defparam add_2849_1.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1602_2_lut_4_lut (.A(rgb_values[30]), .B(rgb_values[22]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_469)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1602_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9802_3_lut (.A(status_bit_index[2]), .B(status_bit_index[0]), 
         .C(status_bit_index[1]), .Z(n13921)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i9802_3_lut.init = 16'h8080;
    LUT4 i9715_2_lut (.A(n9805), .B(n9804), .Z(spi_byte_count[12])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9715_2_lut.init = 16'h8888;
    LUT4 i15695_2_lut_4_lut (.A(rgb_values[29]), .B(rgb_values[21]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_826)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15695_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1603_2_lut_4_lut (.A(rgb_values[29]), .B(rgb_values[21]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_471)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1603_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15698_2_lut_4_lut (.A(rgb_values[28]), .B(rgb_values[20]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_829)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15698_2_lut_4_lut.init = 16'h0035;
    LUT4 i9884_2_lut (.A(n9809), .B(n9808), .Z(spi_byte_count[13])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9884_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1604_2_lut_4_lut (.A(rgb_values[28]), .B(rgb_values[20]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_473)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1604_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15701_2_lut_4_lut (.A(rgb_values[27]), .B(rgb_values[19]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_832)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15701_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1847_2_lut (.A(fpga_cs_n_c), .B(spi_extension_length_15__N_1251[14]), 
         .Z(spi_extension_length_15__N_1252)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1847_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1605_2_lut_4_lut (.A(rgb_values[27]), .B(rgb_values[19]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_475)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1605_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9716_2_lut (.A(n9801), .B(n9800), .Z(spi_byte_count[11])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9716_2_lut.init = 16'h8888;
    LUT4 i15305_2_lut (.A(fpga_cs_n_c), .B(spi_extension_length_15__N_1251[14]), 
         .Z(spi_extension_length_15__N_1301)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15305_2_lut.init = 16'h1111;
    LUT4 i15704_2_lut_4_lut (.A(rgb_values[26]), .B(rgb_values[18]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_835)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15704_2_lut_4_lut.init = 16'h0035;
    LUT4 i9717_2_lut (.A(n9797), .B(n9796), .Z(spi_byte_count[10])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9717_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1749_2_lut_3_lut (.A(n9257), .B(n9256), .C(fpga_cs_n_c), 
         .Z(spi_bit_count_2__N_959)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1749_2_lut_3_lut.init = 16'h0808;
    LUT4 fpga_cs_n_N_339_I_0_1606_2_lut_4_lut (.A(rgb_values[26]), .B(rgb_values[18]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_477)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1606_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1846_2_lut (.A(fpga_cs_n_c), .B(spi_extension_length_15__N_1251[15]), 
         .Z(spi_extension_length_15__N_1250)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1846_2_lut.init = 16'h4444;
    LUT4 i15707_2_lut_4_lut (.A(rgb_values[25]), .B(rgb_values[17]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_838)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15707_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1607_2_lut_4_lut (.A(rgb_values[25]), .B(rgb_values[17]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_479)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1607_2_lut_4_lut.init = 16'h00ca;
    LUT4 i3_3_lut (.A(reset_count[5]), .B(n6_adj_3516), .C(reset_count[4]), 
         .Z(fpga_clk_c_enable_215)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i3_3_lut.init = 16'h8080;
    LUT4 i2_4_lut (.A(reset_count[1]), .B(reset_count[2]), .C(reset_count[0]), 
         .D(n17441), .Z(n6_adj_3516)) /* synthesis lut_function=(!(A ((C+(D))+!B)+!A (((D)+!C)+!B))) */ ;
    defparam i2_4_lut.init = 16'h0048;
    LUT4 i15302_2_lut (.A(fpga_cs_n_c), .B(spi_extension_length_15__N_1251[15]), 
         .Z(spi_extension_length_15__N_1282)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15302_2_lut.init = 16'h1111;
    LUT4 i15710_2_lut_4_lut (.A(rgb_values[24]), .B(rgb_values[16]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_841)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15710_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1608_2_lut_4_lut (.A(rgb_values[24]), .B(rgb_values[16]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_481)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1608_2_lut_4_lut.init = 16'h00ca;
    LUT4 i30_3_lut_4_lut (.A(amplitude_phase[6]), .B(\level_active[25] [0]), 
         .C(\level_active[25] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3468)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_22 (.A(amplitude_phase[6]), .B(\level_active[82] [0]), 
         .C(\level_active[82] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3508)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_22.init = 16'h40f4;
    LUT4 i15713_2_lut_4_lut (.A(rgb_values[23]), .B(rgb_values[15]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_844)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15713_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_2011_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[1]), 
         .Z(spi_channel_index_6__N_1740)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2011_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1609_2_lut_4_lut (.A(rgb_values[23]), .B(rgb_values[15]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_483)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1609_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15461_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[1]), 
         .Z(spi_channel_index_6__N_1766)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15461_2_lut.init = 16'h1111;
    LUT4 i15716_2_lut_4_lut (.A(rgb_values[22]), .B(rgb_values[14]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_847)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15716_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_2010_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[2]), 
         .Z(spi_channel_index_6__N_1738)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2010_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1610_2_lut_4_lut (.A(rgb_values[22]), .B(rgb_values[14]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_485)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1610_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15458_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[2]), 
         .Z(spi_channel_index_6__N_1763)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15458_2_lut.init = 16'h1111;
    LUT4 i3662_3_lut_4_lut (.A(n9701), .B(n9700), .C(n7859), .D(spi_rx_shift[6]), 
         .Z(spi_extension_length_15__N_1251[15])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A (C (D))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i3662_3_lut_4_lut.init = 16'hf808;
    LUT4 i3660_3_lut_4_lut (.A(n9697), .B(n9696), .C(n7859), .D(spi_rx_shift[5]), 
         .Z(spi_extension_length_15__N_1251[14])) /* synthesis lut_function=(A (B ((D)+!C)+!B (C (D)))+!A (C (D))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i3660_3_lut_4_lut.init = 16'hf808;
    LUT4 i14672_2_lut_3_lut_4_lut (.A(n9697), .B(n9696), .C(n9700), .D(n9701), 
         .Z(n18792)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i14672_2_lut_3_lut_4_lut.init = 16'hf888;
    LUT4 fpga_cs_n_N_339_I_0_2009_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[3]), 
         .Z(spi_channel_index_6__N_1736)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2009_2_lut.init = 16'h4444;
    OB us_tx_pad_77 (.I(us_tx_c_77), .O(us_tx[77]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX level_active_83___i157 (.D(n2316), .SP(fpga_clk_c_enable_410), 
            .CK(fpga_clk_c), .Q(\level_active[5] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i157.GSR = "ENABLED";
    FD1P3AX level_active_83___i88 (.D(n2315), .SP(fpga_clk_c_enable_375), 
            .CK(fpga_clk_c), .Q(\level_active[40] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i88.GSR = "ENABLED";
    PFUMX i16128 (.BLUT(n20621), .ALUT(n20620), .C0(spi_byte_count[5]), 
          .Z(n20622));
    DP8KC phase_mem_i (.DIA0(spi1_mosi_c), .DIA1(spi_rx_shift[0]), .DIA2(spi_rx_shift[1]), 
          .DIA3(spi_rx_shift[2]), .DIA4(spi_rx_shift[3]), .DIA5(spi_rx_shift[4]), 
          .DIA6(spi_rx_shift[5]), .DIA7(spi_rx_shift[6]), .DIA8(GND_net), 
          .ADA0(VCC_net), .ADA1(GND_net), .ADA2(GND_net), .ADA3(spi_channel_index[0]), 
          .ADA4(spi_channel_index[1]), .ADA5(spi_channel_index[2]), .ADA6(spi_channel_index[3]), 
          .ADA7(spi_channel_index[4]), .ADA8(spi_channel_index[5]), .ADA9(spi_channel_index[6]), 
          .ADA10(GND_net), .ADA11(GND_net), .ADA12(GND_net), .CEA(VCC_net), 
          .OCEA(VCC_net), .CLKA(spi1_sck_c), .WEA(mem_write_phase), .CSA0(GND_net), 
          .CSA1(GND_net), .CSA2(GND_net), .RSTA(GND_net), .DIB0(GND_net), 
          .DIB1(GND_net), .DIB2(GND_net), .DIB3(GND_net), .DIB4(GND_net), 
          .DIB5(GND_net), .DIB6(GND_net), .DIB7(GND_net), .DIB8(GND_net), 
          .ADB0(VCC_net), .ADB1(GND_net), .ADB2(GND_net), .ADB3(load_index[0]), 
          .ADB4(load_index[1]), .ADB5(load_index[2]), .ADB6(load_index[3]), 
          .ADB7(load_index[4]), .ADB8(load_index[5]), .ADB9(load_index[6]), 
          .ADB10(GND_net), .ADB11(GND_net), .ADB12(GND_net), .CEB(VCC_net), 
          .OCEB(VCC_net), .CLKB(fpga_clk_c), .WEB(GND_net), .CSB0(GND_net), 
          .CSB1(GND_net), .CSB2(GND_net), .RSTB(GND_net), .DOB7(phase_mem_dout[7])) /* synthesis syn_instantiated=1 */ ;
    defparam phase_mem_i.DATA_WIDTH_A = 9;
    defparam phase_mem_i.DATA_WIDTH_B = 9;
    defparam phase_mem_i.REGMODE_A = "NOREG";
    defparam phase_mem_i.REGMODE_B = "NOREG";
    defparam phase_mem_i.CSDECODE_A = "0b000";
    defparam phase_mem_i.CSDECODE_B = "0b000";
    defparam phase_mem_i.WRITEMODE_A = "NORMAL";
    defparam phase_mem_i.WRITEMODE_B = "NORMAL";
    defparam phase_mem_i.GSR = "ENABLED";
    defparam phase_mem_i.RESETMODE = "SYNC";
    defparam phase_mem_i.ASYNC_RESET_RELEASE = "SYNC";
    defparam phase_mem_i.INIT_DATA = "STATIC";
    defparam phase_mem_i.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam phase_mem_i.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    LUT4 i30_3_lut_4_lut_adj_23 (.A(amplitude_phase[6]), .B(\level_active[81] [0]), 
         .C(\level_active[81] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3503)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_23.init = 16'h40f4;
    FD1P3DX rgb_values_i19_5913_5914_reset (.D(n10125), .SP(spi1_sck_c_enable_1), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_856), .Q(n10126)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i19_5913_5914_reset.GSR = "ENABLED";
    LUT4 i30_3_lut_4_lut_adj_24 (.A(amplitude_phase[6]), .B(\level_active[24] [0]), 
         .C(\level_active[24] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3469)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_24.init = 16'h40f4;
    LUT4 i15978_2_lut_3_lut (.A(stop_toggle_spi_N_2878), .B(frame_toggle_spi_N_2870), 
         .C(accepted_sequence_spi_31__N_2262), .Z(invalid_frame_spi_N_2881)) /* synthesis lut_function=(!(A (C)+!A (B (C)))) */ ;   // src/umh_fpga_top.v(262[18] 267[12])
    defparam i15978_2_lut_3_lut.init = 16'h1f1f;
    LUT4 i15455_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[3]), 
         .Z(spi_channel_index_6__N_1760)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15455_2_lut.init = 16'h1111;
    FD1P3AX level_active_83___i156 (.D(n2315), .SP(fpga_clk_c_enable_409), 
            .CK(fpga_clk_c), .Q(\level_active[6] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i156.GSR = "ENABLED";
    FD1P3AX level_active_83___i155 (.D(n2316), .SP(fpga_clk_c_enable_409), 
            .CK(fpga_clk_c), .Q(\level_active[6] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i155.GSR = "ENABLED";
    FD1P3AX level_active_83___i154 (.D(n2315), .SP(fpga_clk_c_enable_408), 
            .CK(fpga_clk_c), .Q(\level_active[7] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i154.GSR = "ENABLED";
    FD1P3AX level_active_83___i153 (.D(n2316), .SP(fpga_clk_c_enable_408), 
            .CK(fpga_clk_c), .Q(\level_active[7] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i153.GSR = "ENABLED";
    FD1P3AX level_active_83___i152 (.D(n2315), .SP(fpga_clk_c_enable_407), 
            .CK(fpga_clk_c), .Q(\level_active[8] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i152.GSR = "ENABLED";
    FD1P3DX rgb_values_i20_5909_5910_reset (.D(n10121), .SP(spi1_sck_c_enable_2), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_853), .Q(n10122)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i20_5909_5910_reset.GSR = "ENABLED";
    OB us_tx_pad_2 (.I(us_tx_c_2), .O(us_tx[2]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX level_active_83___i87 (.D(n2316), .SP(fpga_clk_c_enable_375), 
            .CK(fpga_clk_c), .Q(\level_active[40] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i87.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i27_5409_5410_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1354), .Q(n9621)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i27_5409_5410_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i151 (.D(n2316), .SP(fpga_clk_c_enable_407), 
            .CK(fpga_clk_c), .Q(\level_active[8] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i151.GSR = "ENABLED";
    FD1P3AX level_active_83___i150 (.D(n2315), .SP(fpga_clk_c_enable_406), 
            .CK(fpga_clk_c), .Q(\level_active[9] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i150.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2062_5 (.A0(n9769), .B0(n9768), .C0(GND_net), 
          .D0(GND_net), .A1(n9773), .B1(n9772), .C1(GND_net), .D1(GND_net), 
          .CIN(n17125), .COUT(n17126), .S0(spi_byte_count_15__N_997[3]), 
          .S1(spi_byte_count_15__N_997[4]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_5.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_5.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_5.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_5.INJECT1_1 = "NO";
    OB us_tx_pad_78 (.I(us_tx_c_78), .O(us_tx[78]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_79 (.I(us_tx_c_79), .O(us_tx[79]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX level_active_83___i149 (.D(n2316), .SP(fpga_clk_c_enable_406), 
            .CK(fpga_clk_c), .Q(\level_active[9] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i149.GSR = "ENABLED";
    FD1P3DX rgb_values_i21_5905_5906_reset (.D(n10117), .SP(spi1_sck_c_enable_3), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_850), .Q(n10118)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i21_5905_5906_reset.GSR = "ENABLED";
    FD1P3AX level_active_83___i1 (.D(n2316), .SP(fpga_clk_c_enable_188), 
            .CK(fpga_clk_c), .Q(\level_active[83] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i1.GSR = "ENABLED";
    FD1P3AX level_active_83___i148 (.D(n2315), .SP(fpga_clk_c_enable_405), 
            .CK(fpga_clk_c), .Q(\level_active[10] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i148.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i12_5137_5138_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1576), .Q(n9349)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i12_5137_5138_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i86 (.D(n2315), .SP(fpga_clk_c_enable_374), 
            .CK(fpga_clk_c), .Q(\level_active[41] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i86.GSR = "ENABLED";
    FD1P3AX invalid_frame_spi_1506 (.D(invalid_frame_spi_N_2881), .SP(fpga_cs_n_c_enable_1), 
            .CK(fpga_cs_n_c), .Q(status_flags_wire_15__N_2034[2])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam invalid_frame_spi_1506.GSR = "ENABLED";
    LUT4 i24_3_lut_4_lut (.A(stop_toggle_spi_N_2878), .B(frame_toggle_spi_N_2870), 
         .C(accepted_sequence_spi_31__N_2262), .D(spi_command[0]), .Z(n18818)) /* synthesis lut_function=(A (C+(D))+!A (B (C+(D))+!B !(C (D)+!C !(D)))) */ ;   // src/umh_fpga_top.v(262[18] 267[12])
    defparam i24_3_lut_4_lut.init = 16'heff0;
    FD1P3AX level_active_83___i147 (.D(n2316), .SP(fpga_clk_c_enable_405), 
            .CK(fpga_clk_c), .Q(\level_active[10] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i147.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i7_5117_5118_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1586), .Q(n9329)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i7_5117_5118_set.GSR = "ENABLED";
    LUT4 i15872_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3418), .Z(fpga_clk_c_enable_405)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15872_2_lut_4_lut.init = 16'h0002;
    FD1P3DX rgb_values_i22_5901_5902_reset (.D(n10113), .SP(spi1_sck_c_enable_4), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_847), .Q(n10114)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i22_5901_5902_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i23_5897_5898_reset (.D(n10109), .SP(spi1_sck_c_enable_5), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_844), .Q(n10110)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i23_5897_5898_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i24_5893_5894_reset (.D(n10105), .SP(spi1_sck_c_enable_6), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_841), .Q(n10106)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i24_5893_5894_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i25_5889_5890_reset (.D(n10101), .SP(spi1_sck_c_enable_7), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_838), .Q(n10102)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i25_5889_5890_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i26_5885_5886_reset (.D(n10097), .SP(spi1_sck_c_enable_8), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_835), .Q(n10098)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i26_5885_5886_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i27_5881_5882_reset (.D(n10093), .SP(spi1_sck_c_enable_9), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_832), .Q(n10094)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i27_5881_5882_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i28_5877_5878_reset (.D(n10089), .SP(spi1_sck_c_enable_10), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_829), .Q(n10090)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i28_5877_5878_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i29_5873_5874_reset (.D(n10085), .SP(spi1_sck_c_enable_11), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_826), .Q(n10086)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i29_5873_5874_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i30_5869_5870_reset (.D(n10081), .SP(spi1_sck_c_enable_12), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_823), .Q(n10082)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i30_5869_5870_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i31_5865_5866_reset (.D(n10077), .SP(spi1_sck_c_enable_13), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_820), .Q(n10078)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i31_5865_5866_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i32_5861_5862_reset (.D(n10073), .SP(spi1_sck_c_enable_14), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_817), .Q(n10074)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i32_5861_5862_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i33_5857_5858_reset (.D(n10069), .SP(spi1_sck_c_enable_15), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_814), .Q(n10070)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i33_5857_5858_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i34_5853_5854_reset (.D(n10065), .SP(spi1_sck_c_enable_16), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_811), .Q(n10066)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i34_5853_5854_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i35_5849_5850_reset (.D(n10061), .SP(spi1_sck_c_enable_17), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_808), .Q(n10062)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i35_5849_5850_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i36_5845_5846_reset (.D(n10057), .SP(spi1_sck_c_enable_18), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_805), .Q(n10058)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i36_5845_5846_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i37_5841_5842_reset (.D(n10053), .SP(spi1_sck_c_enable_19), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_802), .Q(n10054)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i37_5841_5842_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i38_5837_5838_reset (.D(n10049), .SP(spi1_sck_c_enable_20), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_799), .Q(n10050)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i38_5837_5838_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i39_5833_5834_reset (.D(n10045), .SP(spi1_sck_c_enable_21), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_796), .Q(n10046)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i39_5833_5834_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i40_5829_5830_reset (.D(n10041), .SP(spi1_sck_c_enable_22), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_793), .Q(n10042)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i40_5829_5830_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i41_5825_5826_reset (.D(n10037), .SP(spi1_sck_c_enable_23), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_790), .Q(n10038)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i41_5825_5826_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i42_5821_5822_reset (.D(n10033), .SP(spi1_sck_c_enable_24), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_787), .Q(n10034)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i42_5821_5822_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i43_5817_5818_reset (.D(n10029), .SP(spi1_sck_c_enable_25), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_784), .Q(n10030)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i43_5817_5818_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i44_5813_5814_reset (.D(n10025), .SP(spi1_sck_c_enable_26), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_781), .Q(n10026)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i44_5813_5814_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i45_5809_5810_reset (.D(n10021), .SP(spi1_sck_c_enable_27), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_778), .Q(n10022)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i45_5809_5810_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i46_5805_5806_reset (.D(n10017), .SP(spi1_sck_c_enable_28), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_775), .Q(n10018)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i46_5805_5806_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i47_5801_5802_reset (.D(n10013), .SP(spi1_sck_c_enable_29), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_772), .Q(n10014)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i47_5801_5802_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i48_5797_5798_reset (.D(n10009), .SP(spi1_sck_c_enable_30), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_769), .Q(n10010)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i48_5797_5798_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i49_5793_5794_reset (.D(n10005), .SP(spi1_sck_c_enable_31), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_766), .Q(n10006)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i49_5793_5794_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i50_5789_5790_reset (.D(n10001), .SP(spi1_sck_c_enable_32), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_763), .Q(n10002)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i50_5789_5790_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i51_5785_5786_reset (.D(n9997), .SP(spi1_sck_c_enable_33), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_760), .Q(n9998)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i51_5785_5786_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i52_5781_5782_reset (.D(n9993), .SP(spi1_sck_c_enable_34), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_757), .Q(n9994)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i52_5781_5782_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i53_5777_5778_reset (.D(n9989), .SP(spi1_sck_c_enable_35), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_754), .Q(n9990)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i53_5777_5778_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i54_5773_5774_reset (.D(n9985), .SP(spi1_sck_c_enable_36), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_751), .Q(n9986)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i54_5773_5774_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i55_5769_5770_reset (.D(n9981), .SP(spi1_sck_c_enable_37), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_748), .Q(n9982)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i55_5769_5770_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i56_5765_5766_reset (.D(n9977), .SP(spi1_sck_c_enable_38), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_745), .Q(n9978)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i56_5765_5766_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i57_5761_5762_reset (.D(n9973), .SP(spi1_sck_c_enable_39), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_742), .Q(n9974)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i57_5761_5762_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i58_5757_5758_reset (.D(n9969), .SP(spi1_sck_c_enable_40), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_739), .Q(n9970)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i58_5757_5758_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i59_5753_5754_reset (.D(n9965), .SP(spi1_sck_c_enable_41), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_736), .Q(n9966)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i59_5753_5754_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i60_5749_5750_reset (.D(n9961), .SP(spi1_sck_c_enable_42), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_733), .Q(n9962)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i60_5749_5750_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i61_5745_5746_reset (.D(n9957), .SP(spi1_sck_c_enable_43), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_730), .Q(n9958)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i61_5745_5746_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i62_5741_5742_reset (.D(n9953), .SP(spi1_sck_c_enable_44), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_727), .Q(n9954)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i62_5741_5742_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i63_5737_5738_reset (.D(n9949), .SP(spi1_sck_c_enable_45), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_724), .Q(n9950)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i63_5737_5738_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i64_5733_5734_reset (.D(n9945), .SP(spi1_sck_c_enable_46), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_721), .Q(n9946)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i64_5733_5734_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i65_5729_5730_reset (.D(n9941), .SP(spi1_sck_c_enable_47), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_718), .Q(n9942)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i65_5729_5730_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i66_5725_5726_reset (.D(n9937), .SP(spi1_sck_c_enable_48), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_715), .Q(n9938)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i66_5725_5726_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i67_5721_5722_reset (.D(n9933), .SP(spi1_sck_c_enable_49), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_712), .Q(n9934)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i67_5721_5722_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i68_5717_5718_reset (.D(n9929), .SP(spi1_sck_c_enable_50), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_709), .Q(n9930)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i68_5717_5718_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i69_5713_5714_reset (.D(n9925), .SP(spi1_sck_c_enable_51), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_706), .Q(n9926)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i69_5713_5714_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i70_5709_5710_reset (.D(n9921), .SP(spi1_sck_c_enable_52), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_703), .Q(n9922)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i70_5709_5710_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i71_5705_5706_reset (.D(n9917), .SP(spi1_sck_c_enable_53), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_700), .Q(n9918)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i71_5705_5706_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i72_5701_5702_reset (.D(n9913), .SP(spi1_sck_c_enable_54), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_697), .Q(n9914)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i72_5701_5702_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i73_5697_5698_reset (.D(n9909), .SP(spi1_sck_c_enable_55), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_694), .Q(n9910)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i73_5697_5698_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i74_5693_5694_reset (.D(n9905), .SP(spi1_sck_c_enable_56), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_691), .Q(n9906)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i74_5693_5694_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i75_5689_5690_reset (.D(n9901), .SP(spi1_sck_c_enable_57), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_688), .Q(n9902)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i75_5689_5690_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i76_5685_5686_reset (.D(n9897), .SP(spi1_sck_c_enable_58), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_685), .Q(n9898)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i76_5685_5686_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i77_5681_5682_reset (.D(n9893), .SP(spi1_sck_c_enable_59), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_682), .Q(n9894)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i77_5681_5682_reset.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i0 (.D(accepted_sequence_spi[0]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[0])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i0.GSR = "ENABLED";
    FD1P3DX rgb_values_i78_5677_5678_reset (.D(n9889), .SP(spi1_sck_c_enable_60), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_679), .Q(n9890)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i78_5677_5678_reset.GSR = "ENABLED";
    FD1P3AX level_active_83___i146 (.D(n2315), .SP(fpga_clk_c_enable_404), 
            .CK(fpga_clk_c), .Q(\level_active[11] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i146.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_2008_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[4]), 
         .Z(spi_channel_index_6__N_1734)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2008_2_lut.init = 16'h4444;
    LUT4 i15452_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[4]), 
         .Z(spi_channel_index_6__N_1757)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15452_2_lut.init = 16'h1111;
    FD1P3AX level_active_83___i145 (.D(n2316), .SP(fpga_clk_c_enable_404), 
            .CK(fpga_clk_c), .Q(\level_active[11] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i145.GSR = "ENABLED";
    FD1P3AX level_active_83___i85 (.D(n2316), .SP(fpga_clk_c_enable_374), 
            .CK(fpga_clk_c), .Q(\level_active[41] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i85.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i6_5113_5114_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1588), .Q(n9325)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i6_5113_5114_set.GSR = "ENABLED";
    LUT4 i15118_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3388), .Z(fpga_clk_c_enable_373)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15118_2_lut_4_lut.init = 16'h0002;
    CCU2D add_2583_5 (.A0(spi_extension_length[4]), .B0(n37_adj_3400), .C0(spi_expected_length[4]), 
          .D0(n7008), .A1(spi_extension_length[5]), .B1(n37_adj_3400), 
          .C1(spi_expected_length[5]), .D1(n7007), .CIN(n17135), .COUT(n17136), 
          .S0(spi_expected_length_31__N_2085[4]), .S1(spi_expected_length_31__N_2085[5]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_5.INIT0 = 16'hd1e2;
    defparam add_2583_5.INIT1 = 16'hd1e2;
    defparam add_2583_5.INJECT1_0 = "NO";
    defparam add_2583_5.INJECT1_1 = "NO";
    FD1S3AX time_half_1510 (.D(time_half_N_2907), .CK(fpga_clk_c), .Q(time_divider_5__N_2270[1])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam time_half_1510.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1625_2_lut_4_lut (.A(rgb_values[7]), .B(spi_rx_shift[6]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_515)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1625_2_lut_4_lut.init = 16'h00ca;
    FD1S3AX frame_toggle_meta_1512 (.D(frame_toggle_spi), .CK(fpga_clk_c), 
            .Q(frame_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam frame_toggle_meta_1512.GSR = "ENABLED";
    FD1S3AX frame_toggle_sync_1513 (.D(frame_toggle_meta), .CK(fpga_clk_c), 
            .Q(frame_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam frame_toggle_sync_1513.GSR = "ENABLED";
    FD1S3AX stop_toggle_meta_1514 (.D(stop_toggle_spi), .CK(fpga_clk_c), 
            .Q(stop_toggle_meta)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam stop_toggle_meta_1514.GSR = "ENABLED";
    FD1S3AX stop_toggle_sync_1515 (.D(stop_toggle_meta), .CK(fpga_clk_c), 
            .Q(stop_toggle_sync)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam stop_toggle_sync_1515.GSR = "ENABLED";
    FD1S3DX spi_expected_length_i5_5109_5110_reset (.D(n21165), .CK(spi1_sck_c), 
            .CD(spi_expected_length_31__N_1712), .Q(n9322)) /* synthesis lse_init_val=1 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i5_5109_5110_reset.GSR = "ENABLED";
    FD1P3IX apply_pending_1517 (.D(n21165), .SP(frame_toggle_seen_N_2889), 
            .CD(n7832), .CK(fpga_clk_c), .Q(fifo_depth[0])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam apply_pending_1517.GSR = "ENABLED";
    FD1P3IX load_active_1518 (.D(n21165), .SP(frame_toggle_seen_N_2889), 
            .CD(fpga_clk_c_enable_188), .CK(fpga_clk_c), .Q(load_active)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam load_active_1518.GSR = "ENABLED";
    FD1P3AX running_1520 (.D(running_N_2903), .SP(fpga_clk_c_enable_23), 
            .CK(fpga_clk_c), .Q(status_flags_wire_15__N_2050[4])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam running_1520.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i1 (.D(mic_data_0_c), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[0])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i1.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i1 (.D(mic_data_1_c), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[0])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i1.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i0 (.D(mic_data_1_c), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[0])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i0.GSR = "ENABLED";
    FD1S3AX mic_clock_reg_1527 (.D(mic_clk_N_2867), .CK(fpga_clk_c), .Q(mic_clk_c)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_clock_reg_1527.GSR = "ENABLED";
    FD1S1D i5044 (.D(n21165), .CK(spi_bit_count_2__N_948), .CD(spi_bit_count_2__N_959), 
           .Q(n9256));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5044.GSR = "ENABLED";
    FD1S1D i5048 (.D(n21165), .CK(spi_rx_shift_6__N_922), .CD(spi_rx_shift_0__N_941), 
           .Q(n9260));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5048.GSR = "ENABLED";
    FD1S1D i5052 (.D(n21165), .CK(spi_command_7__N_1072), .CD(spi_command_7__N_1103), 
           .Q(n9264));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5052.GSR = "ENABLED";
    OB us_tx_pad_3 (.I(us_tx_c_3), .O(us_tx[3]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5056 (.D(n21165), .CK(spi_version_7__N_1120), .CD(spi_version_7__N_1151), 
           .Q(n9268));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5056.GSR = "ENABLED";
    LUT4 i15863_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3407), .Z(fpga_clk_c_enable_397)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15863_2_lut_4_lut.init = 16'h0002;
    LUT4 fpga_cs_n_N_339_I_0_2007_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[5]), 
         .Z(spi_channel_index_6__N_1732)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2007_2_lut.init = 16'h4444;
    FD1S1D i5060 (.D(n21165), .CK(spi_frame_sequence_31__N_1408), .CD(spi_frame_sequence_31__N_1535), 
           .Q(n9272));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5060.GSR = "ENABLED";
    LUT4 i15860_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3383), .Z(fpga_clk_c_enable_365)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15860_2_lut_4_lut.init = 16'h0002;
    CCU2D add_2583_3 (.A0(spi_extension_length[2]), .B0(n37_adj_3400), .C0(spi_expected_length[2]), 
          .D0(spi_update_flags[1]), .A1(n17086), .B1(n8_adj_3389), .C1(n7), 
          .D1(spi_expected_length[3]), .CIN(n17134), .COUT(n17135), .S0(spi_expected_length_31__N_2085[2]), 
          .S1(spi_expected_length_31__N_2085[3]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_3.INIT0 = 16'hd1e2;
    defparam add_2583_3.INIT1 = 16'h56aa;
    defparam add_2583_3.INJECT1_0 = "NO";
    defparam add_2583_3.INJECT1_1 = "NO";
    LUT4 i16065_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3403), .Z(fpga_clk_c_enable_389)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16065_2_lut_4_lut.init = 16'h0002;
    LUT4 i15761_2_lut_4_lut (.A(rgb_values[7]), .B(spi_rx_shift[6]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_892)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15761_2_lut_4_lut.init = 16'h0035;
    FD1S1D i5064 (.D(n21165), .CK(spi_update_flags_15__N_1184), .CD(spi_update_flags_15__N_1247), 
           .Q(n9276));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5064.GSR = "ENABLED";
    LUT4 i16062_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3378), .Z(fpga_clk_c_enable_357)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16062_2_lut_4_lut.init = 16'h0002;
    LUT4 i15952_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3402), .Z(fpga_clk_c_enable_381)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15952_2_lut_4_lut.init = 16'h0002;
    LUT4 i15449_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[5]), 
         .Z(spi_channel_index_6__N_1754)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15449_2_lut.init = 16'h1111;
    LUT4 i15955_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3433), .Z(fpga_clk_c_enable_349)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15955_2_lut_4_lut.init = 16'h0002;
    LUT4 i15917_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3423), .Z(fpga_clk_c_enable_341)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15917_2_lut_4_lut.init = 16'h0002;
    LUT4 i15897_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12), .Z(fpga_clk_c_enable_333)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15897_2_lut_4_lut.init = 16'h0002;
    LUT4 i15883_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3436), 
         .D(n12_adj_3438), .Z(fpga_clk_c_enable_413)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15883_2_lut_4_lut.init = 16'h0002;
    FD1S1D i5068 (.D(n21165), .CK(spi_extension_length_15__N_1280), .CD(spi_extension_length_15__N_1343), 
           .Q(n9280));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5068.GSR = "ENABLED";
    LUT4 load_index_6__I_0_i12_2_lut_3_lut_4_lut (.A(load_index[3]), .B(load_index[4]), 
         .C(load_index[6]), .D(load_index[5]), .Z(n12)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam load_index_6__I_0_i12_2_lut_3_lut_4_lut.init = 16'hffbf;
    LUT4 fpga_cs_n_N_339_I_0_2006_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[6]), 
         .Z(spi_channel_index_6__N_1730)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2006_2_lut.init = 16'h4444;
    LUT4 i15446_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[6]), 
         .Z(spi_channel_index_6__N_1744)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15446_2_lut.init = 16'h1111;
    LUT4 i15886_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3438), 
         .D(load_active), .Z(fpga_clk_c_enable_412)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15886_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i15943_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3433), 
         .D(load_active), .Z(fpga_clk_c_enable_348)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15943_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i15857_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3407), 
         .D(load_active), .Z(fpga_clk_c_enable_396)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15857_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i3_4_lut (.A(spi_channel_field[1]), .B(n18643), .C(n18548), .D(n18752), 
         .Z(n8872)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i3_4_lut.init = 16'h0080;
    LUT4 i15845_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3418), 
         .D(load_active), .Z(fpga_clk_c_enable_404)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15845_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i15961_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3402), 
         .D(load_active), .Z(fpga_clk_c_enable_380)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15961_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 fpga_cs_n_N_339_I_0_2030_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[1]), 
         .Z(spi_level_pending_7__N_1796)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2030_2_lut.init = 16'h4444;
    LUT4 i16056_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3378), 
         .D(load_active), .Z(fpga_clk_c_enable_356)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i16056_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i2_4_lut_adj_25 (.A(n13787), .B(n123), .C(n13789), .D(n20623), 
         .Z(n18548)) /* synthesis lut_function=(!(A+(B (C)+!B (C+!(D))))) */ ;
    defparam i2_4_lut_adj_25.init = 16'h0504;
    LUT4 i1_4_lut (.A(spi_byte_count[8]), .B(spi_byte_count[7]), .C(spi_byte_count[5]), 
         .D(n128), .Z(n123)) /* synthesis lut_function=(!(A+!(B+(C (D))))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i1_4_lut.init = 16'h5444;
    LUT4 i30_3_lut_4_lut_adj_26 (.A(amplitude_phase[6]), .B(\level_active[32] [0]), 
         .C(\level_active[32] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3485)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_26.init = 16'h40f4;
    LUT4 i15115_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3388), 
         .D(load_active), .Z(fpga_clk_c_enable_372)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15115_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i15485_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[1]), 
         .Z(spi_level_pending_7__N_1826)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15485_2_lut.init = 16'h1111;
    FD1P3DX rgb_values_i79_5673_5674_reset (.D(n9885), .SP(spi1_sck_c_enable_61), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_676), .Q(n9886)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i79_5673_5674_reset.GSR = "ENABLED";
    LUT4 i15923_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3423), 
         .D(load_active), .Z(fpga_clk_c_enable_340)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15923_2_lut_3_lut_4_lut.init = 16'h0200;
    FD1P3DX rgb_values_i80_5669_5670_reset (.D(n9881), .SP(spi1_sck_c_enable_62), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_673), .Q(n9882)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i80_5669_5670_reset.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i4_5105_5106_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1592), .Q(n9317)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i4_5105_5106_set.GSR = "ENABLED";
    FD1P3DX rgb_values_i81_5665_5666_reset (.D(n9877), .SP(spi1_sck_c_enable_63), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_670), .Q(n9878)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i81_5665_5666_reset.GSR = "ENABLED";
    FD1S1D i5072 (.D(n21165), .CK(spi_channel_index_6__N_1742), .CD(spi_channel_index_6__N_1769), 
           .Q(n9284));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5072.GSR = "ENABLED";
    LUT4 i15854_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3383), 
         .D(load_active), .Z(fpga_clk_c_enable_364)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i15854_2_lut_3_lut_4_lut.init = 16'h0200;
    LUT4 i16059_2_lut_3_lut_4_lut (.A(load_index[0]), .B(n8_adj_3436), .C(n12_adj_3403), 
         .D(load_active), .Z(fpga_clk_c_enable_388)) /* synthesis lut_function=(!((B+(C+!(D)))+!A)) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam i16059_2_lut_3_lut_4_lut.init = 16'h0200;
    FD1S1D i5076 (.D(n21165), .CK(spi_level_pending_7__N_1798), .CD(spi_level_pending_7__N_1829), 
           .Q(n9288));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5076.GSR = "ENABLED";
    LUT4 i15719_2_lut_4_lut (.A(rgb_values[21]), .B(rgb_values[13]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_850)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15719_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1611_2_lut_4_lut (.A(rgb_values[21]), .B(rgb_values[13]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_487)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1611_2_lut_4_lut.init = 16'h00ca;
    LUT4 i30_3_lut_4_lut_adj_27 (.A(amplitude_phase[6]), .B(\level_active[79] [0]), 
         .C(\level_active[79] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3501)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_27.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_2029_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[2]), 
         .Z(spi_level_pending_7__N_1794)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2029_2_lut.init = 16'h4444;
    LUT4 i30_3_lut_4_lut_adj_28 (.A(amplitude_phase[6]), .B(\level_active[23] [0]), 
         .C(\level_active[23] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3464)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_28.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_29 (.A(amplitude_phase[6]), .B(\level_active[78] [0]), 
         .C(\level_active[78] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3506)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_29.init = 16'h40f4;
    LUT4 i15877_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3418), .Z(fpga_clk_c_enable_406)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15877_2_lut_4_lut.init = 16'h0008;
    LUT4 i15869_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3388), .Z(fpga_clk_c_enable_374)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15869_2_lut_4_lut.init = 16'h0008;
    LUT4 i15937_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3407), .Z(fpga_clk_c_enable_398)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15937_2_lut_4_lut.init = 16'h0008;
    LUT4 i15482_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[2]), 
         .Z(spi_level_pending_7__N_1823)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15482_2_lut.init = 16'h1111;
    LUT4 i15866_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3383), .Z(fpga_clk_c_enable_366)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15866_2_lut_4_lut.init = 16'h0008;
    LUT4 i16071_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3403), .Z(fpga_clk_c_enable_390)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16071_2_lut_4_lut.init = 16'h0008;
    LUT4 i16068_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3378), .Z(fpga_clk_c_enable_358)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16068_2_lut_4_lut.init = 16'h0008;
    FD1S1D i5080 (.D(n21165), .CK(spi_channel_field_1__N_1774), .CD(spi_channel_field_1__N_1781), 
           .Q(n9292));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5080.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_2028_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[3]), 
         .Z(spi_level_pending_7__N_1792)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2028_2_lut.init = 16'h4444;
    LUT4 i15981_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3402), .Z(fpga_clk_c_enable_382)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15981_2_lut_4_lut.init = 16'h0008;
    LUT4 i15479_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[3]), 
         .Z(spi_level_pending_7__N_1820)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15479_2_lut.init = 16'h1111;
    LUT4 i15946_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3433), .Z(fpga_clk_c_enable_350)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15946_2_lut_4_lut.init = 16'h0008;
    LUT4 i15377_2_lut_3_lut (.A(spi_expected_length[22]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1661)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15377_2_lut_3_lut.init = 16'h0707;
    LUT4 i15914_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3423), .Z(fpga_clk_c_enable_342)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15914_2_lut_4_lut.init = 16'h0008;
    LUT4 i15894_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12), .Z(fpga_clk_c_enable_334)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15894_2_lut_4_lut.init = 16'h0008;
    LUT4 i15880_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3438), .Z(fpga_clk_c_enable_414)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15880_2_lut_4_lut.init = 16'h0008;
    LUT4 fpga_cs_n_N_339_I_0_2027_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[4]), 
         .Z(spi_level_pending_7__N_1790)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2027_2_lut.init = 16'h4444;
    LUT4 i15476_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[4]), 
         .Z(spi_level_pending_7__N_1817)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15476_2_lut.init = 16'h1111;
    LUT4 fpga_cs_n_N_339_I_0_1882_2_lut_4_lut (.A(spi_frame_sequence[27]), 
         .B(spi_rx_shift[2]), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1354)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1882_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15211_2_lut_4_lut (.A(spi_frame_sequence[27]), .B(spi_rx_shift[2]), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1454)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15211_2_lut_4_lut.init = 16'h0035;
    LUT4 i30_3_lut_4_lut_adj_30 (.A(amplitude_phase[6]), .B(\level_active[2] [0]), 
         .C(\level_active[2] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3511)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_30.init = 16'h40f4;
    LUT4 i15722_2_lut_4_lut (.A(rgb_values[20]), .B(rgb_values[12]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_853)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15722_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1612_2_lut_4_lut (.A(rgb_values[20]), .B(rgb_values[12]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_489)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1612_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15994_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3438), .Z(fpga_clk_c_enable_409)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15994_2_lut_4_lut.init = 16'h0020;
    LUT4 fpga_cs_n_N_339_I_0_2026_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[5]), 
         .Z(spi_level_pending_7__N_1788)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2026_2_lut.init = 16'h4444;
    LUT4 i16022_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3418), .Z(fpga_clk_c_enable_401)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16022_2_lut_4_lut.init = 16'h0020;
    LUT4 i16016_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3388), .Z(fpga_clk_c_enable_369)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16016_2_lut_4_lut.init = 16'h0020;
    LUT4 i15836_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3407), .Z(fpga_clk_c_enable_393)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15836_2_lut_4_lut.init = 16'h0020;
    LUT4 i15833_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3383), .Z(fpga_clk_c_enable_361)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15833_2_lut_4_lut.init = 16'h0020;
    LUT4 fpga_cs_n_N_339_I_0_1952_2_lut_3_lut (.A(spi_expected_length[21]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1558)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1952_2_lut_3_lut.init = 16'h0808;
    LUT4 i16041_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3403), .Z(fpga_clk_c_enable_385)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16041_2_lut_4_lut.init = 16'h0020;
    LUT4 i15473_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[5]), 
         .Z(spi_level_pending_7__N_1814)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15473_2_lut.init = 16'h1111;
    LUT4 i16038_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3378), .Z(fpga_clk_c_enable_353)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16038_2_lut_4_lut.init = 16'h0020;
    DP8KC level_mem_i (.DIA0(spi_level_pending[0]), .DIA1(spi_level_pending[1]), 
          .DIA2(spi_level_pending[2]), .DIA3(spi_level_pending[3]), .DIA4(spi_level_pending[4]), 
          .DIA5(spi_level_pending[5]), .DIA6(spi_level_pending[6]), .DIA7(spi_level_pending[7]), 
          .DIA8(level_mem_din[8]), .ADA0(VCC_net), .ADA1(GND_net), .ADA2(GND_net), 
          .ADA3(spi_channel_index[0]), .ADA4(spi_channel_index[1]), .ADA5(spi_channel_index[2]), 
          .ADA6(spi_channel_index[3]), .ADA7(spi_channel_index[4]), .ADA8(spi_channel_index[5]), 
          .ADA9(spi_channel_index[6]), .ADA10(GND_net), .ADA11(GND_net), 
          .ADA12(GND_net), .CEA(VCC_net), .OCEA(VCC_net), .CLKA(spi1_sck_c), 
          .WEA(mem_write_level), .CSA0(GND_net), .CSA1(GND_net), .CSA2(GND_net), 
          .RSTA(GND_net), .DIB0(GND_net), .DIB1(GND_net), .DIB2(GND_net), 
          .DIB3(GND_net), .DIB4(GND_net), .DIB5(GND_net), .DIB6(GND_net), 
          .DIB7(GND_net), .DIB8(GND_net), .ADB0(VCC_net), .ADB1(GND_net), 
          .ADB2(GND_net), .ADB3(load_index[0]), .ADB4(load_index[1]), 
          .ADB5(load_index[2]), .ADB6(load_index[3]), .ADB7(load_index[4]), 
          .ADB8(load_index[5]), .ADB9(load_index[6]), .ADB10(GND_net), 
          .ADB11(GND_net), .ADB12(GND_net), .CEB(VCC_net), .OCEB(VCC_net), 
          .CLKB(fpga_clk_c), .WEB(GND_net), .CSB0(GND_net), .CSB1(GND_net), 
          .CSB2(GND_net), .RSTB(GND_net), .DOB6(level_mem_dout[6]), .DOB7(level_mem_dout[7]), 
          .DOB8(level_mem_dout[8])) /* synthesis syn_instantiated=1 */ ;
    defparam level_mem_i.DATA_WIDTH_A = 9;
    defparam level_mem_i.DATA_WIDTH_B = 9;
    defparam level_mem_i.REGMODE_A = "NOREG";
    defparam level_mem_i.REGMODE_B = "NOREG";
    defparam level_mem_i.CSDECODE_A = "0b000";
    defparam level_mem_i.CSDECODE_B = "0b000";
    defparam level_mem_i.WRITEMODE_A = "NORMAL";
    defparam level_mem_i.WRITEMODE_B = "NORMAL";
    defparam level_mem_i.GSR = "ENABLED";
    defparam level_mem_i.RESETMODE = "SYNC";
    defparam level_mem_i.ASYNC_RESET_RELEASE = "SYNC";
    defparam level_mem_i.INIT_DATA = "STATIC";
    defparam level_mem_i.INITVAL_00 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_01 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_02 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_03 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_04 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_05 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_06 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_07 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_08 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_09 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_0A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_0B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_0C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_0D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_0E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_0F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_10 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_11 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_12 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_13 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_14 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_15 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_16 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_17 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_18 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_19 = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_1A = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_1B = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_1C = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_1D = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_1E = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    defparam level_mem_i.INITVAL_1F = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000";
    LUT4 i15940_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3402), .Z(fpga_clk_c_enable_377)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15940_2_lut_4_lut.init = 16'h0020;
    LUT4 i15926_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3433), .Z(fpga_clk_c_enable_345)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15926_2_lut_4_lut.init = 16'h0020;
    LUT4 i15905_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3423), .Z(fpga_clk_c_enable_337)) /* synthesis lut_function=(!((B+((D)+!C))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15905_2_lut_4_lut.init = 16'h0020;
    LUT4 fpga_cs_n_N_339_I_0_2025_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[6]), 
         .Z(spi_level_pending_7__N_1786)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2025_2_lut.init = 16'h4444;
    LUT4 i15470_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[6]), 
         .Z(spi_level_pending_7__N_1811)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15470_2_lut.init = 16'h1111;
    FD1S1D i5084 (.D(n21165), .CK(spi_byte_count_15__N_992), .CD(spi_byte_count_15__N_1055), 
           .Q(n9296));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5084.GSR = "ENABLED";
    LUT4 i2_2_lut_3_lut (.A(spi_byte_count[8]), .B(spi_byte_count[6]), .C(spi_byte_count[7]), 
         .Z(n18527)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_2_lut_3_lut.init = 16'hfefe;
    LUT4 i7_4_lut (.A(spi1_mosi_c), .B(n14_adj_3514), .C(n10_adj_3515), 
         .D(spi_rx_shift[5]), .Z(level_mem_din[8])) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // src/umh_fpga_top.v(80[29:67])
    defparam i7_4_lut.init = 16'hfffe;
    LUT4 i30_3_lut_4_lut_adj_31 (.A(amplitude_phase[6]), .B(\level_active[22] [0]), 
         .C(\level_active[22] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3465)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_31.init = 16'h40f4;
    FD1S1D i5088 (.D(n21165), .CK(spi_expected_length_31__N_1600), .CD(spi_expected_length_31__N_1727), 
           .Q(n9300));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5088.GSR = "ENABLED";
    LUT4 i6_4_lut (.A(spi_rx_shift[2]), .B(spi_rx_shift[0]), .C(spi_rx_shift[4]), 
         .D(spi_rx_shift[6]), .Z(n14_adj_3514)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // src/umh_fpga_top.v(80[29:67])
    defparam i6_4_lut.init = 16'hfffe;
    CCU2D add_2583_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(n7), .B1(n8_adj_3389), .C1(GND_net), .D1(GND_net), .COUT(n17134));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_1.INIT0 = 16'hF000;
    defparam add_2583_1.INIT1 = 16'hffff;
    defparam add_2583_1.INJECT1_0 = "NO";
    defparam add_2583_1.INJECT1_1 = "NO";
    LUT4 m0_lut (.Z(n21164)) /* synthesis lut_function=0, syn_instantiated=1 */ ;
    defparam m0_lut.init = 16'h0000;
    LUT4 i9728_2_lut (.A(n9753), .B(n9752), .Z(spi_level_pending[7])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9728_2_lut.init = 16'h8888;
    LUT4 i30_3_lut_4_lut_adj_32 (.A(amplitude_phase[6]), .B(\level_active[21] [0]), 
         .C(\level_active[21] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3460)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_32.init = 16'h40f4;
    LUT4 i9729_2_lut (.A(n9749), .B(n9748), .Z(spi_level_pending[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9729_2_lut.init = 16'h8888;
    LUT4 i9730_2_lut (.A(n9745), .B(n9744), .Z(spi_level_pending[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9730_2_lut.init = 16'h8888;
    LUT4 i30_3_lut_4_lut_adj_33 (.A(amplitude_phase[6]), .B(\level_active[20] [0]), 
         .C(\level_active[20] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3461)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_33.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_34 (.A(amplitude_phase[6]), .B(\level_active[19] [0]), 
         .C(\level_active[19] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3456)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_34.init = 16'h40f4;
    LUT4 i9731_2_lut (.A(n9741), .B(n9740), .Z(spi_level_pending[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9731_2_lut.init = 16'h8888;
    LUT4 i9627_2_lut (.A(n9545), .B(n9544), .Z(spi_frame_sequence[8])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9627_2_lut.init = 16'h8888;
    CCU2D spi_byte_count_15__I_0_2062_17 (.A0(n9817), .B0(n9816), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17131), .S0(spi_byte_count_15__N_997[15]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_17.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_17.INIT1 = 16'h0000;
    defparam spi_byte_count_15__I_0_2062_17.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_17.INJECT1_1 = "NO";
    LUT4 i9732_2_lut (.A(n9737), .B(n9736), .Z(spi_level_pending[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9732_2_lut.init = 16'h8888;
    LUT4 i9733_2_lut (.A(n9733), .B(n9732), .Z(spi_level_pending[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9733_2_lut.init = 16'h8888;
    LUT4 i30_3_lut_4_lut_adj_35 (.A(amplitude_phase[6]), .B(\level_active[18] [0]), 
         .C(\level_active[18] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3457)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_35.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_36 (.A(amplitude_phase[6]), .B(\level_active[17] [0]), 
         .C(\level_active[17] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3452)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_36.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1548_2_lut_4_lut (.A(rgb_values[84]), .B(rgb_values[76]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_361)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1548_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_2024_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[7]), 
         .Z(spi_level_pending_7__N_1784)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2024_2_lut.init = 16'h4444;
    LUT4 i15530_2_lut_4_lut (.A(rgb_values[84]), .B(rgb_values[76]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_661)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15530_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1547_2_lut_4_lut (.A(rgb_values[85]), .B(rgb_values[77]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_359)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1547_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15527_2_lut_4_lut (.A(rgb_values[85]), .B(rgb_values[77]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_658)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15527_2_lut_4_lut.init = 16'h0035;
    LUT4 i15536_2_lut_4_lut (.A(rgb_values[82]), .B(rgb_values[74]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_667)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15536_2_lut_4_lut.init = 16'h0035;
    LUT4 i15467_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[7]), 
         .Z(spi_level_pending_7__N_1800)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15467_2_lut.init = 16'h1111;
    LUT4 fpga_cs_n_N_339_I_0_1550_2_lut_4_lut (.A(rgb_values[82]), .B(rgb_values[74]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_365)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1550_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1546_2_lut_4_lut (.A(rgb_values[86]), .B(rgb_values[78]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_357)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1546_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15524_2_lut_4_lut (.A(rgb_values[86]), .B(rgb_values[78]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_655)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15524_2_lut_4_lut.init = 16'h0035;
    LUT4 i30_3_lut_4_lut_adj_37 (.A(amplitude_phase[6]), .B(\level_active[16] [0]), 
         .C(\level_active[16] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3453)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_37.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1545_2_lut_4_lut (.A(rgb_values[87]), .B(rgb_values[79]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_355)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1545_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15521_2_lut_4_lut (.A(rgb_values[87]), .B(rgb_values[79]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_652)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15521_2_lut_4_lut.init = 16'h0035;
    LUT4 i15725_2_lut_4_lut (.A(rgb_values[19]), .B(rgb_values[11]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_856)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15725_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1613_2_lut_4_lut (.A(rgb_values[19]), .B(rgb_values[11]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_491)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1613_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_2020_2_lut (.A(fpga_cs_n_c), .B(spi_channel_field_1__N_1773[1]), 
         .Z(spi_channel_field_1__N_1772)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2020_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1544_2_lut_4_lut (.A(rgb_values[88]), .B(rgb_values[80]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_353)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1544_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15518_2_lut_4_lut (.A(rgb_values[88]), .B(rgb_values[80]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_649)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15518_2_lut_4_lut.init = 16'h0035;
    LUT4 i30_3_lut_4_lut_adj_38 (.A(amplitude_phase[6]), .B(\level_active[80] [0]), 
         .C(\level_active[80] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3507)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_38.init = 16'h40f4;
    LUT4 i16003_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3438), .Z(fpga_clk_c_enable_411)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16003_2_lut_4_lut.init = 16'h0002;
    LUT4 i15112_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3418), .Z(fpga_clk_c_enable_403)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15112_2_lut_4_lut.init = 16'h0002;
    LUT4 i15102_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3388), .Z(fpga_clk_c_enable_371)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15102_2_lut_4_lut.init = 16'h0002;
    LUT4 i15491_2_lut (.A(fpga_cs_n_c), .B(spi_channel_field_1__N_1773[1]), 
         .Z(spi_channel_field_1__N_1776)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15491_2_lut.init = 16'h1111;
    LUT4 i15851_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3407), .Z(fpga_clk_c_enable_395)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15851_2_lut_4_lut.init = 16'h0002;
    LUT4 i15848_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3383), .Z(fpga_clk_c_enable_363)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15848_2_lut_4_lut.init = 16'h0002;
    LUT4 i16053_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3403), .Z(fpga_clk_c_enable_387)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16053_2_lut_4_lut.init = 16'h0002;
    LUT4 fpga_cs_n_N_339_I_0_1764_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[1]), 
         .Z(spi_byte_count_15__N_990)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1764_2_lut.init = 16'h4444;
    LUT4 i16050_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3378), .Z(fpga_clk_c_enable_355)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16050_2_lut_4_lut.init = 16'h0002;
    LUT4 i15827_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[1]), 
         .Z(spi_byte_count_15__N_1052)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15827_2_lut.init = 16'h1111;
    LUT4 i15949_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3402), .Z(fpga_clk_c_enable_379)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15949_2_lut_4_lut.init = 16'h0002;
    LUT4 i15964_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3433), .Z(fpga_clk_c_enable_347)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15964_2_lut_4_lut.init = 16'h0002;
    LUT4 i15934_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3423), .Z(fpga_clk_c_enable_339)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15934_2_lut_4_lut.init = 16'h0002;
    LUT4 fpga_cs_n_N_339_I_0_1543_2_lut_4_lut (.A(rgb_values[89]), .B(rgb_values[81]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_351)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1543_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1763_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[2]), 
         .Z(spi_byte_count_15__N_988)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1763_2_lut.init = 16'h4444;
    LUT4 i15824_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[2]), 
         .Z(spi_byte_count_15__N_1049)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15824_2_lut.init = 16'h1111;
    LUT4 i15515_2_lut_4_lut (.A(rgb_values[89]), .B(rgb_values[81]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_646)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15515_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1542_2_lut_4_lut (.A(rgb_values[90]), .B(rgb_values[82]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_349)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1542_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1762_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[3]), 
         .Z(spi_byte_count_15__N_986)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1762_2_lut.init = 16'h4444;
    LUT4 i15512_2_lut_4_lut (.A(rgb_values[90]), .B(rgb_values[82]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_643)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15512_2_lut_4_lut.init = 16'h0035;
    LUT4 i2985_2_lut_3_lut_4_lut (.A(n9429), .B(n9428), .C(n9256), .D(n9257), 
         .Z(n7166)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i2985_2_lut_3_lut_4_lut.init = 16'h8000;
    LUT4 i2983_2_lut_3_lut_4_lut (.A(n9429), .B(n9428), .C(n9256), .D(n9257), 
         .Z(spi_bit_count_2__N_953[1])) /* synthesis lut_function=(!(A (B (C (D))+!B !(C (D)))+!A !(C (D)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i2983_2_lut_3_lut_4_lut.init = 16'h7888;
    LUT4 fpga_cs_n_N_339_I_0_1541_2_lut_4_lut (.A(rgb_values[91]), .B(rgb_values[83]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_347)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1541_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15821_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[3]), 
         .Z(spi_byte_count_15__N_1046)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15821_2_lut.init = 16'h1111;
    LUT4 i15509_2_lut_4_lut (.A(rgb_values[91]), .B(rgb_values[83]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_640)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15509_2_lut_4_lut.init = 16'h0035;
    FD1S3BX spi_frame_sequence_i11_5345_5346_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1386), .Q(n9557)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i11_5345_5346_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1942_2_lut_3_lut (.A(spi_expected_length[31]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1538)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1942_2_lut_3_lut.init = 16'h0808;
    LUT4 i15350_2_lut_3_lut (.A(spi_expected_length[31]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1602)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15350_2_lut_3_lut.init = 16'h0707;
    LUT4 i30_3_lut_4_lut_adj_39 (.A(amplitude_phase[6]), .B(\level_active[15] [0]), 
         .C(\level_active[15] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3379)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_39.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1540_2_lut_4_lut (.A(rgb_values[92]), .B(rgb_values[84]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_345)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1540_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15506_2_lut_4_lut (.A(rgb_values[92]), .B(rgb_values[84]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_637)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15506_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1943_2_lut_3_lut (.A(spi_expected_length[30]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1540)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1943_2_lut_3_lut.init = 16'h0808;
    LUT4 i15353_2_lut_3_lut (.A(spi_expected_length[30]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1637)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15353_2_lut_3_lut.init = 16'h0707;
    LUT4 fpga_cs_n_N_339_I_0_1761_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[4]), 
         .Z(spi_byte_count_15__N_984)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1761_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1539_2_lut_4_lut (.A(rgb_values[93]), .B(rgb_values[85]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_343)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1539_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15503_2_lut_4_lut (.A(rgb_values[93]), .B(rgb_values[85]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_634)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15503_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1944_2_lut_3_lut (.A(spi_expected_length[29]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1542)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1944_2_lut_3_lut.init = 16'h0808;
    LUT4 i15818_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[4]), 
         .Z(spi_byte_count_15__N_1043)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15818_2_lut.init = 16'h1111;
    LUT4 i15356_2_lut_3_lut (.A(spi_expected_length[29]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1640)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15356_2_lut_3_lut.init = 16'h0707;
    LUT4 fpga_cs_n_N_339_I_0_1538_2_lut_4_lut (.A(rgb_values[94]), .B(rgb_values[86]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_341)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1538_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15500_2_lut_4_lut (.A(rgb_values[94]), .B(rgb_values[86]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_631)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15500_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1945_2_lut_3_lut (.A(spi_expected_length[28]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1544)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1945_2_lut_3_lut.init = 16'h0808;
    LUT4 i15359_2_lut_3_lut (.A(spi_expected_length[28]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1643)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15359_2_lut_3_lut.init = 16'h0707;
    FD1S1D i5092 (.D(n21165), .CK(spi_expected_length_31__N_1598), .CD(spi_expected_length_31__N_1724), 
           .Q(n9304));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5092.GSR = "ENABLED";
    FD1S1D i5096 (.D(n21165), .CK(spi_expected_length_31__N_1596), .CD(spi_expected_length_31__N_1721), 
           .Q(n9308));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5096.GSR = "ENABLED";
    FD1S1D i5100 (.D(n21165), .CK(spi_expected_length_31__N_1594), .CD(spi_expected_length_31__N_1718), 
           .Q(n9312));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5100.GSR = "ENABLED";
    FD1S1D i5104 (.D(n21165), .CK(spi_expected_length_31__N_1592), .CD(spi_expected_length_31__N_1715), 
           .Q(n9316));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5104.GSR = "ENABLED";
    FD1S1D i5108 (.D(n21165), .CK(spi_expected_length_31__N_1590), .CD(spi_expected_length_31__N_1712), 
           .Q(n9320));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5108.GSR = "ENABLED";
    FD1S1D i5112 (.D(n21165), .CK(spi_expected_length_31__N_1588), .CD(spi_expected_length_31__N_1709), 
           .Q(n9324));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5112.GSR = "ENABLED";
    FD1S1D i5116 (.D(n21165), .CK(spi_expected_length_31__N_1586), .CD(spi_expected_length_31__N_1706), 
           .Q(n9328));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5116.GSR = "ENABLED";
    FD1S1D i5120 (.D(n21165), .CK(spi_expected_length_31__N_1584), .CD(spi_expected_length_31__N_1703), 
           .Q(n9332));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5120.GSR = "ENABLED";
    FD1S1D i5124 (.D(n21165), .CK(spi_expected_length_31__N_1582), .CD(spi_expected_length_31__N_1700), 
           .Q(n9336));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5124.GSR = "ENABLED";
    FD1S1D i5128 (.D(n21165), .CK(spi_expected_length_31__N_1580), .CD(spi_expected_length_31__N_1697), 
           .Q(n9340));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5128.GSR = "ENABLED";
    FD1S1D i5132 (.D(n21165), .CK(spi_expected_length_31__N_1578), .CD(spi_expected_length_31__N_1694), 
           .Q(n9344));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5132.GSR = "ENABLED";
    FD1S1D i5136 (.D(n21165), .CK(spi_expected_length_31__N_1576), .CD(spi_expected_length_31__N_1691), 
           .Q(n9348));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5136.GSR = "ENABLED";
    FD1S1D i5140 (.D(n21165), .CK(spi_expected_length_31__N_1574), .CD(spi_expected_length_31__N_1688), 
           .Q(n9352));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5140.GSR = "ENABLED";
    FD1S1D i5144 (.D(n21165), .CK(spi_expected_length_31__N_1572), .CD(spi_expected_length_31__N_1685), 
           .Q(n9356));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5144.GSR = "ENABLED";
    FD1S1D i5148 (.D(n21165), .CK(spi_expected_length_31__N_1570), .CD(spi_expected_length_31__N_1682), 
           .Q(n9360));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5148.GSR = "ENABLED";
    FD1S1D i5152 (.D(n21165), .CK(spi_expected_length_31__N_1568), .CD(spi_expected_length_31__N_1679), 
           .Q(n9364));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5152.GSR = "ENABLED";
    FD1S1D i5156 (.D(n21165), .CK(spi_expected_length_31__N_1566), .CD(spi_expected_length_31__N_1676), 
           .Q(n9368));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5156.GSR = "ENABLED";
    FD1S1D i5160 (.D(n21165), .CK(spi_expected_length_31__N_1564), .CD(spi_expected_length_31__N_1673), 
           .Q(n9372));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5160.GSR = "ENABLED";
    FD1S1D i5164 (.D(n21165), .CK(spi_expected_length_31__N_1562), .CD(spi_expected_length_31__N_1670), 
           .Q(n9376));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5164.GSR = "ENABLED";
    FD1S1D i5168 (.D(n21165), .CK(spi_expected_length_31__N_1560), .CD(spi_expected_length_31__N_1667), 
           .Q(n9380));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5168.GSR = "ENABLED";
    FD1S1D i5172 (.D(n21165), .CK(spi_expected_length_31__N_1558), .CD(spi_expected_length_31__N_1664), 
           .Q(n9384));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5172.GSR = "ENABLED";
    FD1S1D i5176 (.D(n21165), .CK(spi_expected_length_31__N_1556), .CD(spi_expected_length_31__N_1661), 
           .Q(n9388));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5176.GSR = "ENABLED";
    FD1S1D i5180 (.D(n21165), .CK(spi_expected_length_31__N_1554), .CD(spi_expected_length_31__N_1658), 
           .Q(n9392));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5180.GSR = "ENABLED";
    FD1S1D i5184 (.D(n21165), .CK(spi_expected_length_31__N_1552), .CD(spi_expected_length_31__N_1655), 
           .Q(n9396));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5184.GSR = "ENABLED";
    FD1S1D i5188 (.D(n21165), .CK(spi_expected_length_31__N_1550), .CD(spi_expected_length_31__N_1652), 
           .Q(n9400));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5188.GSR = "ENABLED";
    FD1S1D i5192 (.D(n21165), .CK(spi_expected_length_31__N_1548), .CD(spi_expected_length_31__N_1649), 
           .Q(n9404));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5192.GSR = "ENABLED";
    FD1S1D i5196 (.D(n21165), .CK(spi_expected_length_31__N_1546), .CD(spi_expected_length_31__N_1646), 
           .Q(n9408));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5196.GSR = "ENABLED";
    FD1S1D i5200 (.D(n21165), .CK(spi_expected_length_31__N_1544), .CD(spi_expected_length_31__N_1643), 
           .Q(n9412));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5200.GSR = "ENABLED";
    FD1S1D i5204 (.D(n21165), .CK(spi_expected_length_31__N_1542), .CD(spi_expected_length_31__N_1640), 
           .Q(n9416));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5204.GSR = "ENABLED";
    FD1S1D i5208 (.D(n21165), .CK(spi_expected_length_31__N_1540), .CD(spi_expected_length_31__N_1637), 
           .Q(n9420));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5208.GSR = "ENABLED";
    FD1S1D i5212 (.D(n21165), .CK(spi_expected_length_31__N_1538), .CD(spi_expected_length_31__N_1602), 
           .Q(n9424));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5212.GSR = "ENABLED";
    FD1S1D i5216 (.D(n21165), .CK(spi_bit_count_2__N_946), .CD(spi_bit_count_2__N_956), 
           .Q(n9428));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5216.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i3_5101_5102_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1594), .Q(n9313)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i3_5101_5102_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1946_2_lut_3_lut (.A(spi_expected_length[27]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1546)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1946_2_lut_3_lut.init = 16'h0808;
    FD1P3AX level_active_83___i84 (.D(n2315), .SP(fpga_clk_c_enable_373), 
            .CK(fpga_clk_c), .Q(\level_active[42] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i84.GSR = "ENABLED";
    FD1S3DX spi_expected_length_i2_5097_5098_reset (.D(n21165), .CK(spi1_sck_c), 
            .CD(spi_expected_length_31__N_1721), .Q(n9310)) /* synthesis lse_init_val=1 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i2_5097_5098_reset.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1760_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[5]), 
         .Z(spi_byte_count_15__N_982)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1760_2_lut.init = 16'h4444;
    FD1P3AX level_active_83___i83 (.D(n2316), .SP(fpga_clk_c_enable_373), 
            .CK(fpga_clk_c), .Q(\level_active[42] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i83.GSR = "ENABLED";
    FD1P3AX level_active_83___i82 (.D(n2315), .SP(fpga_clk_c_enable_372), 
            .CK(fpga_clk_c), .Q(\level_active[43] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i82.GSR = "ENABLED";
    LUT4 i15362_2_lut_3_lut (.A(spi_expected_length[27]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1646)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15362_2_lut_3_lut.init = 16'h0707;
    FD1P3AX level_active_83___i81 (.D(n2316), .SP(fpga_clk_c_enable_372), 
            .CK(fpga_clk_c), .Q(\level_active[43] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i81.GSR = "ENABLED";
    FD1P3AX level_active_83___i144 (.D(n2315), .SP(fpga_clk_c_enable_403), 
            .CK(fpga_clk_c), .Q(\level_active[12] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i144.GSR = "ENABLED";
    FD1P3AX level_active_83___i80 (.D(n2315), .SP(fpga_clk_c_enable_371), 
            .CK(fpga_clk_c), .Q(\level_active[44] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i80.GSR = "ENABLED";
    FD1P3AX level_active_83___i143 (.D(n2316), .SP(fpga_clk_c_enable_403), 
            .CK(fpga_clk_c), .Q(\level_active[12] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i143.GSR = "ENABLED";
    FD1P3AX level_active_83___i142 (.D(n2315), .SP(fpga_clk_c_enable_402), 
            .CK(fpga_clk_c), .Q(\level_active[13] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i142.GSR = "ENABLED";
    FD1P3AX level_active_83___i141 (.D(n2316), .SP(fpga_clk_c_enable_402), 
            .CK(fpga_clk_c), .Q(\level_active[13] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i141.GSR = "ENABLED";
    LUT4 i30_3_lut_4_lut_adj_40 (.A(amplitude_phase[6]), .B(\level_active[14] [0]), 
         .C(\level_active[14] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3448)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_40.init = 16'h40f4;
    FD1P3AX level_active_83___i140 (.D(n2315), .SP(fpga_clk_c_enable_401), 
            .CK(fpga_clk_c), .Q(\level_active[14] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i140.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1947_2_lut_3_lut (.A(spi_expected_length[26]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1548)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1947_2_lut_3_lut.init = 16'h0808;
    FD1P3AX level_active_83___i139 (.D(n2316), .SP(fpga_clk_c_enable_401), 
            .CK(fpga_clk_c), .Q(\level_active[14] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i139.GSR = "ENABLED";
    FD1P3AX level_active_83___i79 (.D(n2316), .SP(fpga_clk_c_enable_371), 
            .CK(fpga_clk_c), .Q(\level_active[44] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i79.GSR = "ENABLED";
    OB us_tx_pad_80 (.I(us_tx_c_80), .O(us_tx[80]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX level_active_83___i78 (.D(n2315), .SP(fpga_clk_c_enable_370), 
            .CK(fpga_clk_c), .Q(\level_active[45] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i78.GSR = "ENABLED";
    FD1P3AX level_active_83___i77 (.D(n2316), .SP(fpga_clk_c_enable_370), 
            .CK(fpga_clk_c), .Q(\level_active[45] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i77.GSR = "ENABLED";
    FD1P3AX level_active_83___i76 (.D(n2315), .SP(fpga_clk_c_enable_369), 
            .CK(fpga_clk_c), .Q(\level_active[46] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i76.GSR = "ENABLED";
    FD1P3AX level_active_83___i75 (.D(n2316), .SP(fpga_clk_c_enable_369), 
            .CK(fpga_clk_c), .Q(\level_active[46] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i75.GSR = "ENABLED";
    FD1P3AX level_active_83___i138 (.D(n2315), .SP(fpga_clk_c_enable_400), 
            .CK(fpga_clk_c), .Q(\level_active[15] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i138.GSR = "ENABLED";
    FD1P3AX level_active_83___i137 (.D(n2316), .SP(fpga_clk_c_enable_400), 
            .CK(fpga_clk_c), .Q(\level_active[15] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i137.GSR = "ENABLED";
    FD1P3AX level_active_83___i74 (.D(n2315), .SP(fpga_clk_c_enable_368), 
            .CK(fpga_clk_c), .Q(\level_active[47] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i74.GSR = "ENABLED";
    FD1P3AX level_active_83___i136 (.D(n2315), .SP(fpga_clk_c_enable_399), 
            .CK(fpga_clk_c), .Q(\level_active[16] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i136.GSR = "ENABLED";
    FD1P3AX level_active_83___i135 (.D(n2316), .SP(fpga_clk_c_enable_399), 
            .CK(fpga_clk_c), .Q(\level_active[16] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i135.GSR = "ENABLED";
    FD1P3DX rgb_values_i82_5661_5662_reset (.D(n9873), .SP(spi1_sck_c_enable_64), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_667), .Q(n9874)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i82_5661_5662_reset.GSR = "ENABLED";
    LUT4 i15365_2_lut_3_lut (.A(spi_expected_length[26]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1649)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15365_2_lut_3_lut.init = 16'h0707;
    FD1P3AX level_active_83___i73 (.D(n2316), .SP(fpga_clk_c_enable_368), 
            .CK(fpga_clk_c), .Q(\level_active[47] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i73.GSR = "ENABLED";
    LUT4 i30_3_lut_4_lut_adj_41 (.A(amplitude_phase[6]), .B(\level_active[13] [0]), 
         .C(\level_active[13] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3444)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_41.init = 16'h40f4;
    FD1S3BX spi_expected_length_i1_5093_5094_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1598), .Q(n9305)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i1_5093_5094_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i72 (.D(n2315), .SP(fpga_clk_c_enable_367), 
            .CK(fpga_clk_c), .Q(\level_active[48] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i72.GSR = "ENABLED";
    FD1P3AX level_active_83___i71 (.D(n2316), .SP(fpga_clk_c_enable_367), 
            .CK(fpga_clk_c), .Q(\level_active[48] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i71.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i30_5209_5210_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1540), .Q(n9421)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i30_5209_5210_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i134 (.D(n2315), .SP(fpga_clk_c_enable_398), 
            .CK(fpga_clk_c), .Q(\level_active[17] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i134.GSR = "ENABLED";
    FD1P3AX level_active_83___i70 (.D(n2315), .SP(fpga_clk_c_enable_366), 
            .CK(fpga_clk_c), .Q(\level_active[49] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i70.GSR = "ENABLED";
    FD1P3AX level_active_83___i133 (.D(n2316), .SP(fpga_clk_c_enable_398), 
            .CK(fpga_clk_c), .Q(\level_active[17] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i133.GSR = "ENABLED";
    FD1P3AX level_active_83___i69 (.D(n2316), .SP(fpga_clk_c_enable_366), 
            .CK(fpga_clk_c), .Q(\level_active[49] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i69.GSR = "ENABLED";
    FD1P3AX level_active_83___i132 (.D(n2315), .SP(fpga_clk_c_enable_397), 
            .CK(fpga_clk_c), .Q(\level_active[18] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i132.GSR = "ENABLED";
    FD1P3AX level_active_83___i68 (.D(n2315), .SP(fpga_clk_c_enable_365), 
            .CK(fpga_clk_c), .Q(\level_active[50] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i68.GSR = "ENABLED";
    FD1P3AX level_active_83___i131 (.D(n2316), .SP(fpga_clk_c_enable_397), 
            .CK(fpga_clk_c), .Q(\level_active[18] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i131.GSR = "ENABLED";
    FD1P3AX level_active_83___i67 (.D(n2316), .SP(fpga_clk_c_enable_365), 
            .CK(fpga_clk_c), .Q(\level_active[50] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i67.GSR = "ENABLED";
    FD1P3AX level_active_83___i130 (.D(n2315), .SP(fpga_clk_c_enable_396), 
            .CK(fpga_clk_c), .Q(\level_active[19] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i130.GSR = "ENABLED";
    FD1P3AX level_active_83___i66 (.D(n2315), .SP(fpga_clk_c_enable_364), 
            .CK(fpga_clk_c), .Q(\level_active[51] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i66.GSR = "ENABLED";
    FD1P3AX level_active_83___i129 (.D(n2316), .SP(fpga_clk_c_enable_396), 
            .CK(fpga_clk_c), .Q(\level_active[19] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i129.GSR = "ENABLED";
    FD1P3AX level_active_83___i65 (.D(n2316), .SP(fpga_clk_c_enable_364), 
            .CK(fpga_clk_c), .Q(\level_active[51] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i65.GSR = "ENABLED";
    FD1P3AX level_active_83___i128 (.D(n2315), .SP(fpga_clk_c_enable_395), 
            .CK(fpga_clk_c), .Q(\level_active[20] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i128.GSR = "ENABLED";
    FD1P3AX level_active_83___i64 (.D(n2315), .SP(fpga_clk_c_enable_363), 
            .CK(fpga_clk_c), .Q(\level_active[52] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i64.GSR = "ENABLED";
    FD1P3AX level_active_83___i127 (.D(n2316), .SP(fpga_clk_c_enable_395), 
            .CK(fpga_clk_c), .Q(\level_active[20] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i127.GSR = "ENABLED";
    FD1P3AX level_active_83___i63 (.D(n2316), .SP(fpga_clk_c_enable_363), 
            .CK(fpga_clk_c), .Q(\level_active[52] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i63.GSR = "ENABLED";
    FD1P3AX level_active_83___i126 (.D(n2315), .SP(fpga_clk_c_enable_394), 
            .CK(fpga_clk_c), .Q(\level_active[21] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i126.GSR = "ENABLED";
    FD1P3AX level_active_83___i62 (.D(n2315), .SP(fpga_clk_c_enable_362), 
            .CK(fpga_clk_c), .Q(\level_active[53] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i62.GSR = "ENABLED";
    FD1P3AX level_active_83___i125 (.D(n2316), .SP(fpga_clk_c_enable_394), 
            .CK(fpga_clk_c), .Q(\level_active[21] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i125.GSR = "ENABLED";
    FD1P3AX level_active_83___i61 (.D(n2316), .SP(fpga_clk_c_enable_362), 
            .CK(fpga_clk_c), .Q(\level_active[53] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i61.GSR = "ENABLED";
    FD1P3AX level_active_83___i124 (.D(n2315), .SP(fpga_clk_c_enable_393), 
            .CK(fpga_clk_c), .Q(\level_active[22] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i124.GSR = "ENABLED";
    FD1P3AX level_active_83___i60 (.D(n2315), .SP(fpga_clk_c_enable_361), 
            .CK(fpga_clk_c), .Q(\level_active[54] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i60.GSR = "ENABLED";
    FD1P3AX level_active_83___i123 (.D(n2316), .SP(fpga_clk_c_enable_393), 
            .CK(fpga_clk_c), .Q(\level_active[22] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i123.GSR = "ENABLED";
    FD1P3AX level_active_83___i59 (.D(n2316), .SP(fpga_clk_c_enable_361), 
            .CK(fpga_clk_c), .Q(\level_active[54] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i59.GSR = "ENABLED";
    FD1P3AX level_active_83___i122 (.D(n2315), .SP(fpga_clk_c_enable_392), 
            .CK(fpga_clk_c), .Q(\level_active[23] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i122.GSR = "ENABLED";
    FD1P3AX level_active_83___i58 (.D(n2315), .SP(fpga_clk_c_enable_360), 
            .CK(fpga_clk_c), .Q(\level_active[55] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i58.GSR = "ENABLED";
    FD1P3AX level_active_83___i121 (.D(n2316), .SP(fpga_clk_c_enable_392), 
            .CK(fpga_clk_c), .Q(\level_active[23] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i121.GSR = "ENABLED";
    FD1P3AX level_active_83___i57 (.D(n2316), .SP(fpga_clk_c_enable_360), 
            .CK(fpga_clk_c), .Q(\level_active[55] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i57.GSR = "ENABLED";
    FD1P3AX level_active_83___i120 (.D(n2315), .SP(fpga_clk_c_enable_391), 
            .CK(fpga_clk_c), .Q(\level_active[24] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i120.GSR = "ENABLED";
    FD1P3AX level_active_83___i56 (.D(n2315), .SP(fpga_clk_c_enable_359), 
            .CK(fpga_clk_c), .Q(\level_active[56] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i56.GSR = "ENABLED";
    FD1P3AX level_active_83___i119 (.D(n2316), .SP(fpga_clk_c_enable_391), 
            .CK(fpga_clk_c), .Q(\level_active[24] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i119.GSR = "ENABLED";
    FD1P3AX level_active_83___i55 (.D(n2316), .SP(fpga_clk_c_enable_359), 
            .CK(fpga_clk_c), .Q(\level_active[56] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i55.GSR = "ENABLED";
    FD1P3AX level_active_83___i118 (.D(n2315), .SP(fpga_clk_c_enable_390), 
            .CK(fpga_clk_c), .Q(\level_active[25] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i118.GSR = "ENABLED";
    FD1P3AX level_active_83___i54 (.D(n2315), .SP(fpga_clk_c_enable_358), 
            .CK(fpga_clk_c), .Q(\level_active[57] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i54.GSR = "ENABLED";
    FD1P3AX level_active_83___i117 (.D(n2316), .SP(fpga_clk_c_enable_390), 
            .CK(fpga_clk_c), .Q(\level_active[25] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i117.GSR = "ENABLED";
    FD1P3AX level_active_83___i53 (.D(n2316), .SP(fpga_clk_c_enable_358), 
            .CK(fpga_clk_c), .Q(\level_active[57] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i53.GSR = "ENABLED";
    FD1P3AX level_active_83___i116 (.D(n2315), .SP(fpga_clk_c_enable_389), 
            .CK(fpga_clk_c), .Q(\level_active[26] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i116.GSR = "ENABLED";
    FD1P3AX level_active_83___i52 (.D(n2315), .SP(fpga_clk_c_enable_357), 
            .CK(fpga_clk_c), .Q(\level_active[58] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i52.GSR = "ENABLED";
    FD1P3AX level_active_83___i115 (.D(n2316), .SP(fpga_clk_c_enable_389), 
            .CK(fpga_clk_c), .Q(\level_active[26] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i115.GSR = "ENABLED";
    FD1P3AX level_active_83___i51 (.D(n2316), .SP(fpga_clk_c_enable_357), 
            .CK(fpga_clk_c), .Q(\level_active[58] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i51.GSR = "ENABLED";
    FD1P3AX level_active_83___i114 (.D(n2315), .SP(fpga_clk_c_enable_388), 
            .CK(fpga_clk_c), .Q(\level_active[27] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i114.GSR = "ENABLED";
    FD1P3AX level_active_83___i50 (.D(n2315), .SP(fpga_clk_c_enable_356), 
            .CK(fpga_clk_c), .Q(\level_active[59] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i50.GSR = "ENABLED";
    FD1P3AX level_active_83___i113 (.D(n2316), .SP(fpga_clk_c_enable_388), 
            .CK(fpga_clk_c), .Q(\level_active[27] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i113.GSR = "ENABLED";
    FD1P3AX level_active_83___i49 (.D(n2316), .SP(fpga_clk_c_enable_356), 
            .CK(fpga_clk_c), .Q(\level_active[59] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i49.GSR = "ENABLED";
    FD1P3AX level_active_83___i112 (.D(n2315), .SP(fpga_clk_c_enable_387), 
            .CK(fpga_clk_c), .Q(\level_active[28] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i112.GSR = "ENABLED";
    FD1P3AX level_active_83___i48 (.D(n2315), .SP(fpga_clk_c_enable_355), 
            .CK(fpga_clk_c), .Q(\level_active[60] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i48.GSR = "ENABLED";
    FD1P3AX level_active_83___i111 (.D(n2316), .SP(fpga_clk_c_enable_387), 
            .CK(fpga_clk_c), .Q(\level_active[28] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i111.GSR = "ENABLED";
    FD1P3AX level_active_83___i47 (.D(n2316), .SP(fpga_clk_c_enable_355), 
            .CK(fpga_clk_c), .Q(\level_active[60] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i47.GSR = "ENABLED";
    FD1P3AX level_active_83___i110 (.D(n2315), .SP(fpga_clk_c_enable_386), 
            .CK(fpga_clk_c), .Q(\level_active[29] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i110.GSR = "ENABLED";
    FD1P3AX level_active_83___i46 (.D(n2315), .SP(fpga_clk_c_enable_354), 
            .CK(fpga_clk_c), .Q(\level_active[61] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i46.GSR = "ENABLED";
    FD1P3AX level_active_83___i109 (.D(n2316), .SP(fpga_clk_c_enable_386), 
            .CK(fpga_clk_c), .Q(\level_active[29] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i109.GSR = "ENABLED";
    FD1P3AX level_active_83___i45 (.D(n2316), .SP(fpga_clk_c_enable_354), 
            .CK(fpga_clk_c), .Q(\level_active[61] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i45.GSR = "ENABLED";
    FD1P3AX level_active_83___i108 (.D(n2315), .SP(fpga_clk_c_enable_385), 
            .CK(fpga_clk_c), .Q(\level_active[30] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i108.GSR = "ENABLED";
    FD1P3AX level_active_83___i44 (.D(n2315), .SP(fpga_clk_c_enable_353), 
            .CK(fpga_clk_c), .Q(\level_active[62] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i44.GSR = "ENABLED";
    FD1P3AX level_active_83___i107 (.D(n2316), .SP(fpga_clk_c_enable_385), 
            .CK(fpga_clk_c), .Q(\level_active[30] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i107.GSR = "ENABLED";
    FD1P3AX level_active_83___i43 (.D(n2316), .SP(fpga_clk_c_enable_353), 
            .CK(fpga_clk_c), .Q(\level_active[62] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i43.GSR = "ENABLED";
    FD1P3AX level_active_83___i106 (.D(n2315), .SP(fpga_clk_c_enable_384), 
            .CK(fpga_clk_c), .Q(\level_active[31] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i106.GSR = "ENABLED";
    FD1P3AX level_active_83___i42 (.D(n2315), .SP(fpga_clk_c_enable_352), 
            .CK(fpga_clk_c), .Q(\level_active[63] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i42.GSR = "ENABLED";
    FD1P3AX level_active_83___i105 (.D(n2316), .SP(fpga_clk_c_enable_384), 
            .CK(fpga_clk_c), .Q(\level_active[31] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i105.GSR = "ENABLED";
    FD1P3AX level_active_83___i41 (.D(n2316), .SP(fpga_clk_c_enable_352), 
            .CK(fpga_clk_c), .Q(\level_active[63] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i41.GSR = "ENABLED";
    FD1P3AX level_active_83___i104 (.D(n2315), .SP(fpga_clk_c_enable_383), 
            .CK(fpga_clk_c), .Q(\level_active[32] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i104.GSR = "ENABLED";
    FD1P3AX level_active_83___i40 (.D(n2315), .SP(fpga_clk_c_enable_351), 
            .CK(fpga_clk_c), .Q(\level_active[64] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i40.GSR = "ENABLED";
    FD1P3AX level_active_83___i103 (.D(n2316), .SP(fpga_clk_c_enable_383), 
            .CK(fpga_clk_c), .Q(\level_active[32] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i103.GSR = "ENABLED";
    FD1P3AX level_active_83___i39 (.D(n2316), .SP(fpga_clk_c_enable_351), 
            .CK(fpga_clk_c), .Q(\level_active[64] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i39.GSR = "ENABLED";
    FD1P3AX level_active_83___i102 (.D(n2315), .SP(fpga_clk_c_enable_382), 
            .CK(fpga_clk_c), .Q(\level_active[33] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i102.GSR = "ENABLED";
    FD1P3AX level_active_83___i38 (.D(n2315), .SP(fpga_clk_c_enable_350), 
            .CK(fpga_clk_c), .Q(\level_active[65] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i38.GSR = "ENABLED";
    FD1P3AX level_active_83___i101 (.D(n2316), .SP(fpga_clk_c_enable_382), 
            .CK(fpga_clk_c), .Q(\level_active[33] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i101.GSR = "ENABLED";
    FD1P3AX level_active_83___i37 (.D(n2316), .SP(fpga_clk_c_enable_350), 
            .CK(fpga_clk_c), .Q(\level_active[65] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i37.GSR = "ENABLED";
    FD1P3AX level_active_83___i100 (.D(n2315), .SP(fpga_clk_c_enable_381), 
            .CK(fpga_clk_c), .Q(\level_active[34] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i100.GSR = "ENABLED";
    FD1P3AX level_active_83___i36 (.D(n2315), .SP(fpga_clk_c_enable_349), 
            .CK(fpga_clk_c), .Q(\level_active[66] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i36.GSR = "ENABLED";
    FD1P3AX level_active_83___i99 (.D(n2316), .SP(fpga_clk_c_enable_381), 
            .CK(fpga_clk_c), .Q(\level_active[34] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i99.GSR = "ENABLED";
    FD1P3AX level_active_83___i35 (.D(n2316), .SP(fpga_clk_c_enable_349), 
            .CK(fpga_clk_c), .Q(\level_active[66] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i35.GSR = "ENABLED";
    FD1P3AX level_active_83___i98 (.D(n2315), .SP(fpga_clk_c_enable_380), 
            .CK(fpga_clk_c), .Q(\level_active[35] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i98.GSR = "ENABLED";
    FD1P3AX level_active_83___i34 (.D(n2315), .SP(fpga_clk_c_enable_348), 
            .CK(fpga_clk_c), .Q(\level_active[67] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i34.GSR = "ENABLED";
    FD1P3AX level_active_83___i97 (.D(n2316), .SP(fpga_clk_c_enable_380), 
            .CK(fpga_clk_c), .Q(\level_active[35] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i97.GSR = "ENABLED";
    FD1P3AX level_active_83___i33 (.D(n2316), .SP(fpga_clk_c_enable_348), 
            .CK(fpga_clk_c), .Q(\level_active[67] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i33.GSR = "ENABLED";
    FD1P3AX level_active_83___i96 (.D(n2315), .SP(fpga_clk_c_enable_379), 
            .CK(fpga_clk_c), .Q(\level_active[36] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i96.GSR = "ENABLED";
    FD1P3AX level_active_83___i95 (.D(n2316), .SP(fpga_clk_c_enable_379), 
            .CK(fpga_clk_c), .Q(\level_active[36] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i95.GSR = "ENABLED";
    FD1P3AX level_active_83___i94 (.D(n2315), .SP(fpga_clk_c_enable_378), 
            .CK(fpga_clk_c), .Q(\level_active[37] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i94.GSR = "ENABLED";
    LUT4 i30_3_lut_4_lut_adj_42 (.A(amplitude_phase[6]), .B(\level_active[12] [0]), 
         .C(\level_active[12] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3445)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_42.init = 16'h40f4;
    LUT4 i15815_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[5]), 
         .Z(spi_byte_count_15__N_1040)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15815_2_lut.init = 16'h1111;
    LUT4 i30_3_lut_4_lut_adj_43 (.A(amplitude_phase[6]), .B(\level_active[11] [0]), 
         .C(\level_active[11] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3369)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_43.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_44 (.A(amplitude_phase[6]), .B(\level_active[10] [0]), 
         .C(\level_active[10] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3371)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_44.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_45 (.A(amplitude_phase[6]), .B(\level_active[9] [0]), 
         .C(\level_active[9] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3398)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_45.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1759_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[6]), 
         .Z(spi_byte_count_15__N_980)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1759_2_lut.init = 16'h4444;
    LUT4 i15812_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[6]), 
         .Z(spi_byte_count_15__N_1037)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15812_2_lut.init = 16'h1111;
    LUT4 i30_3_lut_4_lut_adj_46 (.A(amplitude_phase[6]), .B(\level_active[8] [0]), 
         .C(\level_active[8] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3377)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_46.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_47 (.A(amplitude_phase[6]), .B(\level_active[7] [0]), 
         .C(\level_active[7] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3375)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_47.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1758_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[7]), 
         .Z(spi_byte_count_15__N_978)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1758_2_lut.init = 16'h4444;
    LUT4 i30_3_lut_4_lut_adj_48 (.A(amplitude_phase[6]), .B(\level_active[6] [0]), 
         .C(\level_active[6] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3382)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_48.init = 16'h40f4;
    LUT4 i15533_2_lut_4_lut (.A(rgb_values[83]), .B(rgb_values[75]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_664)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15533_2_lut_4_lut.init = 16'h0035;
    FD1P3AX level_active_83___i93 (.D(n2316), .SP(fpga_clk_c_enable_378), 
            .CK(fpga_clk_c), .Q(\level_active[37] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i93.GSR = "ENABLED";
    OB us_tx_pad_81 (.I(us_tx_c_81), .O(us_tx[81]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX level_active_83___i32 (.D(n2315), .SP(fpga_clk_c_enable_347), 
            .CK(fpga_clk_c), .Q(\level_active[68] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i32.GSR = "ENABLED";
    FD1P3AX level_active_83___i92 (.D(n2315), .SP(fpga_clk_c_enable_377), 
            .CK(fpga_clk_c), .Q(\level_active[38] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i92.GSR = "ENABLED";
    OB us_tx_pad_82 (.I(us_tx_c_82), .O(us_tx[82]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX level_active_83___i91 (.D(n2316), .SP(fpga_clk_c_enable_377), 
            .CK(fpga_clk_c), .Q(\level_active[38] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i91.GSR = "ENABLED";
    IB spi_mic_sck_pad (.I(spi_mic_sck), .O(spi_mic_sck_c));   // src/umh_fpga_top.v(17[24:35])
    IB spi_mic_cs_n_pad (.I(spi_mic_cs_n), .O(spi_mic_cs_n_c));   // src/umh_fpga_top.v(16[24:36])
    IB mic_data_1_pad (.I(mic_data_1), .O(mic_data_1_c));   // src/umh_fpga_top.v(15[24:34])
    IB mic_data_0_pad (.I(mic_data_0), .O(mic_data_0_c));   // src/umh_fpga_top.v(14[24:34])
    IB spi1_mosi_pad (.I(spi1_mosi), .O(spi1_mosi_c));   // src/umh_fpga_top.v(9[24:33])
    FD1P3AX level_active_83___i31 (.D(n2316), .SP(fpga_clk_c_enable_347), 
            .CK(fpga_clk_c), .Q(\level_active[68] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i31.GSR = "ENABLED";
    LUT4 i15809_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[7]), 
         .Z(spi_byte_count_15__N_1034)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15809_2_lut.init = 16'h1111;
    OB us_tx_pad_4 (.I(us_tx_c_4), .O(us_tx[4]));   // src/umh_fpga_top.v(11[24:29])
    IB spi1_sck_pad (.I(spi1_sck), .O(spi1_sck_c));   // src/umh_fpga_top.v(8[24:32])
    LUT4 fpga_cs_n_N_339_I_0_1549_2_lut_4_lut (.A(rgb_values[83]), .B(rgb_values[75]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_363)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1549_2_lut_4_lut.init = 16'h00ca;
    FD1S1D i5220 (.D(n21165), .CK(spi_bit_count_2__N_944), .CD(spi_bit_count_2__N_950), 
           .Q(n9432));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5220.GSR = "ENABLED";
    FD1S1D i5224 (.D(n21165), .CK(spi_rx_shift_6__N_921), .CD(spi_rx_shift_1__N_938), 
           .Q(n9436));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5224.GSR = "ENABLED";
    FD1S1D i5228 (.D(n21165), .CK(spi_rx_shift_6__N_920), .CD(spi_rx_shift_2__N_935), 
           .Q(n9440));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5228.GSR = "ENABLED";
    FD1S1D i5232 (.D(n21165), .CK(spi_rx_shift_6__N_919), .CD(spi_rx_shift_3__N_932), 
           .Q(n9444));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5232.GSR = "ENABLED";
    FD1S1D i5236 (.D(n21165), .CK(spi_rx_shift_6__N_918), .CD(spi_rx_shift_4__N_929), 
           .Q(n9448));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5236.GSR = "ENABLED";
    FD1S1D i5240 (.D(n21165), .CK(spi_rx_shift_6__N_917), .CD(spi_rx_shift_5__N_926), 
           .Q(n9452));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5240.GSR = "ENABLED";
    FD1S1D i5244 (.D(n21165), .CK(spi_rx_shift_6__N_916), .CD(spi_rx_shift_6__N_923), 
           .Q(n9456));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5244.GSR = "ENABLED";
    FD1S1D i5248 (.D(n21165), .CK(spi_command_7__N_1070), .CD(spi_command_7__N_1100), 
           .Q(n9460));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5248.GSR = "ENABLED";
    FD1S1D i5252 (.D(n21165), .CK(spi_command_7__N_1068), .CD(spi_command_7__N_1097), 
           .Q(n9464));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5252.GSR = "ENABLED";
    FD1S1D i5256 (.D(n21165), .CK(spi_command_7__N_1066), .CD(spi_command_7__N_1094), 
           .Q(n9468));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5256.GSR = "ENABLED";
    FD1S1D i5260 (.D(n21165), .CK(spi_command_7__N_1064), .CD(spi_command_7__N_1091), 
           .Q(n9472));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5260.GSR = "ENABLED";
    FD1S1D i5264 (.D(n21165), .CK(spi_command_7__N_1062), .CD(spi_command_7__N_1088), 
           .Q(n9476));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5264.GSR = "ENABLED";
    FD1S1D i5268 (.D(n21165), .CK(spi_command_7__N_1060), .CD(spi_command_7__N_1085), 
           .Q(n9480));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5268.GSR = "ENABLED";
    FD1S1D i5272 (.D(n21165), .CK(spi_command_7__N_1058), .CD(spi_command_7__N_1074), 
           .Q(n9484));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5272.GSR = "ENABLED";
    FD1S1D i5276 (.D(n21165), .CK(spi_version_7__N_1118), .CD(spi_version_7__N_1148), 
           .Q(n9488));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5276.GSR = "ENABLED";
    FD1S1D i5280 (.D(n21165), .CK(spi_version_7__N_1116), .CD(spi_version_7__N_1145), 
           .Q(n9492));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5280.GSR = "ENABLED";
    FD1S1D i5284 (.D(n21165), .CK(spi_version_7__N_1114), .CD(spi_version_7__N_1142), 
           .Q(n9496));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5284.GSR = "ENABLED";
    FD1S1D i5288 (.D(n21165), .CK(spi_version_7__N_1112), .CD(spi_version_7__N_1139), 
           .Q(n9500));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5288.GSR = "ENABLED";
    FD1S1D i5292 (.D(n21165), .CK(spi_version_7__N_1110), .CD(spi_version_7__N_1136), 
           .Q(n9504));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5292.GSR = "ENABLED";
    FD1S1D i5296 (.D(n21165), .CK(spi_version_7__N_1108), .CD(spi_version_7__N_1133), 
           .Q(n9508));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5296.GSR = "ENABLED";
    FD1S1D i5300 (.D(n21165), .CK(spi_version_7__N_1106), .CD(spi_version_7__N_1122), 
           .Q(n9512));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5300.GSR = "ENABLED";
    FD1S1D i5304 (.D(n21165), .CK(spi_frame_sequence_31__N_1406), .CD(spi_frame_sequence_31__N_1532), 
           .Q(n9516));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5304.GSR = "ENABLED";
    FD1S1D i5308 (.D(n21165), .CK(spi_frame_sequence_31__N_1404), .CD(spi_frame_sequence_31__N_1529), 
           .Q(n9520));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5308.GSR = "ENABLED";
    FD1S1D i5312 (.D(n21165), .CK(spi_frame_sequence_31__N_1402), .CD(spi_frame_sequence_31__N_1526), 
           .Q(n9524));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5312.GSR = "ENABLED";
    FD1S1D i5316 (.D(n21165), .CK(spi_frame_sequence_31__N_1400), .CD(spi_frame_sequence_31__N_1523), 
           .Q(n9528));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5316.GSR = "ENABLED";
    FD1S1D i5320 (.D(n21165), .CK(spi_frame_sequence_31__N_1398), .CD(spi_frame_sequence_31__N_1520), 
           .Q(n9532));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5320.GSR = "ENABLED";
    FD1S1D i5324 (.D(n21165), .CK(spi_frame_sequence_31__N_1396), .CD(spi_frame_sequence_31__N_1517), 
           .Q(n9536));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5324.GSR = "ENABLED";
    FD1S1D i5328 (.D(n21165), .CK(spi_frame_sequence_31__N_1394), .CD(spi_frame_sequence_31__N_1514), 
           .Q(n9540));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5328.GSR = "ENABLED";
    FD1S1D i5332 (.D(n21165), .CK(spi_frame_sequence_31__N_1392), .CD(spi_frame_sequence_31__N_1511), 
           .Q(n9544));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5332.GSR = "ENABLED";
    FD1S1D i5336 (.D(n21165), .CK(spi_frame_sequence_31__N_1390), .CD(spi_frame_sequence_31__N_1508), 
           .Q(n9548));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5336.GSR = "ENABLED";
    FD1S1D i5340 (.D(n21165), .CK(spi_frame_sequence_31__N_1388), .CD(spi_frame_sequence_31__N_1505), 
           .Q(n9552));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5340.GSR = "ENABLED";
    FD1S1D i5344 (.D(n21165), .CK(spi_frame_sequence_31__N_1386), .CD(spi_frame_sequence_31__N_1502), 
           .Q(n9556));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5344.GSR = "ENABLED";
    FD1S1D i5348 (.D(n21165), .CK(spi_frame_sequence_31__N_1384), .CD(spi_frame_sequence_31__N_1499), 
           .Q(n9560));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5348.GSR = "ENABLED";
    FD1S1D i5352 (.D(n21165), .CK(spi_frame_sequence_31__N_1382), .CD(spi_frame_sequence_31__N_1496), 
           .Q(n9564));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5352.GSR = "ENABLED";
    FD1S1D i5356 (.D(n21165), .CK(spi_frame_sequence_31__N_1380), .CD(spi_frame_sequence_31__N_1493), 
           .Q(n9568));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5356.GSR = "ENABLED";
    FD1S1D i5360 (.D(n21165), .CK(spi_frame_sequence_31__N_1378), .CD(spi_frame_sequence_31__N_1490), 
           .Q(n9572));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5360.GSR = "ENABLED";
    FD1S1D i5364 (.D(n21165), .CK(spi_frame_sequence_31__N_1376), .CD(spi_frame_sequence_31__N_1487), 
           .Q(n9576));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5364.GSR = "ENABLED";
    FD1S1D i5368 (.D(n21165), .CK(spi_frame_sequence_31__N_1374), .CD(spi_frame_sequence_31__N_1484), 
           .Q(n9580));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5368.GSR = "ENABLED";
    FD1S1D i5372 (.D(n21165), .CK(spi_frame_sequence_31__N_1372), .CD(spi_frame_sequence_31__N_1481), 
           .Q(n9584));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5372.GSR = "ENABLED";
    FD1S1D i5376 (.D(n21165), .CK(spi_frame_sequence_31__N_1370), .CD(spi_frame_sequence_31__N_1478), 
           .Q(n9588));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5376.GSR = "ENABLED";
    FD1S1D i5380 (.D(n21165), .CK(spi_frame_sequence_31__N_1368), .CD(spi_frame_sequence_31__N_1475), 
           .Q(n9592));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5380.GSR = "ENABLED";
    FD1S1D i5384 (.D(n21165), .CK(spi_frame_sequence_31__N_1366), .CD(spi_frame_sequence_31__N_1472), 
           .Q(n9596));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5384.GSR = "ENABLED";
    FD1S1D i5388 (.D(n21165), .CK(spi_frame_sequence_31__N_1364), .CD(spi_frame_sequence_31__N_1469), 
           .Q(n9600));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5388.GSR = "ENABLED";
    FD1S1D i5392 (.D(n21165), .CK(spi_frame_sequence_31__N_1362), .CD(spi_frame_sequence_31__N_1466), 
           .Q(n9604));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5392.GSR = "ENABLED";
    FD1S1D i5396 (.D(n21165), .CK(spi_frame_sequence_31__N_1360), .CD(spi_frame_sequence_31__N_1463), 
           .Q(n9608));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5396.GSR = "ENABLED";
    FD1S1D i5400 (.D(n21165), .CK(spi_frame_sequence_31__N_1358), .CD(spi_frame_sequence_31__N_1460), 
           .Q(n9612));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5400.GSR = "ENABLED";
    FD1S1D i5404 (.D(n21165), .CK(spi_frame_sequence_31__N_1356), .CD(spi_frame_sequence_31__N_1457), 
           .Q(n9616));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5404.GSR = "ENABLED";
    FD1S1D i5408 (.D(n21165), .CK(spi_frame_sequence_31__N_1354), .CD(spi_frame_sequence_31__N_1454), 
           .Q(n9620));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5408.GSR = "ENABLED";
    FD1S1D i5412 (.D(n21165), .CK(spi_frame_sequence_31__N_1352), .CD(spi_frame_sequence_31__N_1451), 
           .Q(n9624));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5412.GSR = "ENABLED";
    FD1S1D i5416 (.D(n21165), .CK(spi_frame_sequence_31__N_1350), .CD(spi_frame_sequence_31__N_1448), 
           .Q(n9628));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5416.GSR = "ENABLED";
    FD1S1D i5420 (.D(n21165), .CK(spi_frame_sequence_31__N_1348), .CD(spi_frame_sequence_31__N_1445), 
           .Q(n9632));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5420.GSR = "ENABLED";
    FD1S1D i5424 (.D(n21165), .CK(spi_frame_sequence_31__N_1346), .CD(spi_frame_sequence_31__N_1410), 
           .Q(n9636));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5424.GSR = "ENABLED";
    FD1S1D i5428 (.D(n21165), .CK(spi_update_flags_15__N_1182), .CD(spi_update_flags_15__N_1244), 
           .Q(n9640));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5428.GSR = "ENABLED";
    FD1S1D i5432 (.D(n21165), .CK(spi_extension_length_15__N_1278), .CD(spi_extension_length_15__N_1340), 
           .Q(n9644));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5432.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i26_5405_5406_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1356), .Q(n9617)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i26_5405_5406_set.GSR = "ENABLED";
    FD1S1D i5436 (.D(n21165), .CK(spi_extension_length_15__N_1276), .CD(spi_extension_length_15__N_1337), 
           .Q(n9648));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5436.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1757_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[8]), 
         .Z(spi_byte_count_15__N_976)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1757_2_lut.init = 16'h4444;
    FD1S1D i5440 (.D(n21165), .CK(spi_extension_length_15__N_1274), .CD(spi_extension_length_15__N_1334), 
           .Q(n9652));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5440.GSR = "ENABLED";
    FD1S1D i5444 (.D(n21165), .CK(spi_extension_length_15__N_1272), .CD(spi_extension_length_15__N_1331), 
           .Q(n9656));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5444.GSR = "ENABLED";
    FD1S1D i5448 (.D(n21165), .CK(spi_extension_length_15__N_1270), .CD(spi_extension_length_15__N_1328), 
           .Q(n9660));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5448.GSR = "ENABLED";
    FD1S1D i5452 (.D(n21165), .CK(spi_extension_length_15__N_1268), .CD(spi_extension_length_15__N_1325), 
           .Q(n9664));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5452.GSR = "ENABLED";
    FD1S1D i5456 (.D(n21165), .CK(spi_extension_length_15__N_1266), .CD(spi_extension_length_15__N_1322), 
           .Q(n9668));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5456.GSR = "ENABLED";
    FD1S1D i5460 (.D(n21165), .CK(spi_extension_length_15__N_1264), .CD(spi_extension_length_15__N_1319), 
           .Q(n9672));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5460.GSR = "ENABLED";
    FD1S1D i5464 (.D(n21165), .CK(spi_extension_length_15__N_1262), .CD(spi_extension_length_15__N_1316), 
           .Q(n9676));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5464.GSR = "ENABLED";
    FD1S1D i5468 (.D(n21165), .CK(spi_extension_length_15__N_1260), .CD(spi_extension_length_15__N_1313), 
           .Q(n9680));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5468.GSR = "ENABLED";
    FD1S1D i5472 (.D(n21165), .CK(spi_extension_length_15__N_1258), .CD(spi_extension_length_15__N_1310), 
           .Q(n9684));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5472.GSR = "ENABLED";
    FD1S1D i5476 (.D(n21165), .CK(spi_extension_length_15__N_1256), .CD(spi_extension_length_15__N_1307), 
           .Q(n9688));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5476.GSR = "ENABLED";
    FD1S1D i5480 (.D(n21165), .CK(spi_extension_length_15__N_1254), .CD(spi_extension_length_15__N_1304), 
           .Q(n9692));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5480.GSR = "ENABLED";
    FD1S1D i5484 (.D(n21165), .CK(spi_extension_length_15__N_1252), .CD(spi_extension_length_15__N_1301), 
           .Q(n9696));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5484.GSR = "ENABLED";
    FD1S1D i5488 (.D(n21165), .CK(spi_extension_length_15__N_1250), .CD(spi_extension_length_15__N_1282), 
           .Q(n9700));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5488.GSR = "ENABLED";
    FD1S1D i5492 (.D(n21165), .CK(spi_channel_index_6__N_1740), .CD(spi_channel_index_6__N_1766), 
           .Q(n9704));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5492.GSR = "ENABLED";
    FD1S1D i5496 (.D(n21165), .CK(spi_channel_index_6__N_1738), .CD(spi_channel_index_6__N_1763), 
           .Q(n9708));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5496.GSR = "ENABLED";
    FD1S1D i5500 (.D(n21165), .CK(spi_channel_index_6__N_1736), .CD(spi_channel_index_6__N_1760), 
           .Q(n9712));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5500.GSR = "ENABLED";
    FD1S1D i5504 (.D(n21165), .CK(spi_channel_index_6__N_1734), .CD(spi_channel_index_6__N_1757), 
           .Q(n9716));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5504.GSR = "ENABLED";
    FD1S1D i5508 (.D(n21165), .CK(spi_channel_index_6__N_1732), .CD(spi_channel_index_6__N_1754), 
           .Q(n9720));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5508.GSR = "ENABLED";
    FD1S1D i5512 (.D(n21165), .CK(spi_channel_index_6__N_1730), .CD(spi_channel_index_6__N_1744), 
           .Q(n9724));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5512.GSR = "ENABLED";
    FD1S1D i5516 (.D(n21165), .CK(spi_level_pending_7__N_1796), .CD(spi_level_pending_7__N_1826), 
           .Q(n9728));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5516.GSR = "ENABLED";
    FD1S1D i5520 (.D(n21165), .CK(spi_level_pending_7__N_1794), .CD(spi_level_pending_7__N_1823), 
           .Q(n9732));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5520.GSR = "ENABLED";
    FD1S1D i5524 (.D(n21165), .CK(spi_level_pending_7__N_1792), .CD(spi_level_pending_7__N_1820), 
           .Q(n9736));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5524.GSR = "ENABLED";
    FD1S1D i5528 (.D(n21165), .CK(spi_level_pending_7__N_1790), .CD(spi_level_pending_7__N_1817), 
           .Q(n9740));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5528.GSR = "ENABLED";
    FD1S1D i5532 (.D(n21165), .CK(spi_level_pending_7__N_1788), .CD(spi_level_pending_7__N_1814), 
           .Q(n9744));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5532.GSR = "ENABLED";
    FD1S1D i5536 (.D(n21165), .CK(spi_level_pending_7__N_1786), .CD(spi_level_pending_7__N_1811), 
           .Q(n9748));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5536.GSR = "ENABLED";
    FD1S1D i5540 (.D(n21165), .CK(spi_level_pending_7__N_1784), .CD(spi_level_pending_7__N_1800), 
           .Q(n9752));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5540.GSR = "ENABLED";
    FD1S1D i5544 (.D(n21165), .CK(spi_channel_field_1__N_1772), .CD(spi_channel_field_1__N_1776), 
           .Q(n9756));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5544.GSR = "ENABLED";
    FD1S1D i5548 (.D(n21165), .CK(spi_byte_count_15__N_990), .CD(spi_byte_count_15__N_1052), 
           .Q(n9760));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5548.GSR = "ENABLED";
    FD1S1D i5552 (.D(n21165), .CK(spi_byte_count_15__N_988), .CD(spi_byte_count_15__N_1049), 
           .Q(n9764));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5552.GSR = "ENABLED";
    FD1S1D i5556 (.D(n21165), .CK(spi_byte_count_15__N_986), .CD(spi_byte_count_15__N_1046), 
           .Q(n9768));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5556.GSR = "ENABLED";
    FD1S1D i5560 (.D(n21165), .CK(spi_byte_count_15__N_984), .CD(spi_byte_count_15__N_1043), 
           .Q(n9772));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5560.GSR = "ENABLED";
    FD1S1D i5564 (.D(n21165), .CK(spi_byte_count_15__N_982), .CD(spi_byte_count_15__N_1040), 
           .Q(n9776));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5564.GSR = "ENABLED";
    FD1S1D i5568 (.D(n21165), .CK(spi_byte_count_15__N_980), .CD(spi_byte_count_15__N_1037), 
           .Q(n9780));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5568.GSR = "ENABLED";
    FD1S1D i5572 (.D(n21165), .CK(spi_byte_count_15__N_978), .CD(spi_byte_count_15__N_1034), 
           .Q(n9784));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5572.GSR = "ENABLED";
    FD1S1D i5576 (.D(n21165), .CK(spi_byte_count_15__N_976), .CD(spi_byte_count_15__N_1031), 
           .Q(n9788));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5576.GSR = "ENABLED";
    FD1S1D i5580 (.D(n21165), .CK(spi_byte_count_15__N_974), .CD(spi_byte_count_15__N_1028), 
           .Q(n9792));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5580.GSR = "ENABLED";
    FD1S1D i5584 (.D(n21165), .CK(spi_byte_count_15__N_972), .CD(spi_byte_count_15__N_1025), 
           .Q(n9796));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5584.GSR = "ENABLED";
    FD1S1D i5588 (.D(n21165), .CK(spi_byte_count_15__N_970), .CD(spi_byte_count_15__N_1022), 
           .Q(n9800));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5588.GSR = "ENABLED";
    FD1S1D i5592 (.D(n21165), .CK(spi_byte_count_15__N_968), .CD(spi_byte_count_15__N_1019), 
           .Q(n9804));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5592.GSR = "ENABLED";
    FD1S1D i5596 (.D(n21165), .CK(spi_byte_count_15__N_966), .CD(spi_byte_count_15__N_1016), 
           .Q(n9808));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5596.GSR = "ENABLED";
    FD1S1D i5600 (.D(n21165), .CK(spi_byte_count_15__N_964), .CD(spi_byte_count_15__N_1013), 
           .Q(n9812));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5600.GSR = "ENABLED";
    FD1S1D i5604 (.D(n21165), .CK(spi_byte_count_15__N_962), .CD(spi_byte_count_15__N_994), 
           .Q(n9816));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5604.GSR = "ENABLED";
    FD1S1D i5608 (.D(n21165), .CK(rgb_values_95__N_337), .CD(rgb_values_95__N_531), 
           .Q(spi1_sck_c_enable_77));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5608.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i9_5337_5338_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1390), .Q(n9549)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i9_5337_5338_set.GSR = "ENABLED";
    LUT4 i30_3_lut_4_lut_adj_49 (.A(amplitude_phase[6]), .B(\level_active[1] [0]), 
         .C(\level_active[1] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3510)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_49.init = 16'h40f4;
    FD1S3BX spi_frame_sequence_i8_5333_5334_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1392), .Q(n9545)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i8_5333_5334_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i7_5329_5330_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1394), .Q(n9541)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i7_5329_5330_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i6_5325_5326_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1396), .Q(n9537)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i6_5325_5326_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i5_5321_5322_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1398), .Q(n9533)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i5_5321_5322_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i4_5317_5318_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1400), .Q(n9529)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i4_5317_5318_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i3_5313_5314_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1402), .Q(n9525)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i3_5313_5314_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i2_5309_5310_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1404), .Q(n9521)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i2_5309_5310_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i1_5305_5306_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1406), .Q(n9517)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i1_5305_5306_set.GSR = "ENABLED";
    FD1S3BX spi_version_i7_5301_5302_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1106), 
            .Q(n9513)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i7_5301_5302_set.GSR = "ENABLED";
    FD1S3BX spi_version_i6_5297_5298_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1108), 
            .Q(n9509)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i6_5297_5298_set.GSR = "ENABLED";
    FD1S3BX spi_version_i5_5293_5294_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1110), 
            .Q(n9505)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i5_5293_5294_set.GSR = "ENABLED";
    FD1S3BX spi_version_i4_5289_5290_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1112), 
            .Q(n9501)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i4_5289_5290_set.GSR = "ENABLED";
    FD1S3BX spi_version_i3_5285_5286_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1114), 
            .Q(n9497)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i3_5285_5286_set.GSR = "ENABLED";
    FD1S3BX spi_version_i2_5281_5282_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1116), 
            .Q(n9493)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i2_5281_5282_set.GSR = "ENABLED";
    FD1S3BX spi_version_i1_5277_5278_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1118), 
            .Q(n9489)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i1_5277_5278_set.GSR = "ENABLED";
    LUT4 i15806_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[8]), 
         .Z(spi_byte_count_15__N_1031)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15806_2_lut.init = 16'h1111;
    FD1S3BX spi_command_i7_5273_5274_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1058), 
            .Q(n9485)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i7_5273_5274_set.GSR = "ENABLED";
    FD1S3BX spi_command_i6_5269_5270_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1060), 
            .Q(n9481)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i6_5269_5270_set.GSR = "ENABLED";
    FD1S3BX spi_command_i5_5265_5266_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1062), 
            .Q(n9477)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i5_5265_5266_set.GSR = "ENABLED";
    FD1S3BX spi_command_i4_5261_5262_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1064), 
            .Q(n9473)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i4_5261_5262_set.GSR = "ENABLED";
    FD1S3BX spi_command_i3_5257_5258_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1066), 
            .Q(n9469)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i3_5257_5258_set.GSR = "ENABLED";
    FD1S3BX spi_command_i2_5253_5254_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1068), 
            .Q(n9465)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i2_5253_5254_set.GSR = "ENABLED";
    FD1S3BX spi_command_i1_5249_5250_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1070), 
            .Q(n9461)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i1_5249_5250_set.GSR = "ENABLED";
    FD1S3BX spi_rx_shift_i7_5245_5246_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_rx_shift_6__N_916), .Q(n9457)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_rx_shift_i7_5245_5246_set.GSR = "ENABLED";
    FD1S3BX spi_rx_shift_i6_5241_5242_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_rx_shift_6__N_917), .Q(n9453)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_rx_shift_i6_5241_5242_set.GSR = "ENABLED";
    FD1S3BX spi_rx_shift_i5_5237_5238_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_rx_shift_6__N_918), .Q(n9449)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_rx_shift_i5_5237_5238_set.GSR = "ENABLED";
    FD1S3BX spi_rx_shift_i4_5233_5234_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_rx_shift_6__N_919), .Q(n9445)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_rx_shift_i4_5233_5234_set.GSR = "ENABLED";
    FD1S3BX spi_rx_shift_i3_5229_5230_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_rx_shift_6__N_920), .Q(n9441)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_rx_shift_i3_5229_5230_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i30 (.D(n2315), .SP(fpga_clk_c_enable_346), 
            .CK(fpga_clk_c), .Q(\level_active[69] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i30.GSR = "ENABLED";
    IB fpga_cs_n_pad (.I(fpga_cs_n), .O(fpga_cs_n_c));   // src/umh_fpga_top.v(7[24:33])
    FD1P3AX level_active_83___i29 (.D(n2316), .SP(fpga_clk_c_enable_346), 
            .CK(fpga_clk_c), .Q(\level_active[69] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i29.GSR = "ENABLED";
    IB fpga_clk_pad (.I(fpga_clk), .O(fpga_clk_c));   // src/umh_fpga_top.v(6[24:32])
    FD1P3AX level_active_83___i28 (.D(n2315), .SP(fpga_clk_c_enable_345), 
            .CK(fpga_clk_c), .Q(\level_active[70] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i28.GSR = "ENABLED";
    FD1P3AX level_active_83___i27 (.D(n2316), .SP(fpga_clk_c_enable_345), 
            .CK(fpga_clk_c), .Q(\level_active[70] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i27.GSR = "ENABLED";
    PFUMX i11 (.BLUT(n18888), .ALUT(n11181), .C0(status_bit_index[6]), 
          .Z(n8));
    OB us_tx_pad_83 (.I(us_tx_c_83), .O(us_tx[83]));   // src/umh_fpga_top.v(11[24:29])
    OB spi_mic_miso_pad (.I(spi_mic_miso_c), .O(spi_mic_miso));   // src/umh_fpga_top.v(18[24:36])
    FD1S3BX spi_bit_count_i2_5221_5222_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_bit_count_2__N_944), .Q(n9433)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_bit_count_i2_5221_5222_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i26 (.D(n2315), .SP(fpga_clk_c_enable_344), 
            .CK(fpga_clk_c), .Q(\level_active[71] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i26.GSR = "ENABLED";
    OB spi1_miso_pad (.I(spi1_miso_c), .O(spi1_miso));   // src/umh_fpga_top.v(10[24:33])
    FD1P3AX level_active_83___i25 (.D(n2316), .SP(fpga_clk_c_enable_344), 
            .CK(fpga_clk_c), .Q(\level_active[71] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i25.GSR = "ENABLED";
    FD1P3AX level_active_83___i24 (.D(n2315), .SP(fpga_clk_c_enable_343), 
            .CK(fpga_clk_c), .Q(\level_active[72] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i24.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i83 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_188), 
            .CK(fpga_clk_c), .Q(phase_active[83])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i83.GSR = "ENABLED";
    FD1P3AX level_active_83___i23 (.D(n2316), .SP(fpga_clk_c_enable_343), 
            .CK(fpga_clk_c), .Q(\level_active[72] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i23.GSR = "ENABLED";
    FD1P3AX level_active_83___i22 (.D(n2315), .SP(fpga_clk_c_enable_342), 
            .CK(fpga_clk_c), .Q(\level_active[73] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i22.GSR = "ENABLED";
    FD1P3AX level_active_83___i21 (.D(n2316), .SP(fpga_clk_c_enable_342), 
            .CK(fpga_clk_c), .Q(\level_active[73] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i21.GSR = "ENABLED";
    FD1P3AX level_active_83___i20 (.D(n2315), .SP(fpga_clk_c_enable_341), 
            .CK(fpga_clk_c), .Q(\level_active[74] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i20.GSR = "ENABLED";
    OB mic_clk_pad (.I(mic_clk_c), .O(mic_clk));   // src/umh_fpga_top.v(13[24:31])
    OB rgb_data_pad (.I(rgb_data_c), .O(rgb_data));   // src/umh_fpga_top.v(12[24:32])
    OB us_tx_pad_0 (.I(us_tx_c_0), .O(us_tx[0]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_expected_length_i0_5089_5090_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1600), .Q(n9301)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i0_5089_5090_set.GSR = "ENABLED";
    OB us_tx_pad_5 (.I(us_tx_c_5), .O(us_tx[5]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_1 (.I(us_tx_c_1), .O(us_tx[1]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_expected_length_i28_5201_5202_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1544), .Q(n9413)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i28_5201_5202_set.GSR = "ENABLED";
    FD1S3BX spi_bit_count_i1_5217_5218_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_bit_count_2__N_946), .Q(n9429)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_bit_count_i1_5217_5218_set.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i0_5085_5086_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_992), .Q(n9297)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i0_5085_5086_set.GSR = "ENABLED";
    FD1P3DX rgb_values_i83_5657_5658_reset (.D(n9869), .SP(spi1_sck_c_enable_65), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_664), .Q(n9870)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i83_5657_5658_reset.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1756_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[9]), 
         .Z(spi_byte_count_15__N_974)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1756_2_lut.init = 16'h4444;
    FD1P3AX level_active_83___i19 (.D(n2316), .SP(fpga_clk_c_enable_341), 
            .CK(fpga_clk_c), .Q(\level_active[74] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i19.GSR = "ENABLED";
    FD1P3AX level_active_83___i18 (.D(n2315), .SP(fpga_clk_c_enable_340), 
            .CK(fpga_clk_c), .Q(\level_active[75] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i18.GSR = "ENABLED";
    OB us_tx_pad_6 (.I(us_tx_c_6), .O(us_tx[6]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_expected_length_i27_5197_5198_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1546), .Q(n9409)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i27_5197_5198_set.GSR = "ENABLED";
    OB us_tx_pad_7 (.I(us_tx_c_7), .O(us_tx[7]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_channel_field_i0_5081_5082_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_field_1__N_1774), .Q(n9293)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_field_i0_5081_5082_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i17 (.D(n2316), .SP(fpga_clk_c_enable_340), 
            .CK(fpga_clk_c), .Q(\level_active[75] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i17.GSR = "ENABLED";
    FD1P3AX level_active_83___i16 (.D(n2315), .SP(fpga_clk_c_enable_339), 
            .CK(fpga_clk_c), .Q(\level_active[76] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i16.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i31_5213_5214_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1538), .Q(n9425)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i31_5213_5214_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i15 (.D(n2316), .SP(fpga_clk_c_enable_339), 
            .CK(fpga_clk_c), .Q(\level_active[76] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i15.GSR = "ENABLED";
    FD1P3AX level_active_83___i14 (.D(n2315), .SP(fpga_clk_c_enable_338), 
            .CK(fpga_clk_c), .Q(\level_active[77] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i14.GSR = "ENABLED";
    FD1P3AX level_active_83___i13 (.D(n2316), .SP(fpga_clk_c_enable_338), 
            .CK(fpga_clk_c), .Q(\level_active[77] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i13.GSR = "ENABLED";
    FD1P3AX level_active_83___i12 (.D(n2315), .SP(fpga_clk_c_enable_337), 
            .CK(fpga_clk_c), .Q(\level_active[78] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i12.GSR = "ENABLED";
    FD1S3BX spi_level_pending_i0_5077_5078_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1798), .Q(n9289)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i0_5077_5078_set.GSR = "ENABLED";
    FD1S3BX spi_rx_shift_i2_5225_5226_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_rx_shift_6__N_921), .Q(n9437)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_rx_shift_i2_5225_5226_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i10_5341_5342_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1388), .Q(n9553)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i10_5341_5342_set.GSR = "ENABLED";
    FD1S3BX spi_channel_index_i0_5073_5074_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_index_6__N_1742), .Q(n9285)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_index_i0_5073_5074_set.GSR = "ENABLED";
    LUT4 i15803_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[9]), 
         .Z(spi_byte_count_15__N_1028)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15803_2_lut.init = 16'h1111;
    FD1P3AX level_active_83___i11 (.D(n2316), .SP(fpga_clk_c_enable_337), 
            .CK(fpga_clk_c), .Q(\level_active[78] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i11.GSR = "ENABLED";
    FD1P3AX level_active_83___i10 (.D(n2315), .SP(fpga_clk_c_enable_336), 
            .CK(fpga_clk_c), .Q(\level_active[79] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i10.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i29_5205_5206_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1542), .Q(n9417)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i29_5205_5206_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i9 (.D(n2316), .SP(fpga_clk_c_enable_336), 
            .CK(fpga_clk_c), .Q(\level_active[79] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i9.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i0_5069_5070_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1280), .Q(n9281)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i0_5069_5070_set.GSR = "ENABLED";
    FD1P3AX frame_toggle_spi_1504 (.D(frame_toggle_spi_N_2869), .SP(accepted_sequence_spi_31__N_2262), 
            .CK(fpga_cs_n_c), .Q(frame_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam frame_toggle_spi_1504.GSR = "ENABLED";
    FD1P3AX level_active_83___i8 (.D(n2315), .SP(fpga_clk_c_enable_335), 
            .CK(fpga_clk_c), .Q(\level_active[80] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i8.GSR = "ENABLED";
    FD1S3IX time_divider_2862__i0 (.D(n35_adj_3442), .CK(fpga_clk_c), .CD(time_divider_5__N_2269), 
            .Q(time_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862__i0.GSR = "ENABLED";
    FD1P3AX level_active_83___i7 (.D(n2316), .SP(fpga_clk_c_enable_335), 
            .CK(fpga_clk_c), .Q(\level_active[80] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i7.GSR = "ENABLED";
    OB us_tx_pad_8 (.I(us_tx_c_8), .O(us_tx[8]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_9 (.I(us_tx_c_9), .O(us_tx[9]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX level_active_83___i6 (.D(n2315), .SP(fpga_clk_c_enable_334), 
            .CK(fpga_clk_c), .Q(\level_active[81] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i6.GSR = "ENABLED";
    FD1P3AX level_active_83___i5 (.D(n2316), .SP(fpga_clk_c_enable_334), 
            .CK(fpga_clk_c), .Q(\level_active[81] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i5.GSR = "ENABLED";
    FD1P3AX level_active_83___i4 (.D(n2315), .SP(fpga_clk_c_enable_333), 
            .CK(fpga_clk_c), .Q(\level_active[82] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i4.GSR = "ENABLED";
    FD1P3AX level_active_83___i3 (.D(n2316), .SP(fpga_clk_c_enable_333), 
            .CK(fpga_clk_c), .Q(\level_active[82] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i3.GSR = "ENABLED";
    FD1P3AX level_active_83___i2 (.D(n2315), .SP(fpga_clk_c_enable_188), 
            .CK(fpga_clk_c), .Q(\level_active[83] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i2.GSR = "ENABLED";
    OB us_tx_pad_10 (.I(us_tx_c_10), .O(us_tx[10]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_11 (.I(us_tx_c_11), .O(us_tx[11]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_12 (.I(us_tx_c_12), .O(us_tx[12]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_13 (.I(us_tx_c_13), .O(us_tx[13]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_14 (.I(us_tx_c_14), .O(us_tx[14]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5612 (.D(n21165), .CK(rgb_values_95__N_341), .CD(rgb_values_95__N_631), 
           .Q(spi1_sck_c_enable_76));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5612.GSR = "ENABLED";
    FD1S1D i5616 (.D(n21165), .CK(rgb_values_95__N_343), .CD(rgb_values_95__N_634), 
           .Q(spi1_sck_c_enable_75));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5616.GSR = "ENABLED";
    FD1S1D i5620 (.D(n21165), .CK(rgb_values_95__N_345), .CD(rgb_values_95__N_637), 
           .Q(spi1_sck_c_enable_74));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5620.GSR = "ENABLED";
    OB us_tx_pad_15 (.I(us_tx_c_15), .O(us_tx[15]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5624 (.D(n21165), .CK(rgb_values_95__N_347), .CD(rgb_values_95__N_640), 
           .Q(spi1_sck_c_enable_73));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5624.GSR = "ENABLED";
    FD1S1D i5628 (.D(n21165), .CK(rgb_values_95__N_349), .CD(rgb_values_95__N_643), 
           .Q(spi1_sck_c_enable_72));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5628.GSR = "ENABLED";
    FD1S1D i5632 (.D(n21165), .CK(rgb_values_95__N_351), .CD(rgb_values_95__N_646), 
           .Q(spi1_sck_c_enable_71));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5632.GSR = "ENABLED";
    FD1S1D i5636 (.D(n21165), .CK(rgb_values_95__N_353), .CD(rgb_values_95__N_649), 
           .Q(spi1_sck_c_enable_70));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5636.GSR = "ENABLED";
    FD1S1D i5640 (.D(n21165), .CK(rgb_values_95__N_355), .CD(rgb_values_95__N_652), 
           .Q(spi1_sck_c_enable_69));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5640.GSR = "ENABLED";
    OB us_tx_pad_16 (.I(us_tx_c_16), .O(us_tx[16]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5644 (.D(n21165), .CK(rgb_values_95__N_357), .CD(rgb_values_95__N_655), 
           .Q(spi1_sck_c_enable_68));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5644.GSR = "ENABLED";
    FD1S1D i5648 (.D(n21165), .CK(rgb_values_95__N_359), .CD(rgb_values_95__N_658), 
           .Q(spi1_sck_c_enable_67));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5648.GSR = "ENABLED";
    FD1S1D i5652 (.D(n21165), .CK(rgb_values_95__N_361), .CD(rgb_values_95__N_661), 
           .Q(spi1_sck_c_enable_66));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5652.GSR = "ENABLED";
    FD1S1D i5656 (.D(n21165), .CK(rgb_values_95__N_363), .CD(rgb_values_95__N_664), 
           .Q(spi1_sck_c_enable_65));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5656.GSR = "ENABLED";
    FD1S1D i5660 (.D(n21165), .CK(rgb_values_95__N_365), .CD(rgb_values_95__N_667), 
           .Q(spi1_sck_c_enable_64));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5660.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2062_9 (.A0(n9785), .B0(n9784), .C0(GND_net), 
          .D0(GND_net), .A1(n9789), .B1(n9788), .C1(GND_net), .D1(GND_net), 
          .CIN(n17127), .COUT(n17128), .S0(spi_byte_count_15__N_997[7]), 
          .S1(spi_byte_count_15__N_997[8]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_9.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_9.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_9.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_9.INJECT1_1 = "NO";
    OB us_tx_pad_17 (.I(us_tx_c_17), .O(us_tx[17]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5664 (.D(n21165), .CK(rgb_values_95__N_367), .CD(rgb_values_95__N_670), 
           .Q(spi1_sck_c_enable_63));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5664.GSR = "ENABLED";
    FD1S1D i5668 (.D(n21165), .CK(rgb_values_95__N_369), .CD(rgb_values_95__N_673), 
           .Q(spi1_sck_c_enable_62));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5668.GSR = "ENABLED";
    FD1S1D i5672 (.D(n21165), .CK(rgb_values_95__N_371), .CD(rgb_values_95__N_676), 
           .Q(spi1_sck_c_enable_61));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5672.GSR = "ENABLED";
    FD1S1D i5676 (.D(n21165), .CK(rgb_values_95__N_373), .CD(rgb_values_95__N_679), 
           .Q(spi1_sck_c_enable_60));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5676.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2062_3 (.A0(n9761), .B0(n9760), .C0(GND_net), 
          .D0(GND_net), .A1(n9765), .B1(n9764), .C1(GND_net), .D1(GND_net), 
          .CIN(n17124), .COUT(n17125), .S0(spi_byte_count_15__N_997[1]), 
          .S1(spi_byte_count_15__N_997[2]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_3.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_3.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_3.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_3.INJECT1_1 = "NO";
    CCU2D spi_byte_count_15__I_0_2062_13 (.A0(n9801), .B0(n9800), .C0(GND_net), 
          .D0(GND_net), .A1(n9805), .B1(n9804), .C1(GND_net), .D1(GND_net), 
          .CIN(n17129), .COUT(n17130), .S0(spi_byte_count_15__N_997[11]), 
          .S1(spi_byte_count_15__N_997[12]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_13.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_13.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_13.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_13.INJECT1_1 = "NO";
    CCU2D spi_byte_count_15__I_0_2062_7 (.A0(n9777), .B0(n9776), .C0(GND_net), 
          .D0(GND_net), .A1(n9781), .B1(n9780), .C1(GND_net), .D1(GND_net), 
          .CIN(n17126), .COUT(n17127), .S0(spi_byte_count_15__N_997[5]), 
          .S1(spi_byte_count_15__N_997[6]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_7.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_7.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_7.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_7.INJECT1_1 = "NO";
    CCU2D spi_byte_count_15__I_0_2062_11 (.A0(n9793), .B0(n9792), .C0(GND_net), 
          .D0(GND_net), .A1(n9797), .B1(n9796), .C1(GND_net), .D1(GND_net), 
          .CIN(n17128), .COUT(n17129), .S0(spi_byte_count_15__N_997[9]), 
          .S1(spi_byte_count_15__N_997[10]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_11.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_11.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_11.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_11.INJECT1_1 = "NO";
    OB us_tx_pad_18 (.I(us_tx_c_18), .O(us_tx[18]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5680 (.D(n21165), .CK(rgb_values_95__N_375), .CD(rgb_values_95__N_682), 
           .Q(spi1_sck_c_enable_59));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5680.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2062_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n9297), .B1(n9296), .C1(spi_bit_count[2]), 
          .D1(n7166), .COUT(n17124), .S1(spi_byte_count_15__N_997[0]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_1.INIT0 = 16'hF000;
    defparam spi_byte_count_15__I_0_2062_1.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_1.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_1.INJECT1_1 = "NO";
    FD1S1D i5684 (.D(n21165), .CK(rgb_values_95__N_377), .CD(rgb_values_95__N_685), 
           .Q(spi1_sck_c_enable_58));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5684.GSR = "ENABLED";
    FD1S1D i5688 (.D(n21165), .CK(rgb_values_95__N_379), .CD(rgb_values_95__N_688), 
           .Q(spi1_sck_c_enable_57));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5688.GSR = "ENABLED";
    FD1P3DX rgb_values_i84_5653_5654_reset (.D(n9865), .SP(spi1_sck_c_enable_66), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_661), .Q(n9866)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i84_5653_5654_reset.GSR = "ENABLED";
    FD1S1D i5692 (.D(n21165), .CK(rgb_values_95__N_381), .CD(rgb_values_95__N_691), 
           .Q(spi1_sck_c_enable_56));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5692.GSR = "ENABLED";
    OB us_tx_pad_19 (.I(us_tx_c_19), .O(us_tx[19]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3DX rgb_values_i85_5649_5650_reset (.D(n9861), .SP(spi1_sck_c_enable_67), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_658), .Q(n9862)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i85_5649_5650_reset.GSR = "ENABLED";
    FD1S1D i5696 (.D(n21165), .CK(rgb_values_95__N_383), .CD(rgb_values_95__N_694), 
           .Q(spi1_sck_c_enable_55));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5696.GSR = "ENABLED";
    FD1S1D i5700 (.D(n21165), .CK(rgb_values_95__N_385), .CD(rgb_values_95__N_697), 
           .Q(spi1_sck_c_enable_54));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5700.GSR = "ENABLED";
    FD1S1D i5704 (.D(n21165), .CK(rgb_values_95__N_387), .CD(rgb_values_95__N_700), 
           .Q(spi1_sck_c_enable_53));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5704.GSR = "ENABLED";
    FD1S1D i5708 (.D(n21165), .CK(rgb_values_95__N_389), .CD(rgb_values_95__N_703), 
           .Q(spi1_sck_c_enable_52));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5708.GSR = "ENABLED";
    FD1P3DX rgb_values_i86_5645_5646_reset (.D(n9857), .SP(spi1_sck_c_enable_68), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_655), .Q(n9858)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i86_5645_5646_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i87_5641_5642_reset (.D(n9853), .SP(spi1_sck_c_enable_69), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_652), .Q(n9854)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i87_5641_5642_reset.GSR = "ENABLED";
    OB us_tx_pad_20 (.I(us_tx_c_20), .O(us_tx[20]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5712 (.D(n21165), .CK(rgb_values_95__N_391), .CD(rgb_values_95__N_706), 
           .Q(spi1_sck_c_enable_51));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5712.GSR = "ENABLED";
    FD1S1D i5716 (.D(n21165), .CK(rgb_values_95__N_393), .CD(rgb_values_95__N_709), 
           .Q(spi1_sck_c_enable_50));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5716.GSR = "ENABLED";
    FD1S1D i5720 (.D(n21165), .CK(rgb_values_95__N_395), .CD(rgb_values_95__N_712), 
           .Q(spi1_sck_c_enable_49));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5720.GSR = "ENABLED";
    FD1S1D i5724 (.D(n21165), .CK(rgb_values_95__N_397), .CD(rgb_values_95__N_715), 
           .Q(spi1_sck_c_enable_48));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5724.GSR = "ENABLED";
    FD1P3DX rgb_values_i88_5637_5638_reset (.D(n9849), .SP(spi1_sck_c_enable_70), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_649), .Q(n9850)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i88_5637_5638_reset.GSR = "ENABLED";
    FD1S1D i5728 (.D(n21165), .CK(rgb_values_95__N_399), .CD(rgb_values_95__N_718), 
           .Q(spi1_sck_c_enable_47));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5728.GSR = "ENABLED";
    OB us_tx_pad_21 (.I(us_tx_c_21), .O(us_tx[21]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5732 (.D(n21165), .CK(rgb_values_95__N_401), .CD(rgb_values_95__N_721), 
           .Q(spi1_sck_c_enable_46));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5732.GSR = "ENABLED";
    FD1S1D i5736 (.D(n21165), .CK(rgb_values_95__N_403), .CD(rgb_values_95__N_724), 
           .Q(spi1_sck_c_enable_45));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5736.GSR = "ENABLED";
    FD1S1D i5740 (.D(n21165), .CK(rgb_values_95__N_405), .CD(rgb_values_95__N_727), 
           .Q(spi1_sck_c_enable_44));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5740.GSR = "ENABLED";
    OB us_tx_pad_22 (.I(us_tx_c_22), .O(us_tx[22]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5744 (.D(n21165), .CK(rgb_values_95__N_407), .CD(rgb_values_95__N_730), 
           .Q(spi1_sck_c_enable_43));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5744.GSR = "ENABLED";
    FD1S1D i5748 (.D(n21165), .CK(rgb_values_95__N_409), .CD(rgb_values_95__N_733), 
           .Q(spi1_sck_c_enable_42));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5748.GSR = "ENABLED";
    FD1S1D i5752 (.D(n21165), .CK(rgb_values_95__N_411), .CD(rgb_values_95__N_736), 
           .Q(spi1_sck_c_enable_41));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5752.GSR = "ENABLED";
    OB us_tx_pad_23 (.I(us_tx_c_23), .O(us_tx[23]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5756 (.D(n21165), .CK(rgb_values_95__N_413), .CD(rgb_values_95__N_739), 
           .Q(spi1_sck_c_enable_40));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5756.GSR = "ENABLED";
    FD1S1D i5760 (.D(n21165), .CK(rgb_values_95__N_415), .CD(rgb_values_95__N_742), 
           .Q(spi1_sck_c_enable_39));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5760.GSR = "ENABLED";
    FD1S1D i5764 (.D(n21165), .CK(rgb_values_95__N_417), .CD(rgb_values_95__N_745), 
           .Q(spi1_sck_c_enable_38));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5764.GSR = "ENABLED";
    FD1S1D i5768 (.D(n21165), .CK(rgb_values_95__N_419), .CD(rgb_values_95__N_748), 
           .Q(spi1_sck_c_enable_37));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5768.GSR = "ENABLED";
    OB us_tx_pad_24 (.I(us_tx_c_24), .O(us_tx[24]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5772 (.D(n21165), .CK(rgb_values_95__N_421), .CD(rgb_values_95__N_751), 
           .Q(spi1_sck_c_enable_36));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5772.GSR = "ENABLED";
    FD1S1D i5776 (.D(n21165), .CK(rgb_values_95__N_423), .CD(rgb_values_95__N_754), 
           .Q(spi1_sck_c_enable_35));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5776.GSR = "ENABLED";
    FD1S1D i5780 (.D(n21165), .CK(rgb_values_95__N_425), .CD(rgb_values_95__N_757), 
           .Q(spi1_sck_c_enable_34));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5780.GSR = "ENABLED";
    FD1S1D i5784 (.D(n21165), .CK(rgb_values_95__N_427), .CD(rgb_values_95__N_760), 
           .Q(spi1_sck_c_enable_33));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5784.GSR = "ENABLED";
    FD1S1D i5788 (.D(n21165), .CK(rgb_values_95__N_429), .CD(rgb_values_95__N_763), 
           .Q(spi1_sck_c_enable_32));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5788.GSR = "ENABLED";
    OB us_tx_pad_25 (.I(us_tx_c_25), .O(us_tx[25]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5792 (.D(n21165), .CK(rgb_values_95__N_431), .CD(rgb_values_95__N_766), 
           .Q(spi1_sck_c_enable_31));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5792.GSR = "ENABLED";
    FD1S1D i5796 (.D(n21165), .CK(rgb_values_95__N_433), .CD(rgb_values_95__N_769), 
           .Q(spi1_sck_c_enable_30));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5796.GSR = "ENABLED";
    FD1S1D i5800 (.D(n21165), .CK(rgb_values_95__N_435), .CD(rgb_values_95__N_772), 
           .Q(spi1_sck_c_enable_29));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5800.GSR = "ENABLED";
    FD1S1D i5804 (.D(n21165), .CK(rgb_values_95__N_437), .CD(rgb_values_95__N_775), 
           .Q(spi1_sck_c_enable_28));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5804.GSR = "ENABLED";
    OB us_tx_pad_26 (.I(us_tx_c_26), .O(us_tx[26]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5808 (.D(n21165), .CK(rgb_values_95__N_439), .CD(rgb_values_95__N_778), 
           .Q(spi1_sck_c_enable_27));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5808.GSR = "ENABLED";
    FD1S1D i5812 (.D(n21165), .CK(rgb_values_95__N_441), .CD(rgb_values_95__N_781), 
           .Q(spi1_sck_c_enable_26));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5812.GSR = "ENABLED";
    FD1S1D i5816 (.D(n21165), .CK(rgb_values_95__N_443), .CD(rgb_values_95__N_784), 
           .Q(spi1_sck_c_enable_25));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5816.GSR = "ENABLED";
    FD1S1D i5820 (.D(n21165), .CK(rgb_values_95__N_445), .CD(rgb_values_95__N_787), 
           .Q(spi1_sck_c_enable_24));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5820.GSR = "ENABLED";
    OB us_tx_pad_27 (.I(us_tx_c_27), .O(us_tx[27]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5824 (.D(n21165), .CK(rgb_values_95__N_447), .CD(rgb_values_95__N_790), 
           .Q(spi1_sck_c_enable_23));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5824.GSR = "ENABLED";
    FD1S1D i5828 (.D(n21165), .CK(rgb_values_95__N_449), .CD(rgb_values_95__N_793), 
           .Q(spi1_sck_c_enable_22));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5828.GSR = "ENABLED";
    FD1S1D i5832 (.D(n21165), .CK(rgb_values_95__N_451), .CD(rgb_values_95__N_796), 
           .Q(spi1_sck_c_enable_21));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5832.GSR = "ENABLED";
    FD1S1D i5836 (.D(n21165), .CK(rgb_values_95__N_453), .CD(rgb_values_95__N_799), 
           .Q(spi1_sck_c_enable_20));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5836.GSR = "ENABLED";
    FD1S1D i5840 (.D(n21165), .CK(rgb_values_95__N_455), .CD(rgb_values_95__N_802), 
           .Q(spi1_sck_c_enable_19));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5840.GSR = "ENABLED";
    OB us_tx_pad_28 (.I(us_tx_c_28), .O(us_tx[28]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5844 (.D(n21165), .CK(rgb_values_95__N_457), .CD(rgb_values_95__N_805), 
           .Q(spi1_sck_c_enable_18));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5844.GSR = "ENABLED";
    FD1S1D i5848 (.D(n21165), .CK(rgb_values_95__N_459), .CD(rgb_values_95__N_808), 
           .Q(spi1_sck_c_enable_17));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5848.GSR = "ENABLED";
    FD1S1D i5852 (.D(n21165), .CK(rgb_values_95__N_461), .CD(rgb_values_95__N_811), 
           .Q(spi1_sck_c_enable_16));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5852.GSR = "ENABLED";
    FD1S1D i5856 (.D(n21165), .CK(rgb_values_95__N_463), .CD(rgb_values_95__N_814), 
           .Q(spi1_sck_c_enable_15));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5856.GSR = "ENABLED";
    OB us_tx_pad_29 (.I(us_tx_c_29), .O(us_tx[29]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5860 (.D(n21165), .CK(rgb_values_95__N_465), .CD(rgb_values_95__N_817), 
           .Q(spi1_sck_c_enable_14));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5860.GSR = "ENABLED";
    FD1S1D i5864 (.D(n21165), .CK(rgb_values_95__N_467), .CD(rgb_values_95__N_820), 
           .Q(spi1_sck_c_enable_13));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5864.GSR = "ENABLED";
    FD1S1D i5868 (.D(n21165), .CK(rgb_values_95__N_469), .CD(rgb_values_95__N_823), 
           .Q(spi1_sck_c_enable_12));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5868.GSR = "ENABLED";
    OB us_tx_pad_30 (.I(us_tx_c_30), .O(us_tx[30]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5872 (.D(n21165), .CK(rgb_values_95__N_471), .CD(rgb_values_95__N_826), 
           .Q(spi1_sck_c_enable_11));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5872.GSR = "ENABLED";
    FD1S1D i5876 (.D(n21165), .CK(rgb_values_95__N_473), .CD(rgb_values_95__N_829), 
           .Q(spi1_sck_c_enable_10));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5876.GSR = "ENABLED";
    FD1S1D i5880 (.D(n21165), .CK(rgb_values_95__N_475), .CD(rgb_values_95__N_832), 
           .Q(spi1_sck_c_enable_9));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5880.GSR = "ENABLED";
    OB us_tx_pad_31 (.I(us_tx_c_31), .O(us_tx[31]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5884 (.D(n21165), .CK(rgb_values_95__N_477), .CD(rgb_values_95__N_835), 
           .Q(spi1_sck_c_enable_8));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5884.GSR = "ENABLED";
    FD1S1D i5888 (.D(n21165), .CK(rgb_values_95__N_479), .CD(rgb_values_95__N_838), 
           .Q(spi1_sck_c_enable_7));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5888.GSR = "ENABLED";
    FD1S1D i5892 (.D(n21165), .CK(rgb_values_95__N_481), .CD(rgb_values_95__N_841), 
           .Q(spi1_sck_c_enable_6));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5892.GSR = "ENABLED";
    FD1S1D i5896 (.D(n21165), .CK(rgb_values_95__N_483), .CD(rgb_values_95__N_844), 
           .Q(spi1_sck_c_enable_5));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5896.GSR = "ENABLED";
    OB us_tx_pad_32 (.I(us_tx_c_32), .O(us_tx[32]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5900 (.D(n21165), .CK(rgb_values_95__N_485), .CD(rgb_values_95__N_847), 
           .Q(spi1_sck_c_enable_4));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5900.GSR = "ENABLED";
    FD1S1D i5904 (.D(n21165), .CK(rgb_values_95__N_487), .CD(rgb_values_95__N_850), 
           .Q(spi1_sck_c_enable_3));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5904.GSR = "ENABLED";
    FD1S1D i5908 (.D(n21165), .CK(rgb_values_95__N_489), .CD(rgb_values_95__N_853), 
           .Q(spi1_sck_c_enable_2));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5908.GSR = "ENABLED";
    FD1S1D i5912 (.D(n21165), .CK(rgb_values_95__N_491), .CD(rgb_values_95__N_856), 
           .Q(spi1_sck_c_enable_1));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5912.GSR = "ENABLED";
    OB us_tx_pad_33 (.I(us_tx_c_33), .O(us_tx[33]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5916 (.D(n21165), .CK(rgb_values_95__N_493), .CD(rgb_values_95__N_859), 
           .Q(spi1_sck_c_enable_192));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5916.GSR = "ENABLED";
    FD1S1D i5920 (.D(n21165), .CK(rgb_values_95__N_495), .CD(rgb_values_95__N_862), 
           .Q(spi1_sck_c_enable_190));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5920.GSR = "ENABLED";
    FD1S1D i5924 (.D(n21165), .CK(rgb_values_95__N_497), .CD(rgb_values_95__N_865), 
           .Q(spi1_sck_c_enable_189));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5924.GSR = "ENABLED";
    OB us_tx_pad_34 (.I(us_tx_c_34), .O(us_tx[34]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5928 (.D(n21165), .CK(rgb_values_95__N_499), .CD(rgb_values_95__N_868), 
           .Q(spi1_sck_c_enable_188));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5928.GSR = "ENABLED";
    FD1S1D i5932 (.D(n21165), .CK(rgb_values_95__N_501), .CD(rgb_values_95__N_871), 
           .Q(spi1_sck_c_enable_187));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5932.GSR = "ENABLED";
    FD1P3DX rgb_values_i89_5633_5634_reset (.D(n9845), .SP(spi1_sck_c_enable_71), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_646), .Q(n9846)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i89_5633_5634_reset.GSR = "ENABLED";
    FD1S1D i5936 (.D(n21165), .CK(rgb_values_95__N_503), .CD(rgb_values_95__N_874), 
           .Q(spi1_sck_c_enable_186));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5936.GSR = "ENABLED";
    OB us_tx_pad_35 (.I(us_tx_c_35), .O(us_tx[35]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3DX rgb_values_i90_5629_5630_reset (.D(n9841), .SP(spi1_sck_c_enable_72), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_643), .Q(n9842)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i90_5629_5630_reset.GSR = "ENABLED";
    FD1S1D i5940 (.D(n21165), .CK(rgb_values_95__N_505), .CD(rgb_values_95__N_877), 
           .Q(spi1_sck_c_enable_185));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5940.GSR = "ENABLED";
    FD1S1D i5944 (.D(n21165), .CK(rgb_values_95__N_507), .CD(rgb_values_95__N_880), 
           .Q(spi1_sck_c_enable_184));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5944.GSR = "ENABLED";
    FD1S1D i5948 (.D(n21165), .CK(rgb_values_95__N_509), .CD(rgb_values_95__N_883), 
           .Q(spi1_sck_c_enable_183));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5948.GSR = "ENABLED";
    FD1P3DX rgb_values_i91_5625_5626_reset (.D(n9837), .SP(spi1_sck_c_enable_73), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_640), .Q(n9838)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i91_5625_5626_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i92_5621_5622_reset (.D(n9833), .SP(spi1_sck_c_enable_74), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_637), .Q(n9834)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i92_5621_5622_reset.GSR = "ENABLED";
    OB us_tx_pad_36 (.I(us_tx_c_36), .O(us_tx[36]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5952 (.D(n21165), .CK(rgb_values_95__N_511), .CD(rgb_values_95__N_886), 
           .Q(spi1_sck_c_enable_182));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5952.GSR = "ENABLED";
    FD1S1D i5956 (.D(n21165), .CK(rgb_values_95__N_513), .CD(rgb_values_95__N_889), 
           .Q(spi1_sck_c_enable_181));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5956.GSR = "ENABLED";
    FD1S1D i5960 (.D(n21165), .CK(rgb_values_95__N_515), .CD(rgb_values_95__N_892), 
           .Q(spi1_sck_c_enable_180));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5960.GSR = "ENABLED";
    FD1P3DX rgb_values_i93_5617_5618_reset (.D(n9829), .SP(spi1_sck_c_enable_75), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_634), .Q(n9830)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i93_5617_5618_reset.GSR = "ENABLED";
    OB us_tx_pad_37 (.I(us_tx_c_37), .O(us_tx[37]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5964 (.D(n21165), .CK(rgb_values_95__N_517), .CD(rgb_values_95__N_895), 
           .Q(spi1_sck_c_enable_179));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5964.GSR = "ENABLED";
    FD1S1D i5968 (.D(n21165), .CK(rgb_values_95__N_519), .CD(rgb_values_95__N_898), 
           .Q(spi1_sck_c_enable_178));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5968.GSR = "ENABLED";
    FD1P3DX rgb_values_i94_5613_5614_reset (.D(n9825), .SP(spi1_sck_c_enable_76), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_631), .Q(n9826)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i94_5613_5614_reset.GSR = "ENABLED";
    OB us_tx_pad_38 (.I(us_tx_c_38), .O(us_tx[38]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5972 (.D(n21165), .CK(rgb_values_95__N_521), .CD(rgb_values_95__N_901), 
           .Q(spi1_sck_c_enable_177));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5972.GSR = "ENABLED";
    FD1S1D i5976 (.D(n21165), .CK(rgb_values_95__N_523), .CD(rgb_values_95__N_904), 
           .Q(spi1_sck_c_enable_176));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5976.GSR = "ENABLED";
    FD1S1D i5980 (.D(n21165), .CK(rgb_values_95__N_525), .CD(rgb_values_95__N_907), 
           .Q(spi1_sck_c_enable_175));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5980.GSR = "ENABLED";
    OB us_tx_pad_39 (.I(us_tx_c_39), .O(us_tx[39]));   // src/umh_fpga_top.v(11[24:29])
    FD1S1D i5984 (.D(n21165), .CK(rgb_values_95__N_527), .CD(rgb_values_95__N_910), 
           .Q(spi1_sck_c_enable_121));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5984.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i31 (.D(mic_shift_0[14]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[31])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i31.GSR = "ENABLED";
    GSR GSR_INST (.GSR(VCC_net));
    LUT4 fpga_cs_n_N_339_I_0_1755_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[10]), 
         .Z(spi_byte_count_15__N_972)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1755_2_lut.init = 16'h4444;
    OB us_tx_pad_40 (.I(us_tx_c_40), .O(us_tx[40]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i30 (.D(mic_shift_0[13]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[30])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i30.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i29 (.D(mic_shift_0[12]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[29])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i29.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i28 (.D(mic_shift_0[11]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[28])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i28.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i27 (.D(mic_shift_0[10]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[27])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i27.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i26 (.D(mic_shift_0[9]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[26])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i26.GSR = "ENABLED";
    OB us_tx_pad_41 (.I(us_tx_c_41), .O(us_tx[41]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i25 (.D(mic_shift_0[8]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[25])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i25.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i24 (.D(mic_shift_0[7]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[24])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i24.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i23 (.D(mic_shift_0[6]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[23])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i23.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i22 (.D(mic_shift_0[5]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[22])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i22.GSR = "ENABLED";
    OB us_tx_pad_42 (.I(us_tx_c_42), .O(us_tx[42]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i21 (.D(mic_shift_0[4]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[21])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i21.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i20 (.D(mic_shift_0[3]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[20])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i20.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i25_5401_5402_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1358), .Q(n9613)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i25_5401_5402_set.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i19 (.D(mic_shift_0[2]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[19])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i19.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i18 (.D(mic_shift_0[1]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[18])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i18.GSR = "ENABLED";
    LUT4 i15800_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[10]), 
         .Z(spi_byte_count_15__N_1025)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15800_2_lut.init = 16'h1111;
    OB us_tx_pad_43 (.I(us_tx_c_43), .O(us_tx[43]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i17 (.D(mic_shift_0[0]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[17])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i17.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i16 (.D(mic_data_0_c), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[16])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i16.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i15 (.D(mic_shift_1[14]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[15])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i15.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i14 (.D(mic_shift_1[13]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[14])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i14.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i13 (.D(mic_shift_1[12]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[13])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i13.GSR = "ENABLED";
    OB us_tx_pad_44 (.I(us_tx_c_44), .O(us_tx[44]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i12 (.D(mic_shift_1[11]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[12])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i12.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i11 (.D(mic_shift_1[10]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[11])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i11.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i10 (.D(mic_shift_1[9]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[10])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i10.GSR = "ENABLED";
    FD1P3DX rgb_values_i95_5609_5610_reset (.D(n9821), .SP(spi1_sck_c_enable_77), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_531), .Q(n9822)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i95_5609_5610_reset.GSR = "ENABLED";
    FD1S3AX amplitude_phase_2864__i0 (.D(n45), .CK(fpga_clk_c), .Q(n8_adj_3412)) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i0.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i9 (.D(mic_shift_1[8]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[9])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i9.GSR = "ENABLED";
    OB us_tx_pad_45 (.I(us_tx_c_45), .O(us_tx[45]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i8 (.D(mic_shift_1[7]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[8])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i8.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i7 (.D(mic_shift_1[6]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[7])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i7.GSR = "ENABLED";
    FD1P3AX mic_sample_count_2865__i0 (.D(n17436), .SP(mic_divider_6__N_2705), 
            .CK(fpga_clk_c), .Q(mic_sample_count[0]));   // src/umh_fpga_top.v(328[33:56])
    defparam mic_sample_count_2865__i0.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i6 (.D(mic_shift_1[5]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[6])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i6.GSR = "ENABLED";
    OB us_tx_pad_46 (.I(us_tx_c_46), .O(us_tx[46]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i5 (.D(mic_shift_1[4]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[5])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i5.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i4 (.D(mic_shift_1[3]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[4])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i4.GSR = "ENABLED";
    FD1S3IX mic_divider_2866__i0 (.D(n40_adj_3387), .CK(fpga_clk_c), .CD(mic_divider_6__N_2705), 
            .Q(mic_divider[0])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866__i0.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i3 (.D(mic_shift_1[2]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[3])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i3.GSR = "ENABLED";
    OB us_tx_pad_47 (.I(us_tx_c_47), .O(us_tx[47]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_latest_i0_i2 (.D(mic_shift_1[1]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[2])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i2.GSR = "ENABLED";
    FD1P3AX mic_latest_i0_i1 (.D(mic_shift_1[0]), .SP(fpga_clk_c_enable_246), 
            .CK(fpga_clk_c), .Q(mic_latest[1])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_latest_i0_i1.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i15 (.D(mic_shift_1[13]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[14])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i15.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i15_5605_5606_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_962), .Q(n9817)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i15_5605_5606_set.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i14 (.D(mic_shift_1[12]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[13])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i14.GSR = "ENABLED";
    OB us_tx_pad_48 (.I(us_tx_c_48), .O(us_tx[48]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_1__i13 (.D(mic_shift_1[11]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[12])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i13.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i12 (.D(mic_shift_1[10]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[11])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i12.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i11 (.D(mic_shift_1[9]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[10])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i11.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i10 (.D(mic_shift_1[8]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[9])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i10.GSR = "ENABLED";
    OB us_tx_pad_49 (.I(us_tx_c_49), .O(us_tx[49]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_1__i9 (.D(mic_shift_1[7]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[8])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i9.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i8 (.D(mic_shift_1[6]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[7])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i8.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i7 (.D(mic_shift_1[5]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[6])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i7.GSR = "ENABLED";
    OB us_tx_pad_50 (.I(us_tx_c_50), .O(us_tx[50]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_1__i6 (.D(mic_shift_1[4]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[5])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i6.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i5 (.D(mic_shift_1[3]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[4])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i5.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i4 (.D(mic_shift_1[2]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[3])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i4.GSR = "ENABLED";
    OB us_tx_pad_51 (.I(us_tx_c_51), .O(us_tx[51]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_1__i3 (.D(mic_shift_1[1]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[2])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i3.GSR = "ENABLED";
    FD1P3AX mic_shift_1__i2 (.D(mic_shift_1[0]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_1[1])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_1__i2.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1754_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[11]), 
         .Z(spi_byte_count_15__N_970)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1754_2_lut.init = 16'h4444;
    FD1P3AX mic_shift_0__i15 (.D(mic_shift_0[13]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[14])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i15.GSR = "ENABLED";
    LUT4 i15797_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[11]), 
         .Z(spi_byte_count_15__N_1022)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15797_2_lut.init = 16'h1111;
    OB us_tx_pad_52 (.I(us_tx_c_52), .O(us_tx[52]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_0__i14 (.D(mic_shift_0[12]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[13])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i14.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i13 (.D(mic_shift_0[11]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[12])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i13.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i12 (.D(mic_shift_0[10]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[11])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i12.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i14_5601_5602_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_964), .Q(n9813)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i14_5601_5602_set.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i11 (.D(mic_shift_0[9]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[10])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i11.GSR = "ENABLED";
    OB us_tx_pad_53 (.I(us_tx_c_53), .O(us_tx[53]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_0__i10 (.D(mic_shift_0[8]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[9])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i10.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i9 (.D(mic_shift_0[7]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[8])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i9.GSR = "ENABLED";
    OB us_tx_pad_54 (.I(us_tx_c_54), .O(us_tx[54]));   // src/umh_fpga_top.v(11[24:29])
    LUT4 fpga_cs_n_N_339_I_0_1753_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[12]), 
         .Z(spi_byte_count_15__N_968)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1753_2_lut.init = 16'h4444;
    LUT4 i15794_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[12]), 
         .Z(spi_byte_count_15__N_1019)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15794_2_lut.init = 16'h1111;
    FD1P3AX mic_shift_0__i8 (.D(mic_shift_0[6]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[7])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i8.GSR = "ENABLED";
    OB us_tx_pad_55 (.I(us_tx_c_55), .O(us_tx[55]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_0__i7 (.D(mic_shift_0[5]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[6])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i7.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i6 (.D(mic_shift_0[4]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[5])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i6.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i5 (.D(mic_shift_0[3]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[4])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i5.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i4 (.D(mic_shift_0[2]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[3])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i4.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1752_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[13]), 
         .Z(spi_byte_count_15__N_966)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1752_2_lut.init = 16'h4444;
    OB us_tx_pad_56 (.I(us_tx_c_56), .O(us_tx[56]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX mic_shift_0__i3 (.D(mic_shift_0[1]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[2])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i3.GSR = "ENABLED";
    FD1S3DX status_bit_index_2860__i0 (.D(n40_adj_3424), .CK(spi1_sck_N_1872), 
            .CD(fpga_cs_n_c), .Q(status_bit_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860__i0.GSR = "ENABLED";
    FD1P3AX mic_shift_0__i2 (.D(mic_shift_0[0]), .SP(fpga_clk_c_enable_274), 
            .CK(fpga_clk_c), .Q(mic_shift_0[1])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam mic_shift_0__i2.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i31 (.D(accepted_sequence_spi[31]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[31])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i31.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i30 (.D(accepted_sequence_spi[30]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[30])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i30.GSR = "ENABLED";
    OB us_tx_pad_57 (.I(us_tx_c_57), .O(us_tx[57]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i29 (.D(accepted_sequence_spi[29]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[29])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i29.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i0 (.D(n165), .CK(fpga_clk_c), .Q(fpga_time[0])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i0.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i28 (.D(accepted_sequence_spi[28]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[28])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i28.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i27 (.D(accepted_sequence_spi[27]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[27])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i27.GSR = "ENABLED";
    OB us_tx_pad_58 (.I(us_tx_c_58), .O(us_tx[58]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i26 (.D(accepted_sequence_spi[26]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[26])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i26.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i25 (.D(accepted_sequence_spi[25]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[25])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i25.GSR = "ENABLED";
    LUT4 i15791_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[13]), 
         .Z(spi_byte_count_15__N_1016)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15791_2_lut.init = 16'h1111;
    FD1P3AX accepted_sequence_i0_i24 (.D(accepted_sequence_spi[24]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[24])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i24.GSR = "ENABLED";
    OB us_tx_pad_76 (.I(us_tx_c_76), .O(us_tx[76]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_59 (.I(us_tx_c_59), .O(us_tx[59]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i23 (.D(accepted_sequence_spi[23]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[23])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i23.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i22 (.D(accepted_sequence_spi[22]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[22])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i22.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i21 (.D(accepted_sequence_spi[21]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[21])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i21.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i20 (.D(accepted_sequence_spi[20]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[20])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i20.GSR = "ENABLED";
    OB us_tx_pad_60 (.I(us_tx_c_60), .O(us_tx[60]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i19 (.D(accepted_sequence_spi[19]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[19])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i19.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i18 (.D(accepted_sequence_spi[18]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[18])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i18.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i17 (.D(accepted_sequence_spi[17]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[17])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i17.GSR = "ENABLED";
    OB us_tx_pad_61 (.I(us_tx_c_61), .O(us_tx[61]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i16 (.D(accepted_sequence_spi[16]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[16])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i16.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i15 (.D(accepted_sequence_spi[15]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[15])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i15.GSR = "ENABLED";
    OB us_tx_pad_62 (.I(us_tx_c_62), .O(us_tx[62]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i14 (.D(accepted_sequence_spi[14]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[14])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i14.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i13 (.D(accepted_sequence_spi[13]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[13])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i13.GSR = "ENABLED";
    OB us_tx_pad_63 (.I(us_tx_c_63), .O(us_tx[63]));   // src/umh_fpga_top.v(11[24:29])
    CCU2D spi_byte_count_15__I_0_2169_32 (.A0(spi_expected_length[1]), .B0(spi_byte_count[1]), 
          .C0(spi_expected_length[0]), .D0(spi_byte_count[0]), .A1(GND_net), 
          .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n17059), .S1(accepted_sequence_spi_31__N_2262));
    defparam spi_byte_count_15__I_0_2169_32.INIT0 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_32.INIT1 = 16'hFFFF;
    defparam spi_byte_count_15__I_0_2169_32.INJECT1_0 = "YES";
    defparam spi_byte_count_15__I_0_2169_32.INJECT1_1 = "NO";
    FD1P3AX accepted_sequence_i0_i12 (.D(accepted_sequence_spi[12]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[12])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i12.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i11 (.D(accepted_sequence_spi[11]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[11])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i11.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i10 (.D(accepted_sequence_spi[10]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[10])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i10.GSR = "ENABLED";
    OB us_tx_pad_64 (.I(us_tx_c_64), .O(us_tx[64]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i9 (.D(accepted_sequence_spi[9]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[9])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i9.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1751_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[14]), 
         .Z(spi_byte_count_15__N_964)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1751_2_lut.init = 16'h4444;
    FD1P3AX accepted_sequence_i0_i8 (.D(accepted_sequence_spi[8]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[8])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i8.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i7 (.D(accepted_sequence_spi[7]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[7])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i7.GSR = "ENABLED";
    OB us_tx_pad_65 (.I(us_tx_c_65), .O(us_tx[65]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i6 (.D(accepted_sequence_spi[6]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[6])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i6.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i5 (.D(accepted_sequence_spi[5]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[5])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i5.GSR = "ENABLED";
    OB us_tx_pad_66 (.I(us_tx_c_66), .O(us_tx[66]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i4 (.D(accepted_sequence_spi[4]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[4])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i4.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i3 (.D(accepted_sequence_spi[3]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[3])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i3.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2062_15 (.A0(n9809), .B0(n9808), .C0(GND_net), 
          .D0(GND_net), .A1(n9813), .B1(n9812), .C1(GND_net), .D1(GND_net), 
          .CIN(n17130), .COUT(n17131), .S0(spi_byte_count_15__N_997[13]), 
          .S1(spi_byte_count_15__N_997[14]));   // src/umh_fpga_top.v(234[31:52])
    defparam spi_byte_count_15__I_0_2062_15.INIT0 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_15.INIT1 = 16'h7888;
    defparam spi_byte_count_15__I_0_2062_15.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2062_15.INJECT1_1 = "NO";
    OB us_tx_pad_67 (.I(us_tx_c_67), .O(us_tx[67]));   // src/umh_fpga_top.v(11[24:29])
    FD1P3AX accepted_sequence_i0_i2 (.D(accepted_sequence_spi[2]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[2])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i2.GSR = "ENABLED";
    FD1P3AX accepted_sequence_i0_i1 (.D(accepted_sequence_spi[1]), .SP(running_N_2903), 
            .CK(fpga_clk_c), .Q(accepted_sequence[1])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam accepted_sequence_i0_i1.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i13_5597_5598_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_966), .Q(n9809)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i13_5597_5598_set.GSR = "ENABLED";
    OB us_tx_pad_68 (.I(us_tx_c_68), .O(us_tx[68]));   // src/umh_fpga_top.v(11[24:29])
    LUT4 i15788_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[14]), 
         .Z(spi_byte_count_15__N_1013)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15788_2_lut.init = 16'h1111;
    OB us_tx_pad_69 (.I(us_tx_c_69), .O(us_tx[69]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_byte_count_i12_5593_5594_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_968), .Q(n9805)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i12_5593_5594_set.GSR = "ENABLED";
    OB us_tx_pad_70 (.I(us_tx_c_70), .O(us_tx[70]));   // src/umh_fpga_top.v(11[24:29])
    OB us_tx_pad_71 (.I(us_tx_c_71), .O(us_tx[71]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_byte_count_i11_5589_5590_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_970), .Q(n9801)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i11_5589_5590_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1750_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[15]), 
         .Z(spi_byte_count_15__N_962)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1750_2_lut.init = 16'h4444;
    OB us_tx_pad_72 (.I(us_tx_c_72), .O(us_tx[72]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_byte_count_i10_5585_5586_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_972), .Q(n9797)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i10_5585_5586_set.GSR = "ENABLED";
    OB us_tx_pad_73 (.I(us_tx_c_73), .O(us_tx[73]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_byte_count_i9_5581_5582_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_974), .Q(n9793)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i9_5581_5582_set.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i8_5577_5578_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_976), .Q(n9789)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i8_5577_5578_set.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2169_31 (.A0(spi_expected_length[5]), .B0(spi_byte_count[5]), 
          .C0(spi_expected_length[4]), .D0(spi_byte_count[4]), .A1(spi_expected_length[3]), 
          .B1(spi_byte_count[3]), .C1(spi_expected_length[2]), .D1(spi_byte_count[2]), 
          .CIN(n17058), .COUT(n17059));
    defparam spi_byte_count_15__I_0_2169_31.INIT0 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_31.INIT1 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_31.INJECT1_0 = "YES";
    defparam spi_byte_count_15__I_0_2169_31.INJECT1_1 = "YES";
    OB us_tx_pad_74 (.I(us_tx_c_74), .O(us_tx[74]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_byte_count_i7_5573_5574_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_978), .Q(n9785)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i7_5573_5574_set.GSR = "ENABLED";
    OB us_tx_pad_75 (.I(us_tx_c_75), .O(us_tx[75]));   // src/umh_fpga_top.v(11[24:29])
    FD1S3BX spi_byte_count_i6_5569_5570_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_980), .Q(n9781)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i6_5569_5570_set.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i5_5565_5566_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_982), .Q(n9777)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i5_5565_5566_set.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i4_5561_5562_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_984), .Q(n9773)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i4_5561_5562_set.GSR = "ENABLED";
    LUT4 i15785_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[15]), 
         .Z(spi_byte_count_15__N_994)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15785_2_lut.init = 16'h1111;
    FD1S3BX spi_byte_count_i3_5557_5558_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_986), .Q(n9769)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i3_5557_5558_set.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i2_5553_5554_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_988), .Q(n9765)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i2_5553_5554_set.GSR = "ENABLED";
    FD1S3BX spi_byte_count_i1_5549_5550_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_byte_count_15__N_990), .Q(n9761)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_byte_count_i1_5549_5550_set.GSR = "ENABLED";
    FD1S3BX spi_channel_field_i1_5545_5546_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_field_1__N_1772), .Q(n9757)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_field_i1_5545_5546_set.GSR = "ENABLED";
    FD1S3BX spi_level_pending_i7_5541_5542_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1784), .Q(n9753)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i7_5541_5542_set.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2169_29 (.A0(spi_expected_length[9]), .B0(spi_byte_count[9]), 
          .C0(spi_expected_length[8]), .D0(spi_byte_count[8]), .A1(spi_expected_length[7]), 
          .B1(spi_byte_count[7]), .C1(spi_expected_length[6]), .D1(spi_byte_count[6]), 
          .CIN(n17057), .COUT(n17058));
    defparam spi_byte_count_15__I_0_2169_29.INIT0 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_29.INIT1 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_29.INJECT1_0 = "YES";
    defparam spi_byte_count_15__I_0_2169_29.INJECT1_1 = "YES";
    FD1S3BX spi_level_pending_i6_5537_5538_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1786), .Q(n9749)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i6_5537_5538_set.GSR = "ENABLED";
    CCU2D spi_byte_count_15__I_0_2169_0 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(spi_expected_length[31]), .B1(spi_expected_length[30]), 
          .C1(spi_expected_length[29]), .D1(spi_expected_length[28]), .COUT(n17054));   // src/umh_fpga_top.v(256[9:46])
    defparam spi_byte_count_15__I_0_2169_0.INIT0 = 16'hF000;
    defparam spi_byte_count_15__I_0_2169_0.INIT1 = 16'h0001;
    defparam spi_byte_count_15__I_0_2169_0.INJECT1_0 = "NO";
    defparam spi_byte_count_15__I_0_2169_0.INJECT1_1 = "YES";
    CCU2D spi_byte_count_15__I_0_2169_27 (.A0(spi_expected_length[13]), .B0(spi_byte_count[13]), 
          .C0(spi_expected_length[12]), .D0(spi_byte_count[12]), .A1(spi_expected_length[11]), 
          .B1(spi_byte_count[11]), .C1(spi_expected_length[10]), .D1(spi_byte_count[10]), 
          .CIN(n17056), .COUT(n17057));
    defparam spi_byte_count_15__I_0_2169_27.INIT0 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_27.INIT1 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_27.INJECT1_0 = "YES";
    defparam spi_byte_count_15__I_0_2169_27.INJECT1_1 = "YES";
    LUT4 i5611_3_lut (.A(n9822), .B(n9821), .C(spi1_sck_c_enable_77), 
         .Z(rgb_values[95])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5611_3_lut.init = 16'hcaca;
    FD1S3BX spi_level_pending_i5_5533_5534_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1788), .Q(n9745)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i5_5533_5534_set.GSR = "ENABLED";
    CCU2D add_2583_cout (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n17141), 
          .S0(spi_expected_length_31__N_2085[16]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_cout.INIT0 = 16'h0000;
    defparam add_2583_cout.INIT1 = 16'h0000;
    defparam add_2583_cout.INJECT1_0 = "NO";
    defparam add_2583_cout.INJECT1_1 = "NO";
    CCU2D add_2583_15 (.A0(n17064), .B0(n8_adj_3389), .C0(n7), .D0(spi_expected_length[14]), 
          .A1(n17062), .B1(n8_adj_3389), .C1(n7), .D1(spi_expected_length[15]), 
          .CIN(n17140), .COUT(n17141), .S0(spi_expected_length_31__N_2085[14]), 
          .S1(spi_expected_length_31__N_2085[15]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_15.INIT0 = 16'h56aa;
    defparam add_2583_15.INIT1 = 16'h56aa;
    defparam add_2583_15.INJECT1_0 = "NO";
    defparam add_2583_15.INJECT1_1 = "NO";
    CCU2D add_2583_13 (.A0(n17068), .B0(n8_adj_3389), .C0(n7), .D0(spi_expected_length[12]), 
          .A1(n17066), .B1(n8_adj_3389), .C1(n7), .D1(spi_expected_length[13]), 
          .CIN(n17139), .COUT(n17140), .S0(spi_expected_length_31__N_2085[12]), 
          .S1(spi_expected_length_31__N_2085[13]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_13.INIT0 = 16'h56aa;
    defparam add_2583_13.INIT1 = 16'h56aa;
    defparam add_2583_13.INJECT1_0 = "NO";
    defparam add_2583_13.INJECT1_1 = "NO";
    FD1S3BX spi_level_pending_i4_5529_5530_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1790), .Q(n9741)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i4_5529_5530_set.GSR = "ENABLED";
    LUT4 i5643_3_lut (.A(n9854), .B(n9853), .C(spi1_sck_c_enable_69), 
         .Z(rgb_values[87])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5643_3_lut.init = 16'hcaca;
    FD1S3BX spi_level_pending_i3_5525_5526_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1792), .Q(n9737)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i3_5525_5526_set.GSR = "ENABLED";
    FD1S3BX spi_level_pending_i2_5521_5522_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1794), .Q(n9733)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i2_5521_5522_set.GSR = "ENABLED";
    CCU2D add_2583_11 (.A0(n17072), .B0(n8_adj_3389), .C0(n7), .D0(spi_expected_length[10]), 
          .A1(n17070), .B1(n8_adj_3389), .C1(n7), .D1(spi_expected_length[11]), 
          .CIN(n17138), .COUT(n17139), .S0(spi_expected_length_31__N_2085[10]), 
          .S1(spi_expected_length_31__N_2085[11]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_11.INIT0 = 16'h56aa;
    defparam add_2583_11.INIT1 = 16'h56aa;
    defparam add_2583_11.INJECT1_0 = "NO";
    defparam add_2583_11.INJECT1_1 = "NO";
    LUT4 i9734_2_lut (.A(n9729), .B(n9728), .Z(spi_level_pending[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9734_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1765_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[0]), 
         .Z(spi_byte_count_15__N_992)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1765_2_lut.init = 16'h4444;
    LUT4 i15830_2_lut (.A(fpga_cs_n_c), .B(spi_byte_count_15__N_997[0]), 
         .Z(spi_byte_count_15__N_1055)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15830_2_lut.init = 16'h1111;
    LUT4 i9870_2_lut (.A(n9301), .B(n9300), .Z(spi_expected_length[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9870_2_lut.init = 16'h8888;
    FD1S3BX spi_level_pending_i1_5517_5518_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_level_pending_7__N_1796), .Q(n9729)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_level_pending_i1_5517_5518_set.GSR = "ENABLED";
    FD1S3BX spi_update_flags_i0_5065_5066_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_update_flags_15__N_1184), .Q(n9277)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_update_flags_i0_5065_5066_set.GSR = "ENABLED";
    LUT4 i32_4_lut (.A(status_bit_index[1]), .B(status_flags_wire_15__N_2034[2]), 
         .C(status_bit_index[2]), .D(status_flags_wire_15__N_2050[4]), .Z(n11)) /* synthesis lut_function=(!(A (C+!(D))+!A !(B (C)))) */ ;
    defparam i32_4_lut.init = 16'h4a40;
    FD1S3BX spi_frame_sequence_i0_5061_5062_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1408), .Q(n9273)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i0_5061_5062_set.GSR = "ENABLED";
    FD1S3BX spi_channel_index_i6_5513_5514_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_index_6__N_1730), .Q(n9725)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_index_i6_5513_5514_set.GSR = "ENABLED";
    FD1S3BX spi_channel_index_i5_5509_5510_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_index_6__N_1732), .Q(n9721)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_index_i5_5509_5510_set.GSR = "ENABLED";
    CCU2D fpga_time_2861_add_4_33 (.A0(fpga_time[31]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17213), .S0(n134));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_33.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_33.INIT1 = 16'h0000;
    defparam fpga_time_2861_add_4_33.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_33.INJECT1_1 = "NO";
    CCU2D fpga_time_2861_add_4_31 (.A0(fpga_time[29]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[30]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17212), .COUT(n17213), .S0(n136), .S1(n135));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_31.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_31.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_31.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_31.INJECT1_1 = "NO";
    FD1S3BX spi_channel_index_i4_5505_5506_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_index_6__N_1734), .Q(n9717)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_index_i4_5505_5506_set.GSR = "ENABLED";
    LUT4 i2_4_lut_adj_50 (.A(status_bit_index[5]), .B(n4), .C(status_bit_index[4]), 
         .D(fifo_depth[0]), .Z(n15)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A ((C (D))+!B))) */ ;
    defparam i2_4_lut_adj_50.init = 16'h0c44;
    CCU2D fpga_time_2861_add_4_29 (.A0(fpga_time[27]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[28]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17211), .COUT(n17212), .S0(n138), .S1(n137));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_29.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_29.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_29.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_29.INJECT1_1 = "NO";
    FD1P3AX frame_toggle_seen_1516 (.D(frame_toggle_sync), .SP(frame_toggle_seen_N_2889), 
            .CK(fpga_clk_c), .Q(frame_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam frame_toggle_seen_1516.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i0 (.D(spi_frame_sequence[0]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[0])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i0.GSR = "ENABLED";
    LUT4 i1_2_lut (.A(status_bit_index[2]), .B(status_bit_index[1]), .Z(n4)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut.init = 16'h8888;
    LUT4 i9849_2_lut (.A(n9557), .B(n9556), .Z(spi_frame_sequence[11])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9849_2_lut.init = 16'h8888;
    FD1S3BX spi_channel_index_i3_5501_5502_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_index_6__N_1736), .Q(n9713)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_index_i3_5501_5502_set.GSR = "ENABLED";
    CCU2D fpga_time_2861_add_4_27 (.A0(fpga_time[25]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[26]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17210), .COUT(n17211), .S0(n140), .S1(n139));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_27.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_27.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_27.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_27.INJECT1_1 = "NO";
    CCU2D add_2583_9 (.A0(spi1_mosi_c), .B0(n37_adj_3400), .C0(spi_expected_length[8]), 
          .D0(spi_update_flags[0]), .A1(n17074), .B1(n8_adj_3389), .C1(n7), 
          .D1(spi_expected_length[9]), .CIN(n17137), .COUT(n17138), .S0(spi_expected_length_31__N_2085[8]), 
          .S1(spi_expected_length_31__N_2085[9]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_9.INIT0 = 16'hd1e2;
    defparam add_2583_9.INIT1 = 16'h56aa;
    defparam add_2583_9.INJECT1_0 = "NO";
    defparam add_2583_9.INJECT1_1 = "NO";
    FD1S3BX spi_channel_index_i2_5497_5498_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_index_6__N_1738), .Q(n9709)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_index_i2_5497_5498_set.GSR = "ENABLED";
    FD1S3BX spi_channel_index_i1_5493_5494_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_channel_index_6__N_1740), .Q(n9705)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_channel_index_i1_5493_5494_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i15_5489_5490_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1250), .Q(n9701)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i15_5489_5490_set.GSR = "ENABLED";
    MUX321 spi1_miso_I_3_i621 (.D0(fpga_time[16]), .D1(fpga_time[17]), .D2(fpga_time[18]), 
           .D3(fpga_time[19]), .D4(fpga_time[20]), .D5(fpga_time[21]), 
           .D6(fpga_time[22]), .D7(fpga_time[23]), .D8(fpga_time[24]), 
           .D9(fpga_time[25]), .D10(fpga_time[26]), .D11(fpga_time[27]), 
           .D12(fpga_time[28]), .D13(fpga_time[29]), .D14(fpga_time[30]), 
           .D15(fpga_time[31]), .D16(fpga_time[0]), .D17(fpga_time[1]), 
           .D18(fpga_time[2]), .D19(fpga_time[3]), .D20(fpga_time[4]), 
           .D21(fpga_time[5]), .D22(fpga_time[6]), .D23(fpga_time[7]), 
           .D24(fpga_time[8]), .D25(fpga_time[9]), .D26(fpga_time[10]), 
           .D27(fpga_time[11]), .D28(fpga_time[12]), .D29(fpga_time[13]), 
           .D30(fpga_time[14]), .D31(fpga_time[15]), .SD1(spi1_miso_N_2860[0]), 
           .SD2(spi1_miso_N_2860[1]), .SD3(spi1_miso_N_2860[2]), .SD4(status_bit_index[3]), 
           .SD5(spi1_miso_N_2860[4]), .Z(n62_adj_3401));
    FD1S3BX spi_expected_length_i26_5193_5194_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1548), .Q(n9405)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i26_5193_5194_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i25_5189_5190_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1550), .Q(n9401)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i25_5189_5190_set.GSR = "ENABLED";
    FD1S3AX phase_acc_i31 (.D(phase_acc_next[31]), .CK(fpga_clk_c), .Q(phase_acc[31])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i31.GSR = "ENABLED";
    FD1S3AX phase_acc_i30 (.D(phase_acc_next[30]), .CK(fpga_clk_c), .Q(phase_acc[30])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i30.GSR = "ENABLED";
    FD1S3AX phase_acc_i29 (.D(phase_acc_next[29]), .CK(fpga_clk_c), .Q(phase_acc[29])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i29.GSR = "ENABLED";
    FD1S3AX phase_acc_i28 (.D(phase_acc_next[28]), .CK(fpga_clk_c), .Q(phase_acc[28])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i28.GSR = "ENABLED";
    FD1S3AX phase_acc_i27 (.D(phase_acc_next[27]), .CK(fpga_clk_c), .Q(phase_acc[27])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i27.GSR = "ENABLED";
    FD1S3AX phase_acc_i26 (.D(phase_acc_next[26]), .CK(fpga_clk_c), .Q(phase_acc[26])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i26.GSR = "ENABLED";
    FD1S3AX phase_acc_i25 (.D(phase_acc_next[25]), .CK(fpga_clk_c), .Q(phase_acc[25])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i25.GSR = "ENABLED";
    FD1S3AX phase_acc_i24 (.D(phase_acc_next[24]), .CK(fpga_clk_c), .Q(phase_acc[24])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i24.GSR = "ENABLED";
    MUX321 i69782 (.D0(accepted_sequence[0]), .D1(accepted_sequence[1]), 
           .D2(accepted_sequence[2]), .D3(accepted_sequence[3]), .D4(accepted_sequence[4]), 
           .D5(accepted_sequence[5]), .D6(accepted_sequence[6]), .D7(accepted_sequence[7]), 
           .D8(accepted_sequence[8]), .D9(accepted_sequence[9]), .D10(accepted_sequence[10]), 
           .D11(accepted_sequence[11]), .D12(accepted_sequence[12]), .D13(accepted_sequence[13]), 
           .D14(accepted_sequence[14]), .D15(accepted_sequence[15]), .D16(accepted_sequence[16]), 
           .D17(accepted_sequence[17]), .D18(accepted_sequence[18]), .D19(accepted_sequence[19]), 
           .D20(accepted_sequence[20]), .D21(accepted_sequence[21]), .D22(accepted_sequence[22]), 
           .D23(accepted_sequence[23]), .D24(accepted_sequence[24]), .D25(accepted_sequence[25]), 
           .D26(accepted_sequence[26]), .D27(accepted_sequence[27]), .D28(accepted_sequence[28]), 
           .D29(accepted_sequence[29]), .D30(accepted_sequence[30]), .D31(accepted_sequence[31]), 
           .SD1(spi1_miso_N_2860[0]), .SD2(spi1_miso_N_2860[1]), .SD3(spi1_miso_N_2860[2]), 
           .SD4(status_bit_index[3]), .SD5(status_bit_index[4]), .Z(n11180));
    FD1S3AX phase_acc_i23 (.D(phase_acc_next[23]), .CK(fpga_clk_c), .Q(phase_acc[23])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i23.GSR = "ENABLED";
    FD1S3AX phase_acc_i22 (.D(phase_acc_next[22]), .CK(fpga_clk_c), .Q(phase_acc[22])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i22.GSR = "ENABLED";
    FD1P3AX stop_toggle_seen_1521 (.D(stop_toggle_sync), .SP(stop_toggle_seen_N_2891), 
            .CK(fpga_clk_c), .Q(stop_toggle_seen)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam stop_toggle_seen_1521.GSR = "ENABLED";
    LUT4 i1_4_lut_adj_51 (.A(status_flags_wire_15__N_2050[4]), .B(n29_adj_3373), 
         .C(phase_active[83]), .D(phase_acc[31]), .Z(us_tx_c_83)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_51.init = 16'h0880;
    FD1S3AX phase_acc_i21 (.D(phase_acc_next[21]), .CK(fpga_clk_c), .Q(phase_acc[21])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i21.GSR = "ENABLED";
    LUT4 i9863_2_lut (.A(n9305), .B(n9304), .Z(spi_expected_length[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9863_2_lut.init = 16'h8888;
    FD1S3AX phase_acc_i20 (.D(phase_acc_next[20]), .CK(fpga_clk_c), .Q(phase_acc[20])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i20.GSR = "ENABLED";
    FD1S3AX phase_acc_i19 (.D(phase_acc_next[19]), .CK(fpga_clk_c), .Q(phase_acc[19])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i19.GSR = "ENABLED";
    FD1S3AX phase_acc_i18 (.D(phase_acc_next[18]), .CK(fpga_clk_c), .Q(phase_acc[18])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i18.GSR = "ENABLED";
    FD1S3AX phase_acc_i17 (.D(phase_acc_next[17]), .CK(fpga_clk_c), .Q(phase_acc[17])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i17.GSR = "ENABLED";
    FD1S3AX phase_acc_i16 (.D(phase_acc_next[16]), .CK(fpga_clk_c), .Q(phase_acc[16])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i16.GSR = "ENABLED";
    FD1S3AX phase_acc_i15 (.D(phase_acc_next[15]), .CK(fpga_clk_c), .Q(phase_acc[15])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i15.GSR = "ENABLED";
    FD1S3AX phase_acc_i14 (.D(phase_acc_next[14]), .CK(fpga_clk_c), .Q(phase_acc[14])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i14.GSR = "ENABLED";
    FD1S3AX phase_acc_i13 (.D(phase_acc_next[13]), .CK(fpga_clk_c), .Q(phase_acc[13])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i13.GSR = "ENABLED";
    FD1S3AX phase_acc_i12 (.D(phase_acc_next[12]), .CK(fpga_clk_c), .Q(phase_acc[12])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i12.GSR = "ENABLED";
    FD1S3AX phase_acc_i11 (.D(phase_acc_next[11]), .CK(fpga_clk_c), .Q(phase_acc[11])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i11.GSR = "ENABLED";
    FD1S3AX phase_acc_i10 (.D(phase_acc_next[10]), .CK(fpga_clk_c), .Q(phase_acc[10])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i10.GSR = "ENABLED";
    FD1S3AX phase_acc_i9 (.D(phase_acc_next[9]), .CK(fpga_clk_c), .Q(phase_acc[9])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i9.GSR = "ENABLED";
    FD1S3AX phase_acc_i8 (.D(phase_acc_next[8]), .CK(fpga_clk_c), .Q(phase_acc[8])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i8.GSR = "ENABLED";
    FD1S3AX phase_acc_i7 (.D(phase_acc_next[7]), .CK(fpga_clk_c), .Q(phase_acc[7])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i7.GSR = "ENABLED";
    FD1S3AX phase_acc_i6 (.D(phase_acc_next[6]), .CK(fpga_clk_c), .Q(phase_acc[6])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i6.GSR = "ENABLED";
    FD1S3AX phase_acc_i5 (.D(phase_acc_next[5]), .CK(fpga_clk_c), .Q(phase_acc[5])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i5.GSR = "ENABLED";
    FD1S3AX phase_acc_i4 (.D(phase_acc_next[4]), .CK(fpga_clk_c), .Q(phase_acc[4])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i4.GSR = "ENABLED";
    FD1S3AX phase_acc_i3 (.D(phase_acc_next[3]), .CK(fpga_clk_c), .Q(phase_acc[3])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i3.GSR = "ENABLED";
    FD1S3AX phase_acc_i2 (.D(phase_acc_next[2]), .CK(fpga_clk_c), .Q(phase_acc[2])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i2.GSR = "ENABLED";
    FD1S3AX phase_acc_i1 (.D(phase_acc_next[1]), .CK(fpga_clk_c), .Q(phase_acc[1])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_acc_i1.GSR = "ENABLED";
    FD1P3AX level_active_83___i168 (.D(n2315), .SP(fpga_clk_c_enable_415), 
            .CK(fpga_clk_c), .Q(\level_active[0] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i168.GSR = "ENABLED";
    FD1P3AX level_active_83___i167 (.D(n2316), .SP(fpga_clk_c_enable_415), 
            .CK(fpga_clk_c), .Q(\level_active[0] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i167.GSR = "ENABLED";
    FD1P3AX level_active_83___i166 (.D(n2315), .SP(fpga_clk_c_enable_414), 
            .CK(fpga_clk_c), .Q(\level_active[1] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i166.GSR = "ENABLED";
    FD1P3AX level_active_83___i165 (.D(n2316), .SP(fpga_clk_c_enable_414), 
            .CK(fpga_clk_c), .Q(\level_active[1] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i165.GSR = "ENABLED";
    FD1P3AX level_active_83___i164 (.D(n2315), .SP(fpga_clk_c_enable_413), 
            .CK(fpga_clk_c), .Q(\level_active[2] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i164.GSR = "ENABLED";
    FD1P3AX level_active_83___i163 (.D(n2316), .SP(fpga_clk_c_enable_413), 
            .CK(fpga_clk_c), .Q(\level_active[2] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i163.GSR = "ENABLED";
    FD1P3AX level_active_83___i162 (.D(n2315), .SP(fpga_clk_c_enable_412), 
            .CK(fpga_clk_c), .Q(\level_active[3] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i162.GSR = "ENABLED";
    FD1P3AX level_active_83___i161 (.D(n2316), .SP(fpga_clk_c_enable_412), 
            .CK(fpga_clk_c), .Q(\level_active[3] [0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i161.GSR = "ENABLED";
    FD1P3AX level_active_83___i160 (.D(n2315), .SP(fpga_clk_c_enable_411), 
            .CK(fpga_clk_c), .Q(\level_active[4] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i160.GSR = "ENABLED";
    FD1S3BX spi_version_i0_5057_5058_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_version_7__N_1120), 
            .Q(n9269)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_version_i0_5057_5058_set.GSR = "ENABLED";
    LUT4 i30_3_lut_4_lut_adj_52 (.A(amplitude_phase[6]), .B(\level_active[5] [0]), 
         .C(\level_active[5] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3370)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_52.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_53 (.A(amplitude_phase[6]), .B(\level_active[0] [0]), 
         .C(\level_active[0] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3505)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_53.init = 16'h40f4;
    LUT4 i32_3_lut_4_lut (.A(amplitude_phase[6]), .B(\level_active[83] [0]), 
         .C(\level_active[83] [1]), .D(amplitude_phase[7]), .Z(n29_adj_3373)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i32_3_lut_4_lut.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1971_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[2]), 
         .Z(spi_expected_length_31__N_1596)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1971_2_lut.init = 16'h4444;
    LUT4 i15434_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[3]), 
         .Z(spi_expected_length_31__N_1718)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15434_2_lut.init = 16'h1111;
    LUT4 i1_2_lut_3_lut (.A(status_bit_index[4]), .B(status_bit_index[5]), 
         .C(n11), .Z(n18641)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i1_2_lut_3_lut.init = 16'h8080;
    LUT4 i1_2_lut_adj_54 (.A(fpga_cs_n_c), .B(n8), .Z(spi1_miso_c)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(7[24:33])
    defparam i1_2_lut_adj_54.init = 16'h4444;
    FD1S3BX spi_expected_length_i9_5125_5126_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1582), .Q(n9337)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i9_5125_5126_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i14_5485_5486_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1252), .Q(n9697)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i14_5485_5486_set.GSR = "ENABLED";
    LUT4 i2_3_lut_4_lut (.A(status_bit_index[4]), .B(status_bit_index[5]), 
         .C(status_bit_index[3]), .D(status_bit_index[6]), .Z(n18642)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i2_3_lut_4_lut.init = 16'h8000;
    FD1S3BX spi_expected_length_i24_5185_5186_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1552), .Q(n9397)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i24_5185_5186_set.GSR = "ENABLED";
    LUT4 i15431_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[4]), 
         .Z(spi_expected_length_31__N_1715)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15431_2_lut.init = 16'h1111;
    LUT4 rgb_values_95__I_0_1633_2_lut_4_lut (.A(rgb_values[95]), .B(rgb_values[87]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_337)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam rgb_values_95__I_0_1633_2_lut_4_lut.init = 16'h00ca;
    FD1S3BX spi_extension_length_i13_5481_5482_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1254), .Q(n9693)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i13_5481_5482_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i12_5477_5478_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1256), .Q(n9689)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i12_5477_5478_set.GSR = "ENABLED";
    LUT4 i9681_2_lut (.A(n9549), .B(n9548), .Z(spi_frame_sequence[9])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9681_2_lut.init = 16'h8888;
    FD1S3BX spi_extension_length_i11_5473_5474_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1258), .Q(n9685)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i11_5473_5474_set.GSR = "ENABLED";
    LUT4 i15497_2_lut_4_lut (.A(rgb_values[95]), .B(rgb_values[87]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_531)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15497_2_lut_4_lut.init = 16'h0035;
    FD1S3BX spi_extension_length_i10_5469_5470_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1260), .Q(n9681)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i10_5469_5470_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i9_5465_5466_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1262), .Q(n9677)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i9_5465_5466_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i8_5461_5462_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1264), .Q(n9673)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i8_5461_5462_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1968_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[5]), 
         .Z(spi_expected_length_31__N_1590)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1968_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1848_2_lut_4_lut (.A(spi_extension_length[13]), 
         .B(spi_rx_shift[4]), .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1254)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1848_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15425_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[6]), 
         .Z(spi_expected_length_31__N_1709)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15425_2_lut.init = 16'h1111;
    FD1S3BX spi_expected_length_i8_5121_5122_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1584), .Q(n9333)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i8_5121_5122_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i23_5181_5182_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1554), .Q(n9393)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i23_5181_5182_set.GSR = "ENABLED";
    FD1S3BX spi_command_i0_5053_5054_set (.D(n21164), .CK(spi1_sck_c), .PD(spi_command_7__N_1072), 
            .Q(n9265)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_command_i0_5053_5054_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i11_5133_5134_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1578), .Q(n9345)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i11_5133_5134_set.GSR = "ENABLED";
    LUT4 i15308_2_lut_4_lut (.A(spi_extension_length[13]), .B(spi_rx_shift[4]), 
         .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1304)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15308_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1849_2_lut_4_lut (.A(spi_extension_length[12]), 
         .B(spi_rx_shift[3]), .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1256)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1849_2_lut_4_lut.init = 16'h00ca;
    LUT4 i2_4_lut_adj_55 (.A(phase_active[0]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3505), .Z(us_tx_c_0)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_55.init = 16'h4800;
    LUT4 i15311_2_lut_4_lut (.A(spi_extension_length[12]), .B(spi_rx_shift[3]), 
         .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1307)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15311_2_lut_4_lut.init = 16'h0035;
    LUT4 n128_bdd_3_lut_4_lut (.A(spi_byte_count[9]), .B(n18524), .C(n20868), 
         .D(n18643), .Z(n8870)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam n128_bdd_3_lut_4_lut.init = 16'h1000;
    FD1S3BX spi_expected_length_i10_5129_5130_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1580), .Q(n9341)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i10_5129_5130_set.GSR = "ENABLED";
    LUT4 i1_2_lut_3_lut_4_lut (.A(spi_byte_count[9]), .B(n18524), .C(n7_adj_3395), 
         .D(n18527), .Z(n36_adj_3431)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut.init = 16'hfffe;
    FD1S3BX spi_expected_length_i22_5177_5178_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1556), .Q(n9389)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i22_5177_5178_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i21_5173_5174_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1558), .Q(n9385)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i21_5173_5174_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i20_5169_5170_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1560), .Q(n9381)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i20_5169_5170_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i19_5165_5166_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1562), .Q(n9377)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i19_5165_5166_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1850_2_lut_4_lut (.A(spi_extension_length[11]), 
         .B(spi_rx_shift[2]), .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1258)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1850_2_lut_4_lut.init = 16'h00ca;
    FD1S3BX spi_expected_length_i18_5161_5162_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1564), .Q(n9373)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i18_5161_5162_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i17_5157_5158_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1566), .Q(n9369)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i17_5157_5158_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i16_5153_5154_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1568), .Q(n9365)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i16_5153_5154_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i15_5149_5150_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1570), .Q(n9361)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i15_5149_5150_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i14_5145_5146_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1572), .Q(n9357)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i14_5145_5146_set.GSR = "ENABLED";
    FD1S3BX spi_expected_length_i13_5141_5142_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_expected_length_31__N_1574), .Q(n9353)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_expected_length_i13_5141_5142_set.GSR = "ENABLED";
    LUT4 i15314_2_lut_4_lut (.A(spi_extension_length[11]), .B(spi_rx_shift[2]), 
         .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1310)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15314_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1851_2_lut_4_lut (.A(spi_extension_length[10]), 
         .B(spi_rx_shift[1]), .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1260)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1851_2_lut_4_lut.init = 16'h00ca;
    FD1P3AX phase_active_i83_i82 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_333), 
            .CK(fpga_clk_c), .Q(phase_active[82])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i82.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i81 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_334), 
            .CK(fpga_clk_c), .Q(phase_active[81])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i81.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i80 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_335), 
            .CK(fpga_clk_c), .Q(phase_active[80])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i80.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i79 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_336), 
            .CK(fpga_clk_c), .Q(phase_active[79])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i79.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i78 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_337), 
            .CK(fpga_clk_c), .Q(phase_active[78])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i78.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i77 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_338), 
            .CK(fpga_clk_c), .Q(phase_active[77])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i77.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i76 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_339), 
            .CK(fpga_clk_c), .Q(phase_active[76])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i76.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i75 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_340), 
            .CK(fpga_clk_c), .Q(phase_active[75])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i75.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i74 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_341), 
            .CK(fpga_clk_c), .Q(phase_active[74])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i74.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i73 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_342), 
            .CK(fpga_clk_c), .Q(phase_active[73])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i73.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i72 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_343), 
            .CK(fpga_clk_c), .Q(phase_active[72])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i72.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i71 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_344), 
            .CK(fpga_clk_c), .Q(phase_active[71])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i71.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i70 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_345), 
            .CK(fpga_clk_c), .Q(phase_active[70])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i70.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i69 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_346), 
            .CK(fpga_clk_c), .Q(phase_active[69])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i69.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i68 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_347), 
            .CK(fpga_clk_c), .Q(phase_active[68])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i68.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i67 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_348), 
            .CK(fpga_clk_c), .Q(phase_active[67])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i67.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i66 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_349), 
            .CK(fpga_clk_c), .Q(phase_active[66])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i66.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i65 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_350), 
            .CK(fpga_clk_c), .Q(phase_active[65])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i65.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i64 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_351), 
            .CK(fpga_clk_c), .Q(phase_active[64])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i64.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i63 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_352), 
            .CK(fpga_clk_c), .Q(phase_active[63])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i63.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i62 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_353), 
            .CK(fpga_clk_c), .Q(phase_active[62])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i62.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i61 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_354), 
            .CK(fpga_clk_c), .Q(phase_active[61])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i61.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i60 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_355), 
            .CK(fpga_clk_c), .Q(phase_active[60])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i60.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i59 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_356), 
            .CK(fpga_clk_c), .Q(phase_active[59])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i59.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i58 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_357), 
            .CK(fpga_clk_c), .Q(phase_active[58])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i58.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i57 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_358), 
            .CK(fpga_clk_c), .Q(phase_active[57])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i57.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i56 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_359), 
            .CK(fpga_clk_c), .Q(phase_active[56])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i56.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i55 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_360), 
            .CK(fpga_clk_c), .Q(phase_active[55])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i55.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i54 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_361), 
            .CK(fpga_clk_c), .Q(phase_active[54])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i54.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i53 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_362), 
            .CK(fpga_clk_c), .Q(phase_active[53])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i53.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i52 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_363), 
            .CK(fpga_clk_c), .Q(phase_active[52])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i52.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i51 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_364), 
            .CK(fpga_clk_c), .Q(phase_active[51])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i51.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i50 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_365), 
            .CK(fpga_clk_c), .Q(phase_active[50])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i50.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i49 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_366), 
            .CK(fpga_clk_c), .Q(phase_active[49])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i49.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i48 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_367), 
            .CK(fpga_clk_c), .Q(phase_active[48])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i48.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i47 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_368), 
            .CK(fpga_clk_c), .Q(phase_active[47])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i47.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i46 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_369), 
            .CK(fpga_clk_c), .Q(phase_active[46])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i46.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i45 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_370), 
            .CK(fpga_clk_c), .Q(phase_active[45])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i45.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i44 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_371), 
            .CK(fpga_clk_c), .Q(phase_active[44])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i44.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i43 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_372), 
            .CK(fpga_clk_c), .Q(phase_active[43])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i43.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i42 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_373), 
            .CK(fpga_clk_c), .Q(phase_active[42])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i42.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i41 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_374), 
            .CK(fpga_clk_c), .Q(phase_active[41])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i41.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i40 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_375), 
            .CK(fpga_clk_c), .Q(phase_active[40])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i40.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i39 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_444), 
            .CK(fpga_clk_c), .Q(phase_active[39])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i39.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i38 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_377), 
            .CK(fpga_clk_c), .Q(phase_active[38])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i38.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i37 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_378), 
            .CK(fpga_clk_c), .Q(phase_active[37])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i37.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i36 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_379), 
            .CK(fpga_clk_c), .Q(phase_active[36])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i36.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i35 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_380), 
            .CK(fpga_clk_c), .Q(phase_active[35])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i35.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i34 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_381), 
            .CK(fpga_clk_c), .Q(phase_active[34])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i34.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i33 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_382), 
            .CK(fpga_clk_c), .Q(phase_active[33])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i33.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i32 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_383), 
            .CK(fpga_clk_c), .Q(phase_active[32])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i32.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i31 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_384), 
            .CK(fpga_clk_c), .Q(phase_active[31])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i31.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i30 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_385), 
            .CK(fpga_clk_c), .Q(phase_active[30])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i30.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i29 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_386), 
            .CK(fpga_clk_c), .Q(phase_active[29])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i29.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i28 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_387), 
            .CK(fpga_clk_c), .Q(phase_active[28])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i28.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i27 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_388), 
            .CK(fpga_clk_c), .Q(phase_active[27])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i27.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i26 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_389), 
            .CK(fpga_clk_c), .Q(phase_active[26])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i26.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i25 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_390), 
            .CK(fpga_clk_c), .Q(phase_active[25])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i25.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i24 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_391), 
            .CK(fpga_clk_c), .Q(phase_active[24])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i24.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i23 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_392), 
            .CK(fpga_clk_c), .Q(phase_active[23])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i23.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i22 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_393), 
            .CK(fpga_clk_c), .Q(phase_active[22])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i22.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i21 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_394), 
            .CK(fpga_clk_c), .Q(phase_active[21])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i21.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i20 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_395), 
            .CK(fpga_clk_c), .Q(phase_active[20])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i20.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i19 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_396), 
            .CK(fpga_clk_c), .Q(phase_active[19])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i19.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i18 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_397), 
            .CK(fpga_clk_c), .Q(phase_active[18])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i18.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i17 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_398), 
            .CK(fpga_clk_c), .Q(phase_active[17])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i17.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i16 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_399), 
            .CK(fpga_clk_c), .Q(phase_active[16])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i16.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i15 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_400), 
            .CK(fpga_clk_c), .Q(phase_active[15])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i15.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i14 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_401), 
            .CK(fpga_clk_c), .Q(phase_active[14])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i14.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i13 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_402), 
            .CK(fpga_clk_c), .Q(phase_active[13])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i13.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i12 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_403), 
            .CK(fpga_clk_c), .Q(phase_active[12])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i12.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i11 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_404), 
            .CK(fpga_clk_c), .Q(phase_active[11])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i11.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i10 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_405), 
            .CK(fpga_clk_c), .Q(phase_active[10])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i10.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i9 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_406), 
            .CK(fpga_clk_c), .Q(phase_active[9])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i9.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i8 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_407), 
            .CK(fpga_clk_c), .Q(phase_active[8])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i8.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i7 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_408), 
            .CK(fpga_clk_c), .Q(phase_active[7])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i7.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i6 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_409), 
            .CK(fpga_clk_c), .Q(phase_active[6])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i6.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i5 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_410), 
            .CK(fpga_clk_c), .Q(phase_active[5])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i5.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i4 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_411), 
            .CK(fpga_clk_c), .Q(phase_active[4])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i4.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i3 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_412), 
            .CK(fpga_clk_c), .Q(phase_active[3])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i3.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i2 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_413), 
            .CK(fpga_clk_c), .Q(phase_active[2])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i2.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i1 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_414), 
            .CK(fpga_clk_c), .Q(phase_active[1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i1.GSR = "ENABLED";
    FD1P3AX phase_active_i83_i0 (.D(phase_mem_dout[7]), .SP(fpga_clk_c_enable_415), 
            .CK(fpga_clk_c), .Q(phase_active[0])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam phase_active_i83_i0.GSR = "ENABLED";
    FD1S3IX time_divider_2862__i1 (.D(n34_adj_3441), .CK(fpga_clk_c), .CD(time_divider_5__N_2269), 
            .Q(time_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862__i1.GSR = "ENABLED";
    LUT4 i15317_2_lut_4_lut (.A(spi_extension_length[10]), .B(spi_rx_shift[1]), 
         .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1313)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15317_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_2_lut_4_lut (.A(n18637), .B(spi_channel_field[0]), .C(fpga_cs_n_c), 
         .D(spi_channel_field[1]), .Z(mem_write_phase)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i1_2_lut_4_lut.init = 16'h0008;
    LUT4 i1_2_lut_4_lut_adj_56 (.A(n18637), .B(spi_channel_field[0]), .C(fpga_cs_n_c), 
         .D(spi_channel_field[1]), .Z(mem_write_level)) /* synthesis lut_function=(!(((C+!(D))+!B)+!A)) */ ;
    defparam i1_2_lut_4_lut_adj_56.init = 16'h0800;
    LUT4 fpga_cs_n_N_339_I_0_1852_2_lut_4_lut (.A(spi_extension_length[9]), 
         .B(spi_rx_shift[0]), .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1262)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1852_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15320_2_lut_4_lut (.A(spi_extension_length[9]), .B(spi_rx_shift[0]), 
         .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1316)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15320_2_lut_4_lut.init = 16'h0035;
    LUT4 equal_2273_i12_2_lut_3_lut_4_lut (.A(load_index[3]), .B(load_index[4]), 
         .C(load_index[6]), .D(load_index[5]), .Z(n12_adj_3423)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2273_i12_2_lut_3_lut_4_lut.init = 16'hffdf;
    CCU2D spi_byte_count_15__I_0_2169_25 (.A0(spi_expected_length[19]), .B0(spi_expected_length[18]), 
          .C0(spi_expected_length[17]), .D0(spi_expected_length[16]), .A1(spi_expected_length[15]), 
          .B1(spi_byte_count[15]), .C1(spi_expected_length[14]), .D1(spi_byte_count[14]), 
          .CIN(n17055), .COUT(n17056));
    defparam spi_byte_count_15__I_0_2169_25.INIT0 = 16'h0001;
    defparam spi_byte_count_15__I_0_2169_25.INIT1 = 16'h9009;
    defparam spi_byte_count_15__I_0_2169_25.INJECT1_0 = "YES";
    defparam spi_byte_count_15__I_0_2169_25.INJECT1_1 = "YES";
    LUT4 i1_4_lut_adj_57 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3370), 
         .C(phase_active[5]), .D(phase_acc[31]), .Z(us_tx_c_5)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_57.init = 16'h0880;
    LUT4 i15997_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3388), .Z(fpga_clk_c_enable_375)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15997_2_lut_4_lut.init = 16'h0002;
    LUT4 i15970_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3418), .Z(fpga_clk_c_enable_407)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15970_2_lut_4_lut.init = 16'h0002;
    LUT4 i16009_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3407), .Z(fpga_clk_c_enable_399)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16009_2_lut_4_lut.init = 16'h0002;
    LUT4 i16006_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3383), .Z(fpga_clk_c_enable_367)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16006_2_lut_4_lut.init = 16'h0002;
    LUT4 i16077_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3403), .Z(fpga_clk_c_enable_391)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16077_2_lut_4_lut.init = 16'h0002;
    CCU2D spi_byte_count_15__I_0_2169_23 (.A0(spi_expected_length[27]), .B0(spi_expected_length[26]), 
          .C0(spi_expected_length[25]), .D0(spi_expected_length[24]), .A1(spi_expected_length[23]), 
          .B1(spi_expected_length[22]), .C1(spi_expected_length[21]), .D1(spi_expected_length[20]), 
          .CIN(n17054), .COUT(n17055));
    defparam spi_byte_count_15__I_0_2169_23.INIT0 = 16'h0001;
    defparam spi_byte_count_15__I_0_2169_23.INIT1 = 16'h0001;
    defparam spi_byte_count_15__I_0_2169_23.INJECT1_0 = "YES";
    defparam spi_byte_count_15__I_0_2169_23.INJECT1_1 = "YES";
    LUT4 i2_4_lut_adj_58 (.A(n26_adj_3510), .B(phase_active[1]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_1)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_58.init = 16'h2080;
    LUT4 i16074_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3378), .Z(fpga_clk_c_enable_359)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16074_2_lut_4_lut.init = 16'h0002;
    LUT4 i30_3_lut_4_lut_adj_59 (.A(amplitude_phase[6]), .B(\level_active[29] [0]), 
         .C(\level_active[29] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3476)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_59.init = 16'h40f4;
    LUT4 i16030_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3402), .Z(fpga_clk_c_enable_383)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16030_2_lut_4_lut.init = 16'h0002;
    LUT4 i16033_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3433), .Z(fpga_clk_c_enable_351)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16033_2_lut_4_lut.init = 16'h0002;
    LUT4 i15931_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3423), .Z(fpga_clk_c_enable_343)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15931_2_lut_4_lut.init = 16'h0002;
    LUT4 i15908_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12), .Z(fpga_clk_c_enable_335)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15908_2_lut_4_lut.init = 16'h0002;
    LUT4 i15889_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3437), 
         .D(n12_adj_3438), .Z(fpga_clk_c_enable_415)) /* synthesis lut_function=(!((B+(C+(D)))+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15889_2_lut_4_lut.init = 16'h0002;
    LUT4 fpga_cs_n_N_339_I_0_1853_2_lut_4_lut (.A(spi_extension_length[8]), 
         .B(spi1_mosi_c), .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1264)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1853_2_lut_4_lut.init = 16'h00ca;
    CCU2D add_2583_7 (.A0(spi_extension_length[6]), .B0(n37_adj_3400), .C0(spi_expected_length[6]), 
          .D0(n7607), .A1(spi_extension_length[7]), .B1(n37_adj_3400), 
          .C1(spi_expected_length[7]), .D1(n7113), .CIN(n17136), .COUT(n17137), 
          .S0(spi_expected_length_31__N_2085[6]), .S1(spi_expected_length_31__N_2085[7]));   // src/umh_fpga_top.v(200[44] 203[90])
    defparam add_2583_7.INIT0 = 16'hd1e2;
    defparam add_2583_7.INIT1 = 16'hd1e2;
    defparam add_2583_7.INJECT1_0 = "NO";
    defparam add_2583_7.INJECT1_1 = "NO";
    LUT4 i15323_2_lut_4_lut (.A(spi_extension_length[8]), .B(spi1_mosi_c), 
         .C(n7859), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1319)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15323_2_lut_4_lut.init = 16'h0035;
    FD1S3IX time_divider_2862__i2 (.D(n33), .CK(fpga_clk_c), .CD(time_divider_5__N_2269), 
            .Q(time_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862__i2.GSR = "ENABLED";
    FD1S3IX time_divider_2862__i3 (.D(n32_adj_3440), .CK(fpga_clk_c), .CD(time_divider_5__N_2269), 
            .Q(time_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862__i3.GSR = "ENABLED";
    FD1S3IX time_divider_2862__i4 (.D(n31), .CK(fpga_clk_c), .CD(time_divider_5__N_2269), 
            .Q(time_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862__i4.GSR = "ENABLED";
    FD1S3IX time_divider_2862__i5 (.D(n30), .CK(fpga_clk_c), .CD(time_divider_5__N_2269), 
            .Q(time_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862__i5.GSR = "ENABLED";
    FD1S3AX amplitude_phase_2864__i1 (.D(n44), .CK(fpga_clk_c), .Q(n7_adj_3411)) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i1.GSR = "ENABLED";
    LUT4 i30_3_lut_4_lut_adj_60 (.A(amplitude_phase[6]), .B(\level_active[77] [0]), 
         .C(\level_active[77] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3499)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_60.init = 16'h40f4;
    CCU2D fpga_time_2861_add_4_25 (.A0(fpga_time[23]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[24]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17209), .COUT(n17210), .S0(n142), .S1(n141));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_25.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_25.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_25.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_25.INJECT1_1 = "NO";
    FD1S3AX amplitude_phase_2864__i2 (.D(n43), .CK(fpga_clk_c), .Q(n6_adj_3410)) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i2.GSR = "ENABLED";
    FD1S3AX amplitude_phase_2864__i3 (.D(n42_adj_3422), .CK(fpga_clk_c), 
            .Q(n5_adj_3409)) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i3.GSR = "ENABLED";
    FD1S3AX amplitude_phase_2864__i4 (.D(n41), .CK(fpga_clk_c), .Q(n4_adj_3408)) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i4.GSR = "ENABLED";
    FD1S3AX amplitude_phase_2864__i5 (.D(n40_adj_3421), .CK(fpga_clk_c), 
            .Q(n3)) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i5.GSR = "ENABLED";
    FD1S3AX amplitude_phase_2864__i6 (.D(n39_adj_3420), .CK(fpga_clk_c), 
            .Q(amplitude_phase[6])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i6.GSR = "ENABLED";
    FD1S3AX amplitude_phase_2864__i7 (.D(n38_adj_3419), .CK(fpga_clk_c), 
            .Q(amplitude_phase[7])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864__i7.GSR = "ENABLED";
    FD1P3AX mic_sample_count_2865__i1 (.D(n29_adj_3406), .SP(mic_divider_6__N_2705), 
            .CK(fpga_clk_c), .Q(mic_sample_count[1]));   // src/umh_fpga_top.v(328[33:56])
    defparam mic_sample_count_2865__i1.GSR = "ENABLED";
    FD1P3AX mic_sample_count_2865__i2 (.D(n28_adj_3405), .SP(mic_divider_6__N_2705), 
            .CK(fpga_clk_c), .Q(mic_sample_count[2]));   // src/umh_fpga_top.v(328[33:56])
    defparam mic_sample_count_2865__i2.GSR = "ENABLED";
    FD1P3AX mic_sample_count_2865__i3 (.D(n27), .SP(mic_divider_6__N_2705), 
            .CK(fpga_clk_c), .Q(mic_sample_count[3]));   // src/umh_fpga_top.v(328[33:56])
    defparam mic_sample_count_2865__i3.GSR = "ENABLED";
    FD1P3AX mic_sample_count_2865__i4 (.D(n26_adj_3404), .SP(mic_divider_6__N_2705), 
            .CK(fpga_clk_c), .Q(mic_sample_count[4]));   // src/umh_fpga_top.v(328[33:56])
    defparam mic_sample_count_2865__i4.GSR = "ENABLED";
    FD1S3IX mic_divider_2866__i1 (.D(n39), .CK(fpga_clk_c), .CD(mic_divider_6__N_2705), 
            .Q(mic_divider[1])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866__i1.GSR = "ENABLED";
    FD1S3IX mic_divider_2866__i2 (.D(n38_adj_3386), .CK(fpga_clk_c), .CD(mic_divider_6__N_2705), 
            .Q(mic_divider[2])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866__i2.GSR = "ENABLED";
    FD1S3IX mic_divider_2866__i3 (.D(n37_adj_3385), .CK(fpga_clk_c), .CD(mic_divider_6__N_2705), 
            .Q(mic_divider[3])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866__i3.GSR = "ENABLED";
    FD1S3IX mic_divider_2866__i4 (.D(n36_adj_3384), .CK(fpga_clk_c), .CD(mic_divider_6__N_2705), 
            .Q(mic_divider[4])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866__i4.GSR = "ENABLED";
    FD1S3IX mic_divider_2866__i5 (.D(n35), .CK(fpga_clk_c), .CD(mic_divider_6__N_2705), 
            .Q(mic_divider[5])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866__i5.GSR = "ENABLED";
    FD1S3IX mic_divider_2866__i6 (.D(n34), .CK(fpga_clk_c), .CD(mic_divider_6__N_2705), 
            .Q(mic_divider[6])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866__i6.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1854_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_extension_length[7]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1266)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1854_2_lut_4_lut.init = 16'h00ca;
    FD1S3DX status_bit_index_2860__i1 (.D(n39_adj_3425), .CK(spi1_sck_N_1872), 
            .CD(fpga_cs_n_c), .Q(status_bit_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860__i1.GSR = "ENABLED";
    CCU2D fpga_time_2861_add_4_23 (.A0(fpga_time[21]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[22]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17208), .COUT(n17209), .S0(n144), .S1(n143));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_23.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_23.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_23.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_23.INJECT1_1 = "NO";
    LUT4 i15326_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_extension_length[7]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1322)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15326_2_lut_4_lut.init = 16'h0035;
    FD1S3DX status_bit_index_2860__i2 (.D(n38_adj_3427), .CK(spi1_sck_N_1872), 
            .CD(fpga_cs_n_c), .Q(status_bit_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860__i2.GSR = "ENABLED";
    FD1S3DX status_bit_index_2860__i3 (.D(n37_adj_3428), .CK(spi1_sck_N_1872), 
            .CD(fpga_cs_n_c), .Q(status_bit_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860__i3.GSR = "ENABLED";
    FD1S3DX status_bit_index_2860__i4 (.D(n36_adj_3429), .CK(spi1_sck_N_1872), 
            .CD(fpga_cs_n_c), .Q(status_bit_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860__i4.GSR = "ENABLED";
    FD1S3DX status_bit_index_2860__i5 (.D(n35_adj_3432), .CK(spi1_sck_N_1872), 
            .CD(fpga_cs_n_c), .Q(status_bit_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860__i5.GSR = "ENABLED";
    FD1S3DX status_bit_index_2860__i6 (.D(n34_adj_3434), .CK(spi1_sck_N_1872), 
            .CD(fpga_cs_n_c), .Q(status_bit_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860__i6.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i1 (.D(n164), .CK(fpga_clk_c), .Q(fpga_time[1])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i1.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i2 (.D(n163), .CK(fpga_clk_c), .Q(fpga_time[2])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i2.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i3 (.D(n162), .CK(fpga_clk_c), .Q(fpga_time[3])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i3.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i4 (.D(n161), .CK(fpga_clk_c), .Q(fpga_time[4])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i4.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i5 (.D(n160), .CK(fpga_clk_c), .Q(fpga_time[5])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i5.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i6 (.D(n159), .CK(fpga_clk_c), .Q(fpga_time[6])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i6.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i7 (.D(n158), .CK(fpga_clk_c), .Q(fpga_time[7])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i7.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i8 (.D(n157), .CK(fpga_clk_c), .Q(fpga_time[8])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i8.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i9 (.D(n156), .CK(fpga_clk_c), .Q(fpga_time[9])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i9.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i10 (.D(n155), .CK(fpga_clk_c), .Q(fpga_time[10])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i10.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i11 (.D(n154), .CK(fpga_clk_c), .Q(fpga_time[11])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i11.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i12 (.D(n153), .CK(fpga_clk_c), .Q(fpga_time[12])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i12.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i13 (.D(n152), .CK(fpga_clk_c), .Q(fpga_time[13])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i13.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i14 (.D(n151), .CK(fpga_clk_c), .Q(fpga_time[14])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i14.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i15 (.D(n150), .CK(fpga_clk_c), .Q(fpga_time[15])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i15.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i16 (.D(n149), .CK(fpga_clk_c), .Q(fpga_time[16])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i16.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i17 (.D(n148), .CK(fpga_clk_c), .Q(fpga_time[17])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i17.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i18 (.D(n147), .CK(fpga_clk_c), .Q(fpga_time[18])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i18.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i19 (.D(n146), .CK(fpga_clk_c), .Q(fpga_time[19])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i19.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i20 (.D(n145), .CK(fpga_clk_c), .Q(fpga_time[20])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i20.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i21 (.D(n144), .CK(fpga_clk_c), .Q(fpga_time[21])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i21.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i22 (.D(n143), .CK(fpga_clk_c), .Q(fpga_time[22])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i22.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i23 (.D(n142), .CK(fpga_clk_c), .Q(fpga_time[23])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i23.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i24 (.D(n141), .CK(fpga_clk_c), .Q(fpga_time[24])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i24.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i25 (.D(n140), .CK(fpga_clk_c), .Q(fpga_time[25])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i25.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i26 (.D(n139), .CK(fpga_clk_c), .Q(fpga_time[26])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i26.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i27 (.D(n138), .CK(fpga_clk_c), .Q(fpga_time[27])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i27.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i28 (.D(n137), .CK(fpga_clk_c), .Q(fpga_time[28])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i28.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i29 (.D(n136), .CK(fpga_clk_c), .Q(fpga_time[29])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i29.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i30 (.D(n135), .CK(fpga_clk_c), .Q(fpga_time[30])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i30.GSR = "ENABLED";
    FD1S3AX fpga_time_2861__i31 (.D(n134), .CK(fpga_clk_c), .Q(fpga_time[31])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861__i31.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i1 (.D(spi_frame_sequence[1]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[1])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i1.GSR = "ENABLED";
    CCU2D fpga_time_2861_add_4_21 (.A0(fpga_time[19]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[20]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17207), .COUT(n17208), .S0(n146), .S1(n145));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_21.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_21.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_21.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_21.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1855_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_extension_length[6]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1268)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1855_2_lut_4_lut.init = 16'h00ca;
    CCU2D fpga_time_2861_add_4_19 (.A0(fpga_time[17]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[18]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17206), .COUT(n17207), .S0(n148), .S1(n147));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_19.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_19.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_19.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_19.INJECT1_1 = "NO";
    LUT4 i15329_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_extension_length[6]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1325)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15329_2_lut_4_lut.init = 16'h0035;
    LUT4 equal_2335_i12_2_lut_3_lut_4_lut (.A(load_index[5]), .B(load_index[6]), 
         .C(load_index[4]), .D(load_index[3]), .Z(n12_adj_3418)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2335_i12_2_lut_3_lut_4_lut.init = 16'hfeff;
    LUT4 equal_2329_i12_2_lut_3_lut_4_lut (.A(load_index[5]), .B(load_index[6]), 
         .C(load_index[4]), .D(load_index[3]), .Z(n12_adj_3407)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2329_i12_2_lut_3_lut_4_lut.init = 16'hffef;
    LUT4 equal_2321_i12_2_lut_3_lut_4_lut (.A(load_index[5]), .B(load_index[6]), 
         .C(load_index[4]), .D(load_index[3]), .Z(n12_adj_3403)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2321_i12_2_lut_3_lut_4_lut.init = 16'hefff;
    LUT4 i16000_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3438), .Z(fpga_clk_c_enable_410)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16000_2_lut_4_lut.init = 16'h0008;
    CCU2D fpga_time_2861_add_4_17 (.A0(fpga_time[15]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[16]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17205), .COUT(n17206), .S0(n150), .S1(n149));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_17.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_17.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_17.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_17.INJECT1_1 = "NO";
    LUT4 i16025_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3418), .Z(fpga_clk_c_enable_402)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16025_2_lut_4_lut.init = 16'h0008;
    LUT4 i16019_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3388), .Z(fpga_clk_c_enable_370)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16019_2_lut_4_lut.init = 16'h0008;
    LUT4 fpga_cs_n_N_339_I_0_1970_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[3]), 
         .Z(spi_expected_length_31__N_1594)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1970_2_lut.init = 16'h4444;
    LUT4 i15842_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3407), .Z(fpga_clk_c_enable_394)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15842_2_lut_4_lut.init = 16'h0008;
    LUT4 i15839_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3383), .Z(fpga_clk_c_enable_362)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15839_2_lut_4_lut.init = 16'h0008;
    LUT4 i15422_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[7]), 
         .Z(spi_expected_length_31__N_1706)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15422_2_lut.init = 16'h1111;
    LUT4 i16047_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3403), .Z(fpga_clk_c_enable_386)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16047_2_lut_4_lut.init = 16'h0008;
    LUT4 i16044_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3378), .Z(fpga_clk_c_enable_354)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i16044_2_lut_4_lut.init = 16'h0008;
    LUT4 i15958_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3402), .Z(fpga_clk_c_enable_378)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15958_2_lut_4_lut.init = 16'h0008;
    LUT4 i15920_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3433), .Z(fpga_clk_c_enable_346)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15920_2_lut_4_lut.init = 16'h0008;
    LUT4 i15911_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n8_adj_3435), 
         .D(n12_adj_3423), .Z(fpga_clk_c_enable_338)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i15911_2_lut_4_lut.init = 16'h0008;
    FD1P3BX rgb_values_i18_5917_5918_set (.D(n10130), .SP(spi1_sck_c_enable_78), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_493), .Q(n10129)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i18_5917_5918_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i0_5041_5042_set (.D(n9254), .SP(spi1_sck_c_enable_79), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_529), .Q(n9253)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i0_5041_5042_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i17_5921_5922_set (.D(n10134), .SP(spi1_sck_c_enable_80), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_495), .Q(n10133)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i17_5921_5922_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i16_5925_5926_set (.D(n10138), .SP(spi1_sck_c_enable_81), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_497), .Q(n10137)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i16_5925_5926_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i15_5929_5930_set (.D(n10142), .SP(spi1_sck_c_enable_82), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_499), .Q(n10141)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i15_5929_5930_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i14_5933_5934_set (.D(n10146), .SP(spi1_sck_c_enable_83), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_501), .Q(n10145)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i14_5933_5934_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i13_5937_5938_set (.D(n10150), .SP(spi1_sck_c_enable_84), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_503), .Q(n10149)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i13_5937_5938_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i12_5941_5942_set (.D(n10154), .SP(spi1_sck_c_enable_85), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_505), .Q(n10153)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i12_5941_5942_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i11_5945_5946_set (.D(n10158), .SP(spi1_sck_c_enable_86), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_507), .Q(n10157)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i11_5945_5946_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i10_5949_5950_set (.D(n10162), .SP(spi1_sck_c_enable_87), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_509), .Q(n10161)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i10_5949_5950_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i9_5953_5954_set (.D(n10166), .SP(spi1_sck_c_enable_88), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_511), .Q(n10165)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i9_5953_5954_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i8_5957_5958_set (.D(n10170), .SP(spi1_sck_c_enable_89), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_513), .Q(n10169)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i8_5957_5958_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i7_5961_5962_set (.D(n10174), .SP(spi1_sck_c_enable_90), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_515), .Q(n10173)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i7_5961_5962_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i6_5965_5966_set (.D(n10178), .SP(spi1_sck_c_enable_91), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_517), .Q(n10177)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i6_5965_5966_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i5_5969_5970_set (.D(n10182), .SP(spi1_sck_c_enable_92), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_519), .Q(n10181)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i5_5969_5970_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i4_5973_5974_set (.D(n10186), .SP(spi1_sck_c_enable_93), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_521), .Q(n10185)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i4_5973_5974_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i3_5977_5978_set (.D(n10190), .SP(spi1_sck_c_enable_94), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_523), .Q(n10189)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i3_5977_5978_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i2_5981_5982_set (.D(n10194), .SP(spi1_sck_c_enable_95), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_525), .Q(n10193)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i2_5981_5982_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i1_5985_5986_set (.D(n10198), .SP(spi1_sck_c_enable_96), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_527), .Q(n10197)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i1_5985_5986_set.GSR = "ENABLED";
    FD1P3IX load_index_2863__i6 (.D(n34_adj_3397), .SP(fpga_clk_c_enable_441), 
            .CD(n10305), .CK(fpga_clk_c), .Q(load_index[6])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863__i6.GSR = "ENABLED";
    FD1P3IX load_index_2863__i5 (.D(n35_adj_3439), .SP(fpga_clk_c_enable_441), 
            .CD(n10305), .CK(fpga_clk_c), .Q(load_index[5])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863__i5.GSR = "ENABLED";
    FD1P3IX load_index_2863__i4 (.D(n36_adj_3396), .SP(fpga_clk_c_enable_441), 
            .CD(n10305), .CK(fpga_clk_c), .Q(load_index[4])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863__i4.GSR = "ENABLED";
    FD1P3IX load_index_2863__i3 (.D(n37_adj_3394), .SP(fpga_clk_c_enable_441), 
            .CD(n10305), .CK(fpga_clk_c), .Q(load_index[3])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863__i3.GSR = "ENABLED";
    FD1P3IX load_index_2863__i2 (.D(n38_adj_3393), .SP(fpga_clk_c_enable_441), 
            .CD(n10305), .CK(fpga_clk_c), .Q(load_index[2])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863__i2.GSR = "ENABLED";
    FD1P3IX load_index_2863__i1 (.D(n39_adj_3430), .SP(fpga_clk_c_enable_441), 
            .CD(n10305), .CK(fpga_clk_c), .Q(load_index[1])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863__i1.GSR = "ENABLED";
    FD1P3AX stop_toggle_spi_1507 (.D(stop_toggle_spi_N_2877), .SP(fpga_cs_n_c_enable_4), 
            .CK(fpga_cs_n_c), .Q(stop_toggle_spi)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam stop_toggle_spi_1507.GSR = "ENABLED";
    FD1P3BX rgb_values_i95_5609_5610_set (.D(n9822), .SP(spi1_sck_c_enable_97), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_337), .Q(n9821)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i95_5609_5610_set.GSR = "ENABLED";
    FD1P3IX load_index_2863__i0 (.D(n40), .SP(fpga_clk_c_enable_441), .CD(n10305), 
            .CK(fpga_clk_c), .Q(load_index[0])) /* synthesis syn_use_carry_chain=1 */ ;   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863__i0.GSR = "ENABLED";
    FD1P3BX rgb_values_i94_5613_5614_set (.D(n9826), .SP(spi1_sck_c_enable_98), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_341), .Q(n9825)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i94_5613_5614_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i93_5617_5618_set (.D(n9830), .SP(spi1_sck_c_enable_99), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_343), .Q(n9829)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i93_5617_5618_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i92_5621_5622_set (.D(n9834), .SP(spi1_sck_c_enable_100), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_345), .Q(n9833)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i92_5621_5622_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i91_5625_5626_set (.D(n9838), .SP(spi1_sck_c_enable_101), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_347), .Q(n9837)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i91_5625_5626_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i90_5629_5630_set (.D(n9842), .SP(spi1_sck_c_enable_102), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_349), .Q(n9841)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i90_5629_5630_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i89_5633_5634_set (.D(n9846), .SP(spi1_sck_c_enable_103), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_351), .Q(n9845)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i89_5633_5634_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i88_5637_5638_set (.D(n9850), .SP(spi1_sck_c_enable_104), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_353), .Q(n9849)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i88_5637_5638_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i87_5641_5642_set (.D(n9854), .SP(spi1_sck_c_enable_105), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_355), .Q(n9853)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i87_5641_5642_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i86_5645_5646_set (.D(n9858), .SP(spi1_sck_c_enable_106), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_357), .Q(n9857)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i86_5645_5646_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i85_5649_5650_set (.D(n9862), .SP(spi1_sck_c_enable_107), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_359), .Q(n9861)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i85_5649_5650_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i84_5653_5654_set (.D(n9866), .SP(spi1_sck_c_enable_108), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_361), .Q(n9865)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i84_5653_5654_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i83_5657_5658_set (.D(n9870), .SP(spi1_sck_c_enable_109), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_363), .Q(n9869)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i83_5657_5658_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i82_5661_5662_set (.D(n9874), .SP(spi1_sck_c_enable_110), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_365), .Q(n9873)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i82_5661_5662_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i81_5665_5666_set (.D(n9878), .SP(spi1_sck_c_enable_111), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_367), .Q(n9877)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i81_5665_5666_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i80_5669_5670_set (.D(n9882), .SP(spi1_sck_c_enable_112), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_369), .Q(n9881)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i80_5669_5670_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i79_5673_5674_set (.D(n9886), .SP(spi1_sck_c_enable_113), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_371), .Q(n9885)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i79_5673_5674_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i78_5677_5678_set (.D(n9890), .SP(spi1_sck_c_enable_114), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_373), .Q(n9889)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i78_5677_5678_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i77_5681_5682_set (.D(n9894), .SP(spi1_sck_c_enable_115), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_375), .Q(n9893)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i77_5681_5682_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i76_5685_5686_set (.D(n9898), .SP(spi1_sck_c_enable_116), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_377), .Q(n9897)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i76_5685_5686_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i75_5689_5690_set (.D(n9902), .SP(spi1_sck_c_enable_117), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_379), .Q(n9901)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i75_5689_5690_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i74_5693_5694_set (.D(n9906), .SP(spi1_sck_c_enable_118), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_381), .Q(n9905)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i74_5693_5694_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i73_5697_5698_set (.D(n9910), .SP(spi1_sck_c_enable_119), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_383), .Q(n9909)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i73_5697_5698_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i72_5701_5702_set (.D(n9914), .SP(spi1_sck_c_enable_120), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_385), .Q(n9913)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i72_5701_5702_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1856_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_extension_length[5]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1270)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1856_2_lut_4_lut.init = 16'h00ca;
    CCU2D fpga_time_2861_add_4_15 (.A0(fpga_time[13]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[14]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17204), .COUT(n17205), .S0(n152), .S1(n151));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_15.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_15.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_15.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_15.INJECT1_1 = "NO";
    LUT4 i5659_3_lut (.A(n9870), .B(n9869), .C(spi1_sck_c_enable_65), 
         .Z(rgb_values[83])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5659_3_lut.init = 16'hcaca;
    CCU2D fpga_time_2861_add_4_13 (.A0(fpga_time[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17203), .COUT(n17204), .S0(n154), .S1(n153));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_13.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_13.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_13.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_13.INJECT1_1 = "NO";
    LUT4 i15332_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_extension_length[5]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1328)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15332_2_lut_4_lut.init = 16'h0035;
    LUT4 equal_2305_i12_2_lut_3_lut_4_lut (.A(load_index[5]), .B(load_index[6]), 
         .C(load_index[4]), .D(load_index[3]), .Z(n12_adj_3388)) /* synthesis lut_function=((B+(C+!(D)))+!A) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2305_i12_2_lut_3_lut_4_lut.init = 16'hfdff;
    FD1P3AX accepted_sequence_spi_i0_i2 (.D(spi_frame_sequence[2]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[2])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i2.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i3 (.D(spi_frame_sequence[3]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[3])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i3.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i4 (.D(spi_frame_sequence[4]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[4])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i4.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i5 (.D(spi_frame_sequence[5]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[5])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i5.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i6 (.D(spi_frame_sequence[6]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[6])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i6.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i7 (.D(spi_frame_sequence[7]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[7])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i7.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i8 (.D(spi_frame_sequence[8]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[8])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i8.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i9 (.D(spi_frame_sequence[9]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[9])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i9.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i10 (.D(spi_frame_sequence[10]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[10])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i10.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i11 (.D(spi_frame_sequence[11]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[11])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i11.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i12 (.D(spi_frame_sequence[12]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[12])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i12.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i13 (.D(spi_frame_sequence[13]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[13])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i13.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i14 (.D(spi_frame_sequence[14]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[14])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i14.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i15 (.D(spi_frame_sequence[15]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[15])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i15.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i16 (.D(spi_frame_sequence[16]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[16])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i16.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i17 (.D(spi_frame_sequence[17]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[17])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i17.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i18 (.D(spi_frame_sequence[18]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[18])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i18.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i19 (.D(spi_frame_sequence[19]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[19])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i19.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i20 (.D(spi_frame_sequence[20]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[20])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i20.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i21 (.D(spi_frame_sequence[21]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[21])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i21.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i22 (.D(spi_frame_sequence[22]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[22])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i22.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i23 (.D(spi_frame_sequence[23]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[23])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i23.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i24 (.D(spi_frame_sequence[24]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[24])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i24.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i25 (.D(spi_frame_sequence[25]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[25])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i25.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i26 (.D(spi_frame_sequence[26]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[26])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i26.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i27 (.D(spi_frame_sequence[27]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[27])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i27.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i28 (.D(spi_frame_sequence[28]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[28])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i28.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i29 (.D(spi_frame_sequence[29]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[29])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i29.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i30 (.D(spi_frame_sequence[30]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[30])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i30.GSR = "ENABLED";
    FD1P3AX accepted_sequence_spi_i0_i31 (.D(spi_frame_sequence[31]), .SP(fpga_cs_n_c_enable_34), 
            .CK(fpga_cs_n_c), .Q(accepted_sequence_spi[31])) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam accepted_sequence_spi_i0_i31.GSR = "ENABLED";
    FD1P3DX rgb_values_i1_5985_5986_reset (.D(n10197), .SP(spi1_sck_c_enable_121), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_910), .Q(n10198)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i1_5985_5986_reset.GSR = "ENABLED";
    FD1P3BX rgb_values_i71_5705_5706_set (.D(n9918), .SP(spi1_sck_c_enable_122), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_387), .Q(n9917)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i71_5705_5706_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i70_5709_5710_set (.D(n9922), .SP(spi1_sck_c_enable_123), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_389), .Q(n9921)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i70_5709_5710_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i69_5713_5714_set (.D(n9926), .SP(spi1_sck_c_enable_124), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_391), .Q(n9925)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i69_5713_5714_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i68_5717_5718_set (.D(n9930), .SP(spi1_sck_c_enable_125), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_393), .Q(n9929)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i68_5717_5718_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i67_5721_5722_set (.D(n9934), .SP(spi1_sck_c_enable_126), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_395), .Q(n9933)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i67_5721_5722_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i66_5725_5726_set (.D(n9938), .SP(spi1_sck_c_enable_127), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_397), .Q(n9937)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i66_5725_5726_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i65_5729_5730_set (.D(n9942), .SP(spi1_sck_c_enable_128), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_399), .Q(n9941)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i65_5729_5730_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i64_5733_5734_set (.D(n9946), .SP(spi1_sck_c_enable_129), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_401), .Q(n9945)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i64_5733_5734_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i63_5737_5738_set (.D(n9950), .SP(spi1_sck_c_enable_130), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_403), .Q(n9949)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i63_5737_5738_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i62_5741_5742_set (.D(n9954), .SP(spi1_sck_c_enable_131), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_405), .Q(n9953)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i62_5741_5742_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i61_5745_5746_set (.D(n9958), .SP(spi1_sck_c_enable_132), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_407), .Q(n9957)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i61_5745_5746_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i60_5749_5750_set (.D(n9962), .SP(spi1_sck_c_enable_133), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_409), .Q(n9961)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i60_5749_5750_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i59_5753_5754_set (.D(n9966), .SP(spi1_sck_c_enable_134), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_411), .Q(n9965)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i59_5753_5754_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i58_5757_5758_set (.D(n9970), .SP(spi1_sck_c_enable_135), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_413), .Q(n9969)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i58_5757_5758_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i57_5761_5762_set (.D(n9974), .SP(spi1_sck_c_enable_136), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_415), .Q(n9973)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i57_5761_5762_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i56_5765_5766_set (.D(n9978), .SP(spi1_sck_c_enable_137), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_417), .Q(n9977)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i56_5765_5766_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i55_5769_5770_set (.D(n9982), .SP(spi1_sck_c_enable_138), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_419), .Q(n9981)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i55_5769_5770_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i54_5773_5774_set (.D(n9986), .SP(spi1_sck_c_enable_139), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_421), .Q(n9985)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i54_5773_5774_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i53_5777_5778_set (.D(n9990), .SP(spi1_sck_c_enable_140), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_423), .Q(n9989)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i53_5777_5778_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i52_5781_5782_set (.D(n9994), .SP(spi1_sck_c_enable_141), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_425), .Q(n9993)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i52_5781_5782_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i51_5785_5786_set (.D(n9998), .SP(spi1_sck_c_enable_142), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_427), .Q(n9997)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i51_5785_5786_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i50_5789_5790_set (.D(n10002), .SP(spi1_sck_c_enable_143), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_429), .Q(n10001)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i50_5789_5790_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i49_5793_5794_set (.D(n10006), .SP(spi1_sck_c_enable_144), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_431), .Q(n10005)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i49_5793_5794_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i48_5797_5798_set (.D(n10010), .SP(spi1_sck_c_enable_145), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_433), .Q(n10009)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i48_5797_5798_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i47_5801_5802_set (.D(n10014), .SP(spi1_sck_c_enable_146), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_435), .Q(n10013)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i47_5801_5802_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i46_5805_5806_set (.D(n10018), .SP(spi1_sck_c_enable_147), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_437), .Q(n10017)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i46_5805_5806_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i45_5809_5810_set (.D(n10022), .SP(spi1_sck_c_enable_148), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_439), .Q(n10021)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i45_5809_5810_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i44_5813_5814_set (.D(n10026), .SP(spi1_sck_c_enable_149), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_441), .Q(n10025)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i44_5813_5814_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i43_5817_5818_set (.D(n10030), .SP(spi1_sck_c_enable_150), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_443), .Q(n10029)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i43_5817_5818_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i42_5821_5822_set (.D(n10034), .SP(spi1_sck_c_enable_151), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_445), .Q(n10033)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i42_5821_5822_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i41_5825_5826_set (.D(n10038), .SP(spi1_sck_c_enable_152), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_447), .Q(n10037)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i41_5825_5826_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i40_5829_5830_set (.D(n10042), .SP(spi1_sck_c_enable_153), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_449), .Q(n10041)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i40_5829_5830_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i39_5833_5834_set (.D(n10046), .SP(spi1_sck_c_enable_154), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_451), .Q(n10045)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i39_5833_5834_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i38_5837_5838_set (.D(n10050), .SP(spi1_sck_c_enable_155), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_453), .Q(n10049)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i38_5837_5838_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i37_5841_5842_set (.D(n10054), .SP(spi1_sck_c_enable_156), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_455), .Q(n10053)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i37_5841_5842_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i36_5845_5846_set (.D(n10058), .SP(spi1_sck_c_enable_157), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_457), .Q(n10057)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i36_5845_5846_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i35_5849_5850_set (.D(n10062), .SP(spi1_sck_c_enable_158), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_459), .Q(n10061)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i35_5849_5850_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i34_5853_5854_set (.D(n10066), .SP(spi1_sck_c_enable_159), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_461), .Q(n10065)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i34_5853_5854_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i33_5857_5858_set (.D(n10070), .SP(spi1_sck_c_enable_160), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_463), .Q(n10069)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i33_5857_5858_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i32_5861_5862_set (.D(n10074), .SP(spi1_sck_c_enable_161), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_465), .Q(n10073)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i32_5861_5862_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i31_5865_5866_set (.D(n10078), .SP(spi1_sck_c_enable_162), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_467), .Q(n10077)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i31_5865_5866_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i30_5869_5870_set (.D(n10082), .SP(spi1_sck_c_enable_163), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_469), .Q(n10081)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i30_5869_5870_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i29_5873_5874_set (.D(n10086), .SP(spi1_sck_c_enable_164), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_471), .Q(n10085)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i29_5873_5874_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i28_5877_5878_set (.D(n10090), .SP(spi1_sck_c_enable_165), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_473), .Q(n10089)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i28_5877_5878_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i27_5881_5882_set (.D(n10094), .SP(spi1_sck_c_enable_166), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_475), .Q(n10093)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i27_5881_5882_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i26_5885_5886_set (.D(n10098), .SP(spi1_sck_c_enable_167), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_477), .Q(n10097)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i26_5885_5886_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i25_5889_5890_set (.D(n10102), .SP(spi1_sck_c_enable_168), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_479), .Q(n10101)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i25_5889_5890_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i24_5893_5894_set (.D(n10106), .SP(spi1_sck_c_enable_169), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_481), .Q(n10105)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i24_5893_5894_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i23_5897_5898_set (.D(n10110), .SP(spi1_sck_c_enable_170), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_483), .Q(n10109)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i23_5897_5898_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i22_5901_5902_set (.D(n10114), .SP(spi1_sck_c_enable_171), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_485), .Q(n10113)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i22_5901_5902_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i21_5905_5906_set (.D(n10118), .SP(spi1_sck_c_enable_172), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_487), .Q(n10117)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i21_5905_5906_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i20_5909_5910_set (.D(n10122), .SP(spi1_sck_c_enable_173), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_489), .Q(n10121)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i20_5909_5910_set.GSR = "ENABLED";
    FD1P3BX rgb_values_i19_5913_5914_set (.D(n10126), .SP(spi1_sck_c_enable_174), 
            .CK(spi1_sck_c), .PD(rgb_values_95__N_491), .Q(n10125)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i19_5913_5914_set.GSR = "ENABLED";
    FD1P3AX level_active_83___i90 (.D(n2315), .SP(fpga_clk_c_enable_444), 
            .CK(fpga_clk_c), .Q(\level_active[39] [1])) /* synthesis lse_init_val=0, syn_ramstyle="registers" */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam level_active_83___i90.GSR = "ENABLED";
    FD1P3DX rgb_values_i2_5981_5982_reset (.D(n10193), .SP(spi1_sck_c_enable_175), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_907), .Q(n10194)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i2_5981_5982_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i3_5977_5978_reset (.D(n10189), .SP(spi1_sck_c_enable_176), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_904), .Q(n10190)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i3_5977_5978_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i4_5973_5974_reset (.D(n10185), .SP(spi1_sck_c_enable_177), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_901), .Q(n10186)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i4_5973_5974_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i5_5969_5970_reset (.D(n10181), .SP(spi1_sck_c_enable_178), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_898), .Q(n10182)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i5_5969_5970_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i6_5965_5966_reset (.D(n10177), .SP(spi1_sck_c_enable_179), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_895), .Q(n10178)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i6_5965_5966_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i7_5961_5962_reset (.D(n10173), .SP(spi1_sck_c_enable_180), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_892), .Q(n10174)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i7_5961_5962_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i8_5957_5958_reset (.D(n10169), .SP(spi1_sck_c_enable_181), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_889), .Q(n10170)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i8_5957_5958_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i9_5953_5954_reset (.D(n10165), .SP(spi1_sck_c_enable_182), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_886), .Q(n10166)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i9_5953_5954_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i10_5949_5950_reset (.D(n10161), .SP(spi1_sck_c_enable_183), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_883), .Q(n10162)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i10_5949_5950_reset.GSR = "ENABLED";
    LUT4 equal_2288_i12_2_lut_3_lut_4_lut (.A(load_index[5]), .B(load_index[6]), 
         .C(load_index[4]), .D(load_index[3]), .Z(n12_adj_3378)) /* synthesis lut_function=((B+!(C (D)))+!A) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2288_i12_2_lut_3_lut_4_lut.init = 16'hdfff;
    CCU2D fpga_time_2861_add_4_11 (.A0(fpga_time[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17202), .COUT(n17203), .S0(n156), .S1(n155));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_11.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_11.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_11.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_11.INJECT1_1 = "NO";
    LUT4 equal_2298_i12_2_lut_3_lut_4_lut (.A(load_index[5]), .B(load_index[6]), 
         .C(load_index[4]), .D(load_index[3]), .Z(n12_adj_3383)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2298_i12_2_lut_3_lut_4_lut.init = 16'hffdf;
    LUT4 i2_4_lut_adj_61 (.A(n26_adj_3382), .B(phase_active[6]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_6)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_61.init = 16'h2080;
    LUT4 equal_2346_i12_2_lut_3_lut_4_lut (.A(load_index[3]), .B(load_index[4]), 
         .C(load_index[6]), .D(load_index[5]), .Z(n12_adj_3438)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2346_i12_2_lut_3_lut_4_lut.init = 16'hfffe;
    LUT4 equal_2314_i12_2_lut_3_lut_4_lut (.A(load_index[3]), .B(load_index[4]), 
         .C(load_index[6]), .D(load_index[5]), .Z(n12_adj_3402)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2314_i12_2_lut_3_lut_4_lut.init = 16'hfeff;
    LUT4 equal_2282_i12_2_lut_3_lut_4_lut (.A(load_index[3]), .B(load_index[4]), 
         .C(load_index[6]), .D(load_index[5]), .Z(n12_adj_3433)) /* synthesis lut_function=(A+(B+((D)+!C))) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2282_i12_2_lut_3_lut_4_lut.init = 16'hffef;
    CCU2D fpga_time_2861_add_4_9 (.A0(fpga_time[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17201), .COUT(n17202), .S0(n158), .S1(n157));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_9.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_9.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_9.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_9.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1965_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[8]), 
         .Z(spi_expected_length_31__N_1584)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1965_2_lut.init = 16'h4444;
    LUT4 i16079_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3402), .Z(fpga_clk_c_enable_444)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i16079_2_lut_4_lut.init = 16'h0080;
    LUT4 i15972_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3438), .Z(fpga_clk_c_enable_408)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i15972_2_lut_4_lut.init = 16'h0080;
    LUT4 i16013_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3418), .Z(fpga_clk_c_enable_400)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i16013_2_lut_4_lut.init = 16'h0080;
    LUT4 i16011_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3388), .Z(fpga_clk_c_enable_368)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i16011_2_lut_4_lut.init = 16'h0080;
    LUT4 i15095_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3407), .Z(fpga_clk_c_enable_392)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i15095_2_lut_4_lut.init = 16'h0080;
    LUT4 i16081_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3383), .Z(fpga_clk_c_enable_360)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i16081_2_lut_4_lut.init = 16'h0080;
    LUT4 i15093_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3403), .Z(fpga_clk_c_enable_384)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i15093_2_lut_4_lut.init = 16'h0080;
    LUT4 i16027_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3378), .Z(fpga_clk_c_enable_352)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i16027_2_lut_4_lut.init = 16'h0080;
    LUT4 i15928_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3433), .Z(fpga_clk_c_enable_344)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i15928_2_lut_4_lut.init = 16'h0080;
    LUT4 i15902_2_lut_4_lut (.A(load_active), .B(load_index[0]), .C(n13820), 
         .D(n12_adj_3423), .Z(fpga_clk_c_enable_336)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i15902_2_lut_4_lut.init = 16'h0080;
    LUT4 fpga_cs_n_N_339_I_0_1857_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_extension_length[4]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1272)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1857_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9749_2_lut (.A(n9397), .B(n9396), .Z(spi_expected_length[24])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9749_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_62 (.A(phase_active[7]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3375), .Z(us_tx_c_7)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_62.init = 16'h4800;
    LUT4 i15335_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_extension_length[4]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1331)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15335_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1858_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_extension_length[3]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1274)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1858_2_lut_4_lut.init = 16'h00ca;
    CCU2D fpga_time_2861_add_4_7 (.A0(fpga_time[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17200), .COUT(n17201), .S0(n160), .S1(n159));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_7.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_7.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_7.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_7.INJECT1_1 = "NO";
    CCU2D fpga_time_2861_add_4_5 (.A0(fpga_time[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17199), .COUT(n17200), .S0(n162), .S1(n161));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_5.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_5.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_5.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_5.INJECT1_1 = "NO";
    LUT4 i15338_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_extension_length[3]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1334)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15338_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1859_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_extension_length[2]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1276)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1859_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15341_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_extension_length[2]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1337)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15341_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1860_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_expected_length_31__N_2085[1]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1278)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1860_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15344_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_expected_length_31__N_2085[1]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1340)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15344_2_lut_4_lut.init = 16'h0035;
    CCU2D fpga_time_2861_add_4_3 (.A0(fpga_time[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(fpga_time[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17198), .COUT(n17199), .S0(n164), .S1(n163));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_3.INIT0 = 16'hfaaa;
    defparam fpga_time_2861_add_4_3.INIT1 = 16'hfaaa;
    defparam fpga_time_2861_add_4_3.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_3.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1897_2_lut_4_lut (.A(spi_frame_sequence[12]), 
         .B(spi_rx_shift[3]), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1384)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1897_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15256_2_lut_4_lut (.A(spi_frame_sequence[12]), .B(spi_rx_shift[3]), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1499)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15256_2_lut_4_lut.init = 16'h0035;
    FD1P3DX rgb_values_i11_5945_5946_reset (.D(n10157), .SP(spi1_sck_c_enable_184), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_880), .Q(n10158)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i11_5945_5946_reset.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1828_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_update_flags[1]), 
         .C(n7626), .D(fpga_cs_n_c), .Z(spi_update_flags_15__N_1182)) /* synthesis lut_function=(!(A (B (C+(D))+!B (D))+!A (B+((D)+!C)))) */ ;
    defparam fpga_cs_n_N_339_I_0_1828_2_lut_4_lut.init = 16'h003a;
    FD1P3DX rgb_values_i12_5941_5942_reset (.D(n10153), .SP(spi1_sck_c_enable_185), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_877), .Q(n10154)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i12_5941_5942_reset.GSR = "ENABLED";
    LUT4 i15380_2_lut_3_lut (.A(spi_expected_length[21]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1664)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15380_2_lut_3_lut.init = 16'h0707;
    LUT4 i30_3_lut_4_lut_adj_63 (.A(amplitude_phase[6]), .B(\level_active[40] [0]), 
         .C(\level_active[40] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3390)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_63.init = 16'h40f4;
    FD1P3DX rgb_values_i13_5937_5938_reset (.D(n10149), .SP(spi1_sck_c_enable_186), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_874), .Q(n10150)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i13_5937_5938_reset.GSR = "ENABLED";
    CCU2D fpga_time_2861_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n8892), .B1(n4_adj_3443), .C1(fpga_time[0]), 
          .D1(GND_net), .COUT(n17198), .S1(n165));   // src/umh_fpga_top.v(281[22:38])
    defparam fpga_time_2861_add_4_1.INIT0 = 16'hF000;
    defparam fpga_time_2861_add_4_1.INIT1 = 16'he1e1;
    defparam fpga_time_2861_add_4_1.INJECT1_0 = "NO";
    defparam fpga_time_2861_add_4_1.INJECT1_1 = "NO";
    LUT4 i15296_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_update_flags[1]), 
         .C(n7626), .D(fpga_cs_n_c), .Z(spi_update_flags_15__N_1244)) /* synthesis lut_function=(!(A (((D)+!C)+!B)+!A (B (D)+!B (C+(D))))) */ ;
    defparam i15296_2_lut_4_lut.init = 16'h00c5;
    FD1P3DX rgb_values_i14_5933_5934_reset (.D(n10145), .SP(spi1_sck_c_enable_187), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_871), .Q(n10146)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i14_5933_5934_reset.GSR = "ENABLED";
    FD1P3DX rgb_values_i15_5929_5930_reset (.D(n10141), .SP(spi1_sck_c_enable_188), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_868), .Q(n10142)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i15_5929_5930_reset.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1878_2_lut_4_lut (.A(spi_frame_sequence[31]), 
         .B(spi_rx_shift[6]), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1346)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1878_2_lut_4_lut.init = 16'h00ca;
    FD1P3DX rgb_values_i16_5925_5926_reset (.D(n10137), .SP(spi1_sck_c_enable_189), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_865), .Q(n10138)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i16_5925_5926_reset.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1953_2_lut_3_lut (.A(spi_expected_length[20]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1560)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1953_2_lut_3_lut.init = 16'h0808;
    LUT4 i15199_2_lut_4_lut (.A(spi_frame_sequence[31]), .B(spi_rx_shift[6]), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1410)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15199_2_lut_4_lut.init = 16'h0035;
    FD1P3DX rgb_values_i17_5921_5922_reset (.D(n10133), .SP(spi1_sck_c_enable_190), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_862), .Q(n10134)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i17_5921_5922_reset.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i24_5397_5398_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1360), .Q(n9609)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i24_5397_5398_set.GSR = "ENABLED";
    LUT4 i15383_2_lut_3_lut (.A(spi_expected_length[20]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1667)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15383_2_lut_3_lut.init = 16'h0707;
    FD1S3BX spi_extension_length_i7_5457_5458_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1266), .Q(n9669)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i7_5457_5458_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i23_5393_5394_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1362), .Q(n9605)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i23_5393_5394_set.GSR = "ENABLED";
    LUT4 i15099_2_lut_3_lut (.A(n8892), .B(time_divider[4]), .C(time_divider[2]), 
         .Z(time_divider_5__N_2269)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;   // src/umh_fpga_top.v(278[9:52])
    defparam i15099_2_lut_3_lut.init = 16'h0101;
    FD1S3BX spi_extension_length_i6_5453_5454_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1268), .Q(n9665)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i6_5453_5454_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i22_5389_5390_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1364), .Q(n9601)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i22_5389_5390_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i5_5449_5450_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1270), .Q(n9661)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i5_5449_5450_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i21_5385_5386_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1366), .Q(n9597)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i21_5385_5386_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i4_5445_5446_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1272), .Q(n9657)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i4_5445_5446_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i20_5381_5382_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1368), .Q(n9593)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i20_5381_5382_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1879_2_lut_4_lut (.A(spi_frame_sequence[30]), 
         .B(spi_rx_shift[5]), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1348)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1879_2_lut_4_lut.init = 16'h00ca;
    FD1S3BX spi_extension_length_i3_5441_5442_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1274), .Q(n9653)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i3_5441_5442_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i19_5377_5378_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1370), .Q(n9589)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i19_5377_5378_set.GSR = "ENABLED";
    LUT4 i15202_2_lut_4_lut (.A(spi_frame_sequence[30]), .B(spi_rx_shift[5]), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1445)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15202_2_lut_4_lut.init = 16'h0035;
    FD1S3BX spi_extension_length_i2_5437_5438_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1276), .Q(n9649)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i2_5437_5438_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i18_5373_5374_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1372), .Q(n9585)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i18_5373_5374_set.GSR = "ENABLED";
    FD1S3BX spi_extension_length_i1_5433_5434_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_extension_length_15__N_1278), .Q(n9645)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_extension_length_i1_5433_5434_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i17_5369_5370_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1374), .Q(n9581)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i17_5369_5370_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1632_2_lut_4_lut (.A(rgb_values[0]), .B(spi1_mosi_c), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_529)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1632_2_lut_4_lut.init = 16'h00ca;
    FD1S3BX spi_update_flags_i1_5429_5430_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_update_flags_15__N_1182), .Q(n9641)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_update_flags_i1_5429_5430_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i16_5365_5366_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1376), .Q(n9577)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i16_5365_5366_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i31_5425_5426_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1346), .Q(n9637)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i31_5425_5426_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i15_5361_5362_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1378), .Q(n9573)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i15_5361_5362_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i30_5421_5422_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1348), .Q(n9633)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i30_5421_5422_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i14_5357_5358_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1380), .Q(n9569)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i14_5357_5358_set.GSR = "ENABLED";
    LUT4 i2_2_lut_4_lut (.A(n9437), .B(n9436), .C(n9445), .D(n9444), 
         .Z(n10_adj_3515)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;   // src/umh_fpga_top.v(80[29:67])
    defparam i2_2_lut_4_lut.init = 16'hf888;
    LUT4 i15782_2_lut_4_lut (.A(rgb_values[0]), .B(spi1_mosi_c), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_913)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15782_2_lut_4_lut.init = 16'h0035;
    FD1S3BX spi_frame_sequence_i13_5353_5354_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1382), .Q(n9565)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i13_5353_5354_set.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i29_5417_5418_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1350), .Q(n9629)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i29_5417_5418_set.GSR = "ENABLED";
    LUT4 i15494_2_lut_4_lut (.A(fpga_cs_n_c), .B(n58_adj_3416), .C(n18637), 
         .D(spi_channel_field[0]), .Z(spi_channel_field_1__N_1781)) /* synthesis lut_function=(!(A+(B (D)+!B !(C (D)+!C !(D))))) */ ;
    defparam i15494_2_lut_4_lut.init = 16'h1045;
    FD1S3BX spi_rx_shift_i1_5049_5050_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_rx_shift_6__N_922), .Q(n9261)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_rx_shift_i1_5049_5050_set.GSR = "ENABLED";
    LUT4 i2_4_lut_adj_64 (.A(n26_adj_3446), .B(phase_active[44]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_44)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_64.init = 16'h2080;
    LUT4 i13080_2_lut_3_lut (.A(mic_sample_count[2]), .B(n17100), .C(mic_sample_count[3]), 
         .Z(n27)) /* synthesis lut_function=(!(A (B (C)+!B !(C))+!A !(C))) */ ;   // src/umh_fpga_top.v(328[33:56])
    defparam i13080_2_lut_3_lut.init = 16'h7878;
    LUT4 i13087_3_lut_4_lut (.A(mic_sample_count[2]), .B(n17100), .C(mic_sample_count[3]), 
         .D(mic_sample_count[4]), .Z(n26_adj_3404)) /* synthesis lut_function=(!(A (B (C (D)+!C !(D))+!B !(D))+!A !(D))) */ ;   // src/umh_fpga_top.v(328[33:56])
    defparam i13087_3_lut_4_lut.init = 16'h7f80;
    LUT4 i13066_2_lut_3_lut (.A(mic_clk_c), .B(mic_sample_count[0]), .C(mic_sample_count[1]), 
         .Z(n29_adj_3406)) /* synthesis lut_function=(A (C)+!A !(B (C)+!B !(C))) */ ;   // src/umh_fpga_top.v(328[33:56])
    defparam i13066_2_lut_3_lut.init = 16'hb4b4;
    LUT4 fpga_cs_n_N_339_I_0_2021_2_lut_4_lut (.A(fpga_cs_n_c), .B(n58_adj_3416), 
         .C(n18637), .D(spi_channel_field[0]), .Z(spi_channel_field_1__N_1774)) /* synthesis lut_function=(!(A+!(B (D)+!B !(C (D)+!C !(D))))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2021_2_lut_4_lut.init = 16'h4510;
    LUT4 n128_bdd_2_lut_4_lut (.A(n9789), .B(n9788), .C(n9781), .D(n9780), 
         .Z(n20866)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;
    defparam n128_bdd_2_lut_4_lut.init = 16'hf888;
    LUT4 i13100_2_lut_4_lut (.A(n9653), .B(n9652), .C(n7), .D(n8_adj_3389), 
         .Z(n17086)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13100_2_lut_4_lut.init = 16'h0008;
    LUT4 i13069_2_lut_3_lut (.A(mic_clk_c), .B(mic_sample_count[0]), .C(mic_sample_count[1]), 
         .Z(n17100)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(328[33:56])
    defparam i13069_2_lut_3_lut.init = 16'h4040;
    LUT4 fpga_cs_n_N_339_I_0_1896_2_lut_4_lut (.A(spi_frame_sequence[13]), 
         .B(spi_rx_shift[4]), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1382)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1896_2_lut_4_lut.init = 16'h00ca;
    LUT4 i2_3_lut_4_lut_adj_65 (.A(n9019), .B(spi_bit_count[2]), .C(n7166), 
         .D(spi_byte_count[0]), .Z(n37)) /* synthesis lut_function=(A+(((D)+!C)+!B)) */ ;   // src/umh_fpga_top.v(189[17:22])
    defparam i2_3_lut_4_lut_adj_65.init = 16'hffbf;
    LUT4 fpga_cs_n_N_339_I_0_1880_2_lut_4_lut (.A(spi_frame_sequence[29]), 
         .B(spi_rx_shift[4]), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1350)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1880_2_lut_4_lut.init = 16'h00ca;
    CCU2D status_bit_index_2860_add_4_7 (.A0(status_bit_index[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[6]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n17196), .S0(n35_adj_3432), 
          .S1(n34_adj_3434));   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860_add_4_7.INIT0 = 16'hfaaa;
    defparam status_bit_index_2860_add_4_7.INIT1 = 16'hfaaa;
    defparam status_bit_index_2860_add_4_7.INJECT1_0 = "NO";
    defparam status_bit_index_2860_add_4_7.INJECT1_1 = "NO";
    LUT4 i9900_2_lut (.A(n9577), .B(n9576), .Z(spi_frame_sequence[16])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9900_2_lut.init = 16'h8888;
    LUT4 i1_4_lut_adj_66 (.A(status_flags_wire_15__N_2050[4]), .B(n25), 
         .C(phase_active[45]), .D(phase_acc[31]), .Z(us_tx_c_45)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_66.init = 16'h0880;
    LUT4 i15205_2_lut_4_lut (.A(spi_frame_sequence[29]), .B(spi_rx_shift[4]), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1448)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15205_2_lut_4_lut.init = 16'h0035;
    LUT4 i5_3_lut_4_lut (.A(mic_clk_c), .B(mic_divider_6__N_2705), .C(n10_adj_3512), 
         .D(mic_sample_count[0]), .Z(fpga_clk_c_enable_246)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam i5_3_lut_4_lut.init = 16'h4000;
    LUT4 i15253_2_lut_4_lut (.A(spi_frame_sequence[13]), .B(spi_rx_shift[4]), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1496)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15253_2_lut_4_lut.init = 16'h0035;
    LUT4 i3620_2_lut_3_lut (.A(running_N_2903), .B(stop_toggle_sync), .C(stop_toggle_seen), 
         .Z(n7832)) /* synthesis lut_function=(A+!(B (C)+!B !(C))) */ ;   // src/umh_fpga_top.v(313[5] 320[8])
    defparam i3620_2_lut_3_lut.init = 16'hbebe;
    LUT4 fpga_cs_n_N_339_I_0_1881_2_lut_4_lut (.A(spi_frame_sequence[28]), 
         .B(spi_rx_shift[3]), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1352)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1881_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15208_2_lut_4_lut (.A(spi_frame_sequence[28]), .B(spi_rx_shift[3]), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1451)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15208_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1883_2_lut_4_lut (.A(spi_frame_sequence[26]), 
         .B(spi_rx_shift[1]), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1356)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1883_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_2_lut_adj_67 (.A(mic_clk_c), .B(mic_sample_count[0]), .Z(n17436)) /* synthesis lut_function=(A (B)+!A !(B)) */ ;
    defparam i1_2_lut_adj_67.init = 16'h9999;
    LUT4 i15214_2_lut_4_lut (.A(spi_frame_sequence[26]), .B(spi_rx_shift[1]), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1457)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15214_2_lut_4_lut.init = 16'h0035;
    LUT4 i2_4_lut_adj_68 (.A(n26_adj_3450), .B(phase_active[46]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_46)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_68.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1884_2_lut_4_lut (.A(spi_frame_sequence[25]), 
         .B(spi_rx_shift[0]), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1358)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1884_2_lut_4_lut.init = 16'h00ca;
    LUT4 i2912_2_lut_4_lut (.A(spi_byte_count[3]), .B(spi_byte_count[2]), 
         .C(n9773), .D(n9772), .Z(n10)) /* synthesis lut_function=(A+(B+(C (D)))) */ ;
    defparam i2912_2_lut_4_lut.init = 16'hfeee;
    LUT4 frame_toggle_spi_I_0_2_lut (.A(frame_toggle_spi), .B(frame_toggle_spi_N_2870), 
         .Z(frame_toggle_spi_N_2869)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // src/umh_fpga_top.v(262[18] 267[12])
    defparam frame_toggle_spi_I_0_2_lut.init = 16'h6666;
    LUT4 i15217_2_lut_4_lut (.A(spi_frame_sequence[25]), .B(spi_rx_shift[0]), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1460)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15217_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1885_2_lut_4_lut (.A(spi_frame_sequence[24]), 
         .B(spi1_mosi_c), .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1360)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1885_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1954_2_lut_3_lut (.A(spi_expected_length[19]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1562)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1954_2_lut_3_lut.init = 16'h0808;
    LUT4 i15220_2_lut_4_lut (.A(spi_frame_sequence[24]), .B(spi1_mosi_c), 
         .C(n8652), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1463)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15220_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1886_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_frame_sequence[23]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1362)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1886_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15223_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_frame_sequence[23]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1466)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15223_2_lut_4_lut.init = 16'h0035;
    LUT4 i15419_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[8]), 
         .Z(spi_expected_length_31__N_1703)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15419_2_lut.init = 16'h1111;
    LUT4 i30_3_lut_4_lut_adj_69 (.A(amplitude_phase[6]), .B(\level_active[75] [0]), 
         .C(\level_active[75] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3497)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_69.init = 16'h40f4;
    LUT4 i2_4_lut_adj_70 (.A(phase_active[43]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3372), .Z(us_tx_c_43)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_70.init = 16'h4800;
    LUT4 i15386_2_lut_3_lut (.A(spi_expected_length[19]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1670)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15386_2_lut_3_lut.init = 16'h0707;
    LUT4 m1_lut (.Z(n21165)) /* synthesis lut_function=1, syn_instantiated=1 */ ;
    defparam m1_lut.init = 16'hffff;
    LUT4 i1_4_lut_adj_71 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[8]), 
         .C(n26_adj_3377), .D(phase_acc[31]), .Z(us_tx_c_8)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_71.init = 16'h2080;
    LUT4 i30_3_lut_4_lut_adj_72 (.A(amplitude_phase[6]), .B(\level_active[74] [0]), 
         .C(\level_active[74] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3502)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_72.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1785_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_command[4]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1064)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1785_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9909_2_lut_4_lut (.A(n9297), .B(n9296), .C(spi_bit_count[2]), 
         .D(n7166), .Z(n14028)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i9909_2_lut_4_lut.init = 16'h8000;
    LUT4 i2_4_lut_adj_73 (.A(phase_active[47]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3451), .Z(us_tx_c_47)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_73.init = 16'h4800;
    LUT4 i30_3_lut_4_lut_adj_74 (.A(amplitude_phase[6]), .B(\level_active[73] [0]), 
         .C(\level_active[73] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3447)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_74.init = 16'h40f4;
    LUT4 i2_2_lut_3_lut_adj_75 (.A(n9761), .B(n9760), .C(n10), .Z(n7)) /* synthesis lut_function=(((C)+!B)+!A) */ ;   // src/umh_fpga_top.v(198[17:23])
    defparam i2_2_lut_3_lut_adj_75.init = 16'hf7f7;
    LUT4 fpga_cs_n_N_339_I_0_1631_2_lut_4_lut (.A(rgb_values[1]), .B(spi_rx_shift[0]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_527)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1631_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1887_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_frame_sequence[22]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1364)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1887_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15226_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_frame_sequence[22]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1469)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15226_2_lut_4_lut.init = 16'h0035;
    CCU2D status_bit_index_2860_add_4_5 (.A0(status_bit_index[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n17195), .COUT(n17196), .S0(n37_adj_3428), 
          .S1(n36_adj_3429));   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860_add_4_5.INIT0 = 16'hfaaa;
    defparam status_bit_index_2860_add_4_5.INIT1 = 16'hfaaa;
    defparam status_bit_index_2860_add_4_5.INJECT1_0 = "NO";
    defparam status_bit_index_2860_add_4_5.INJECT1_1 = "NO";
    LUT4 i2932_2_lut_4_lut (.A(n9277), .B(n9276), .C(n9641), .D(n9640), 
         .Z(n7113)) /* synthesis lut_function=(A (B (C (D)))) */ ;   // src/umh_fpga_top.v(200[44] 202[63])
    defparam i2932_2_lut_4_lut.init = 16'h8000;
    LUT4 i2943_1_lut_4_lut (.A(n9277), .B(n9276), .C(n9641), .D(n9640), 
         .Z(n7007)) /* synthesis lut_function=(!(A (B (C (D))))) */ ;   // src/umh_fpga_top.v(200[44] 202[63])
    defparam i2943_1_lut_4_lut.init = 16'h7fff;
    LUT4 i1_4_lut_adj_76 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[48]), 
         .C(n26_adj_3454), .D(phase_acc[31]), .Z(us_tx_c_48)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_76.init = 16'h2080;
    LUT4 i30_3_lut_4_lut_adj_77 (.A(amplitude_phase[6]), .B(\level_active[72] [0]), 
         .C(\level_active[72] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3500)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_77.init = 16'h40f4;
    CCU2D status_bit_index_2860_add_4_3 (.A0(status_bit_index[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(status_bit_index[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n17194), .COUT(n17195), .S0(n39_adj_3425), 
          .S1(n38_adj_3427));   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860_add_4_3.INIT0 = 16'hfaaa;
    defparam status_bit_index_2860_add_4_3.INIT1 = 16'hfaaa;
    defparam status_bit_index_2860_add_4_3.INJECT1_0 = "NO";
    defparam status_bit_index_2860_add_4_3.INJECT1_1 = "NO";
    LUT4 i30_3_lut_4_lut_adj_78 (.A(amplitude_phase[6]), .B(\level_active[71] [0]), 
         .C(\level_active[71] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3492)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_78.init = 16'h40f4;
    LUT4 i2930_2_lut_4_lut (.A(n9277), .B(n9276), .C(n9641), .D(n9640), 
         .Z(n7008)) /* synthesis lut_function=(!(A (B (C (D))+!B !(C (D)))+!A !(C (D)))) */ ;   // src/umh_fpga_top.v(200[44] 202[63])
    defparam i2930_2_lut_4_lut.init = 16'h7888;
    LUT4 i30_3_lut_4_lut_adj_79 (.A(amplitude_phase[6]), .B(\level_active[70] [0]), 
         .C(\level_active[70] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3498)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_79.init = 16'h40f4;
    LUT4 i15779_2_lut_4_lut (.A(rgb_values[1]), .B(spi_rx_shift[0]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_910)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15779_2_lut_4_lut.init = 16'h0035;
    LUT4 i30_3_lut_4_lut_adj_80 (.A(amplitude_phase[6]), .B(\level_active[69] [0]), 
         .C(\level_active[69] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3494)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_80.init = 16'h40f4;
    LUT4 i15130_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9453), .C(n9452), .Z(spi_rx_shift_6__N_923)) /* synthesis lut_function=(!(A+(B (C)))) */ ;
    defparam i15130_2_lut_3_lut.init = 16'h1515;
    LUT4 i30_3_lut_4_lut_adj_81 (.A(amplitude_phase[6]), .B(\level_active[68] [0]), 
         .C(\level_active[68] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3495)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_81.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1730_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9453), 
         .C(n9452), .Z(spi_rx_shift_6__N_916)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1730_2_lut_3_lut.init = 16'h4040;
    LUT4 i2_4_lut_adj_82 (.A(phase_active[49]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3455), .Z(us_tx_c_49)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_82.init = 16'h4800;
    LUT4 i15133_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9449), .C(n9448), .Z(spi_rx_shift_5__N_926)) /* synthesis lut_function=(!(A+(B (C)))) */ ;
    defparam i15133_2_lut_3_lut.init = 16'h1515;
    LUT4 n128_bdd_4_lut (.A(n128), .B(spi_byte_count[4]), .C(spi_byte_count[8]), 
         .D(spi_byte_count[6]), .Z(n20865)) /* synthesis lut_function=(!(A (B (C (D)))+!A !(B+(C+(D))))) */ ;
    defparam n128_bdd_4_lut.init = 16'h7ffe;
    LUT4 fpga_cs_n_N_339_I_0_1731_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9449), 
         .C(n9448), .Z(spi_rx_shift_6__N_917)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1731_2_lut_3_lut.init = 16'h4040;
    CCU2D status_bit_index_2860_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n13921), .B1(n18642), .C1(status_bit_index[0]), 
          .D1(GND_net), .COUT(n17194), .S1(n40_adj_3424));   // src/umh_fpga_top.v(177[29:52])
    defparam status_bit_index_2860_add_4_1.INIT0 = 16'hF000;
    defparam status_bit_index_2860_add_4_1.INIT1 = 16'h8787;
    defparam status_bit_index_2860_add_4_1.INJECT1_0 = "NO";
    defparam status_bit_index_2860_add_4_1.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1888_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_frame_sequence[21]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1366)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1888_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_4_lut_adj_83 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3398), 
         .C(phase_active[9]), .D(phase_acc[31]), .Z(us_tx_c_9)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_83.init = 16'h0880;
    LUT4 i15136_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9445), .C(n9444), .Z(spi_rx_shift_4__N_929)) /* synthesis lut_function=(!(A+(B (C)))) */ ;
    defparam i15136_2_lut_3_lut.init = 16'h1515;
    LUT4 i15229_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_frame_sequence[21]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1472)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15229_2_lut_4_lut.init = 16'h0035;
    LUT4 i30_3_lut_4_lut_adj_84 (.A(amplitude_phase[6]), .B(\level_active[67] [0]), 
         .C(\level_active[67] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3491)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_84.init = 16'h40f4;
    LUT4 i1_4_lut_adj_85 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[50]), 
         .C(n26_adj_3458), .D(phase_acc[31]), .Z(us_tx_c_50)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_85.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1732_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9445), 
         .C(n9444), .Z(spi_rx_shift_6__N_918)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1732_2_lut_3_lut.init = 16'h4040;
    LUT4 i30_3_lut_4_lut_adj_86 (.A(amplitude_phase[6]), .B(\level_active[66] [0]), 
         .C(\level_active[66] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3490)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_86.init = 16'h40f4;
    LUT4 i15139_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9441), .C(n9440), .Z(spi_rx_shift_3__N_932)) /* synthesis lut_function=(!(A+(B (C)))) */ ;
    defparam i15139_2_lut_3_lut.init = 16'h1515;
    LUT4 i30_3_lut_4_lut_adj_87 (.A(amplitude_phase[6]), .B(\level_active[65] [0]), 
         .C(\level_active[65] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3487)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_87.init = 16'h40f4;
    LUT4 i9899_2_lut (.A(n9581), .B(n9580), .Z(spi_frame_sequence[17])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9899_2_lut.init = 16'h8888;
    LUT4 i30_3_lut_4_lut_adj_88 (.A(amplitude_phase[6]), .B(\level_active[64] [0]), 
         .C(\level_active[64] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3486)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_88.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1733_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9441), 
         .C(n9440), .Z(spi_rx_shift_6__N_919)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1733_2_lut_3_lut.init = 16'h4040;
    LUT4 i2_4_lut_adj_89 (.A(phase_active[51]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3459), .Z(us_tx_c_51)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_89.init = 16'h4800;
    LUT4 i15142_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9437), .C(n9436), .Z(spi_rx_shift_2__N_935)) /* synthesis lut_function=(!(A+(B (C)))) */ ;
    defparam i15142_2_lut_3_lut.init = 16'h1515;
    LUT4 i30_3_lut_4_lut_adj_90 (.A(amplitude_phase[6]), .B(\level_active[63] [0]), 
         .C(\level_active[63] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3483)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_90.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1889_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_frame_sequence[20]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1368)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1889_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1734_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9437), 
         .C(n9436), .Z(spi_rx_shift_6__N_920)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1734_2_lut_3_lut.init = 16'h4040;
    LUT4 i15232_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_frame_sequence[20]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1475)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15232_2_lut_4_lut.init = 16'h0035;
    LUT4 i15145_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9261), .C(n9260), .Z(spi_rx_shift_1__N_938)) /* synthesis lut_function=(!(A+(B (C)))) */ ;
    defparam i15145_2_lut_3_lut.init = 16'h1515;
    LUT4 i30_3_lut_4_lut_adj_91 (.A(amplitude_phase[6]), .B(\level_active[62] [0]), 
         .C(\level_active[62] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3482)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_91.init = 16'h40f4;
    LUT4 i1_4_lut_adj_92 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[52]), 
         .C(n26_adj_3462), .D(phase_acc[31]), .Z(us_tx_c_52)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_92.init = 16'h2080;
    LUT4 i30_3_lut_4_lut_adj_93 (.A(amplitude_phase[6]), .B(\level_active[61] [0]), 
         .C(\level_active[61] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3479)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_93.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1735_2_lut_3_lut (.A(fpga_cs_n_c), .B(n9261), 
         .C(n9260), .Z(spi_rx_shift_6__N_921)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1735_2_lut_3_lut.init = 16'h4040;
    LUT4 i30_3_lut_4_lut_adj_94 (.A(amplitude_phase[6]), .B(\level_active[60] [0]), 
         .C(\level_active[60] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3478)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_94.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_95 (.A(amplitude_phase[6]), .B(\level_active[59] [0]), 
         .C(\level_active[59] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3475)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_95.init = 16'h40f4;
    LUT4 spi_command_7__I_0_2159_i10_2_lut_4_lut (.A(n9465), .B(n9464), 
         .C(n9469), .D(n9468), .Z(n10_adj_3392)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;   // src/umh_fpga_top.v(262[46:66])
    defparam spi_command_7__I_0_2159_i10_2_lut_4_lut.init = 16'hf888;
    LUT4 i30_3_lut_4_lut_adj_96 (.A(amplitude_phase[6]), .B(\level_active[76] [0]), 
         .C(\level_active[76] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3504)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_96.init = 16'h40f4;
    LUT4 i1_2_lut_3_lut_adj_97 (.A(n9473), .B(n9472), .C(n9003), .Z(n14_adj_3399)) /* synthesis lut_function=(((C)+!B)+!A) */ ;   // src/umh_fpga_top.v(262[22:42])
    defparam i1_2_lut_3_lut_adj_97.init = 16'hf7f7;
    LUT4 i1_2_lut_3_lut_adj_98 (.A(n9461), .B(n9460), .C(n10_adj_3392), 
         .Z(n9009)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;   // src/umh_fpga_top.v(268[18:38])
    defparam i1_2_lut_3_lut_adj_98.init = 16'hf8f8;
    LUT4 i2_4_lut_adj_99 (.A(phase_active[53]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3463), .Z(us_tx_c_53)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_99.init = 16'h4800;
    LUT4 i9668_2_lut_4_lut (.A(n9809), .B(n9808), .C(n9805), .D(n9804), 
         .Z(n13787)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;
    defparam i9668_2_lut_4_lut.init = 16'hf888;
    LUT4 fpga_cs_n_N_339_I_0_1955_2_lut_3_lut (.A(spi_expected_length[18]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1564)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1955_2_lut_3_lut.init = 16'h0808;
    LUT4 i1_2_lut_3_lut_adj_100 (.A(n9793), .B(n9792), .C(n18527), .Z(n13987)) /* synthesis lut_function=(A (B+(C))+!A (C)) */ ;
    defparam i1_2_lut_3_lut_adj_100.init = 16'hf8f8;
    CCU2D mic_divider_2866_add_4_7 (.A0(mic_divider[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17188), .S0(n35), .S1(n34));   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866_add_4_7.INIT0 = 16'hfaaa;
    defparam mic_divider_2866_add_4_7.INIT1 = 16'hfaaa;
    defparam mic_divider_2866_add_4_7.INJECT1_0 = "NO";
    defparam mic_divider_2866_add_4_7.INJECT1_1 = "NO";
    LUT4 i9911_2_lut_4_lut (.A(n9793), .B(n9792), .C(n18527), .D(n13957), 
         .Z(n14030)) /* synthesis lut_function=(A (B+(C+(D)))+!A (C+(D))) */ ;
    defparam i9911_2_lut_4_lut.init = 16'hfff8;
    LUT4 i1_4_lut_adj_101 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[54]), 
         .C(n26_adj_3466), .D(phase_acc[31]), .Z(us_tx_c_54)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_101.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1890_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_frame_sequence[19]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1370)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1890_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_2_lut_4_lut_adj_102 (.A(n9761), .B(n9760), .C(spi_byte_count[3]), 
         .D(spi_byte_count[2]), .Z(n18634)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;
    defparam i1_2_lut_4_lut_adj_102.init = 16'h0008;
    LUT4 i15389_2_lut_3_lut (.A(spi_expected_length[18]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1673)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15389_2_lut_3_lut.init = 16'h0707;
    LUT4 i14637_2_lut_4_lut (.A(n9769), .B(n9768), .C(n9765), .D(n9764), 
         .Z(n128)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;
    defparam i14637_2_lut_4_lut.init = 16'hf888;
    LUT4 i15235_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_frame_sequence[19]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1478)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15235_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_2_lut_4_lut_adj_103 (.A(n9817), .B(n9816), .C(n9813), .D(n9812), 
         .Z(n7_adj_3395)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;
    defparam i1_2_lut_4_lut_adj_103.init = 16'hf888;
    CCU2D mic_divider_2866_add_4_5 (.A0(mic_divider[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17187), .COUT(n17188), .S0(n37_adj_3385), 
          .S1(n36_adj_3384));   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866_add_4_5.INIT0 = 16'hfaaa;
    defparam mic_divider_2866_add_4_5.INIT1 = 16'hfaaa;
    defparam mic_divider_2866_add_4_5.INJECT1_0 = "NO";
    defparam mic_divider_2866_add_4_5.INJECT1_1 = "NO";
    LUT4 i2_4_lut_adj_104 (.A(phase_active[55]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3467), .Z(us_tx_c_55)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_104.init = 16'h4800;
    LUT4 i6097_2_lut_3_lut_4_lut (.A(n13), .B(load_active), .C(frame_toggle_sync), 
         .D(frame_toggle_seen), .Z(n10305)) /* synthesis lut_function=(!(A (B+(C (D)+!C !(D)))+!A (C (D)+!C !(D)))) */ ;
    defparam i6097_2_lut_3_lut_4_lut.init = 16'h0770;
    LUT4 i30_3_lut_4_lut_adj_105 (.A(amplitude_phase[6]), .B(\level_active[58] [0]), 
         .C(\level_active[58] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3474)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_105.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1956_2_lut_3_lut (.A(spi_expected_length[17]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1566)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1956_2_lut_3_lut.init = 16'h0808;
    LUT4 i30_3_lut_4_lut_adj_106 (.A(amplitude_phase[6]), .B(\level_active[57] [0]), 
         .C(\level_active[57] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3471)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_106.init = 16'h40f4;
    LUT4 i9898_2_lut (.A(n9585), .B(n9584), .Z(spi_frame_sequence[18])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9898_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_4_lut_adj_107 (.A(n13), .B(load_active), .C(frame_toggle_sync), 
         .D(frame_toggle_seen), .Z(fpga_clk_c_enable_441)) /* synthesis lut_function=(A (B+!(C (D)+!C !(D)))+!A !(C (D)+!C !(D))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_107.init = 16'h8ff8;
    LUT4 i1_4_lut_adj_108 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[56]), 
         .C(n26_adj_3470), .D(phase_acc[31]), .Z(us_tx_c_56)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_108.init = 16'h2080;
    LUT4 i30_3_lut_4_lut_adj_109 (.A(amplitude_phase[6]), .B(\level_active[56] [0]), 
         .C(\level_active[56] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3470)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_109.init = 16'h40f4;
    LUT4 i15392_2_lut_3_lut (.A(spi_expected_length[17]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1676)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15392_2_lut_3_lut.init = 16'h0707;
    LUT4 i30_3_lut_4_lut_adj_110 (.A(amplitude_phase[6]), .B(\level_active[55] [0]), 
         .C(\level_active[55] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3467)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_110.init = 16'h40f4;
    CCU2D mic_divider_2866_add_4_3 (.A0(mic_divider[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17186), .COUT(n17187), .S0(n39), .S1(n38_adj_3386));   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866_add_4_3.INIT0 = 16'hfaaa;
    defparam mic_divider_2866_add_4_3.INIT1 = 16'hfaaa;
    defparam mic_divider_2866_add_4_3.INJECT1_0 = "NO";
    defparam mic_divider_2866_add_4_3.INJECT1_1 = "NO";
    LUT4 i30_3_lut_4_lut_adj_111 (.A(amplitude_phase[6]), .B(\level_active[54] [0]), 
         .C(\level_active[54] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3466)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_111.init = 16'h40f4;
    LUT4 i32_3_lut_4_lut_adj_112 (.A(amplitude_phase[6]), .B(\level_active[39] [0]), 
         .C(\level_active[39] [1]), .D(amplitude_phase[7]), .Z(n29)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i32_3_lut_4_lut_adj_112.init = 16'h40f4;
    LUT4 i2_4_lut_adj_113 (.A(phase_active[57]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3471), .Z(us_tx_c_57)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_113.init = 16'h4800;
    LUT4 fpga_cs_n_N_339_I_0_1891_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_frame_sequence[18]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1372)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1891_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15238_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_frame_sequence[18]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1481)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15238_2_lut_4_lut.init = 16'h0035;
    LUT4 i2_4_lut_adj_114 (.A(n26_adj_3371), .B(phase_active[10]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_10)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_114.init = 16'h2080;
    CCU2D mic_divider_2866_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(mic_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n17186), .S1(n40_adj_3387));   // src/umh_fpga_top.v(333[24:42])
    defparam mic_divider_2866_add_4_1.INIT0 = 16'hF000;
    defparam mic_divider_2866_add_4_1.INIT1 = 16'h0555;
    defparam mic_divider_2866_add_4_1.INJECT1_0 = "NO";
    defparam mic_divider_2866_add_4_1.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1957_2_lut_4_lut (.A(spi_expected_length_31__N_2085[16]), 
         .B(spi_expected_length[16]), .C(n37_adj_3400), .D(fpga_cs_n_c), 
         .Z(spi_expected_length_31__N_1568)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1957_2_lut_4_lut.init = 16'h00ca;
    LUT4 i30_3_lut_4_lut_adj_115 (.A(amplitude_phase[6]), .B(\level_active[53] [0]), 
         .C(\level_active[53] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3463)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_115.init = 16'h40f4;
    LUT4 i1_4_lut_adj_116 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[58]), 
         .C(n26_adj_3474), .D(phase_acc[31]), .Z(us_tx_c_58)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_116.init = 16'h2080;
    LUT4 i30_3_lut_4_lut_adj_117 (.A(amplitude_phase[6]), .B(\level_active[52] [0]), 
         .C(\level_active[52] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3462)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_117.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_118 (.A(amplitude_phase[6]), .B(\level_active[51] [0]), 
         .C(\level_active[51] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3459)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_118.init = 16'h40f4;
    CCU2D amplitude_phase_2864_add_4_9 (.A0(amplitude_phase[7]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17185), .S0(n38_adj_3419));   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864_add_4_9.INIT0 = 16'hfaaa;
    defparam amplitude_phase_2864_add_4_9.INIT1 = 16'h0000;
    defparam amplitude_phase_2864_add_4_9.INJECT1_0 = "NO";
    defparam amplitude_phase_2864_add_4_9.INJECT1_1 = "NO";
    LUT4 i30_3_lut_4_lut_adj_119 (.A(amplitude_phase[6]), .B(\level_active[50] [0]), 
         .C(\level_active[50] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3458)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_119.init = 16'h40f4;
    LUT4 n20867_bdd_3_lut_3_lut (.A(spi_byte_count[8]), .B(spi_byte_count[7]), 
         .C(n20867), .Z(n20868)) /* synthesis lut_function=(!(A (B+!(C))+!A !(B+(C)))) */ ;
    defparam n20867_bdd_3_lut_3_lut.init = 16'h7474;
    LUT4 i30_3_lut_4_lut_adj_120 (.A(amplitude_phase[6]), .B(\level_active[49] [0]), 
         .C(\level_active[49] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3455)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_120.init = 16'h40f4;
    LUT4 i15395_2_lut_4_lut (.A(spi_expected_length_31__N_2085[16]), .B(spi_expected_length[16]), 
         .C(n37_adj_3400), .D(fpga_cs_n_c), .Z(spi_expected_length_31__N_1679)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15395_2_lut_4_lut.init = 16'h0035;
    CCU2D amplitude_phase_2864_add_4_7 (.A0(n3), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(amplitude_phase[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17184), .COUT(n17185), .S0(n40_adj_3421), 
          .S1(n39_adj_3420));   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864_add_4_7.INIT0 = 16'hfaaa;
    defparam amplitude_phase_2864_add_4_7.INIT1 = 16'hfaaa;
    defparam amplitude_phase_2864_add_4_7.INJECT1_0 = "NO";
    defparam amplitude_phase_2864_add_4_7.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1892_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_frame_sequence[17]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1374)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1892_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_4_lut_adj_121 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[76]), 
         .C(n26_adj_3504), .D(phase_acc[31]), .Z(us_tx_c_76)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_121.init = 16'h2080;
    LUT4 i15241_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_frame_sequence[17]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1484)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15241_2_lut_4_lut.init = 16'h0035;
    CCU2D amplitude_phase_2864_add_4_5 (.A0(n5_adj_3409), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(n4_adj_3408), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n17183), .COUT(n17184), .S0(n42_adj_3422), 
          .S1(n41));   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864_add_4_5.INIT0 = 16'hfaaa;
    defparam amplitude_phase_2864_add_4_5.INIT1 = 16'hfaaa;
    defparam amplitude_phase_2864_add_4_5.INJECT1_0 = "NO";
    defparam amplitude_phase_2864_add_4_5.INJECT1_1 = "NO";
    LUT4 i30_3_lut_4_lut_adj_122 (.A(amplitude_phase[6]), .B(\level_active[48] [0]), 
         .C(\level_active[48] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3454)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_122.init = 16'h40f4;
    CCU2D amplitude_phase_2864_add_4_3 (.A0(n7_adj_3411), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(n6_adj_3410), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n17182), .COUT(n17183), .S0(n44), 
          .S1(n43));   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864_add_4_3.INIT0 = 16'hfaaa;
    defparam amplitude_phase_2864_add_4_3.INIT1 = 16'hfaaa;
    defparam amplitude_phase_2864_add_4_3.INJECT1_0 = "NO";
    defparam amplitude_phase_2864_add_4_3.INJECT1_1 = "NO";
    LUT4 i30_3_lut_4_lut_adj_123 (.A(amplitude_phase[6]), .B(\level_active[47] [0]), 
         .C(\level_active[47] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3451)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_123.init = 16'h40f4;
    CCU2D amplitude_phase_2864_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(n8_adj_3412), .B1(phase_acc_next[31]), .C1(n62), 
          .D1(phase_acc[31]), .COUT(n17182), .S1(n45));   // src/umh_fpga_top.v(314[28:50])
    defparam amplitude_phase_2864_add_4_1.INIT0 = 16'hF000;
    defparam amplitude_phase_2864_add_4_1.INIT1 = 16'h599a;
    defparam amplitude_phase_2864_add_4_1.INJECT1_0 = "NO";
    defparam amplitude_phase_2864_add_4_1.INJECT1_1 = "NO";
    LUT4 i30_3_lut_4_lut_adj_124 (.A(amplitude_phase[6]), .B(\level_active[46] [0]), 
         .C(\level_active[46] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3450)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_124.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1899_2_lut_4_lut (.A(spi_frame_sequence[10]), 
         .B(spi_rx_shift[1]), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1388)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1899_2_lut_4_lut.init = 16'h00ca;
    CCU2D load_index_2863_add_4_7 (.A0(load_index[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(load_index[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17180), .S0(n35_adj_3439), .S1(n34_adj_3397));   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863_add_4_7.INIT0 = 16'hfaaa;
    defparam load_index_2863_add_4_7.INIT1 = 16'hfaaa;
    defparam load_index_2863_add_4_7.INJECT1_0 = "NO";
    defparam load_index_2863_add_4_7.INJECT1_1 = "NO";
    CCU2D load_index_2863_add_4_5 (.A0(load_index[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(load_index[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17179), .COUT(n17180), .S0(n37_adj_3394), 
          .S1(n36_adj_3396));   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863_add_4_5.INIT0 = 16'hfaaa;
    defparam load_index_2863_add_4_5.INIT1 = 16'hfaaa;
    defparam load_index_2863_add_4_5.INJECT1_0 = "NO";
    defparam load_index_2863_add_4_5.INJECT1_1 = "NO";
    LUT4 i30_3_lut_4_lut_adj_125 (.A(amplitude_phase[6]), .B(\level_active[45] [0]), 
         .C(\level_active[45] [1]), .D(amplitude_phase[7]), .Z(n25)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_125.init = 16'h40f4;
    LUT4 i2_4_lut_adj_126 (.A(phase_active[59]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3475), .Z(us_tx_c_59)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_126.init = 16'h4800;
    LUT4 i30_3_lut_4_lut_adj_127 (.A(amplitude_phase[6]), .B(\level_active[44] [0]), 
         .C(\level_active[44] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3446)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_127.init = 16'h40f4;
    CCU2D load_index_2863_add_4_3 (.A0(load_index[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(load_index[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17178), .COUT(n17179), .S0(n39_adj_3430), 
          .S1(n38_adj_3393));   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863_add_4_3.INIT0 = 16'hfaaa;
    defparam load_index_2863_add_4_3.INIT1 = 16'hfaaa;
    defparam load_index_2863_add_4_3.INJECT1_0 = "NO";
    defparam load_index_2863_add_4_3.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1893_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_frame_sequence[16]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1376)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1893_2_lut_4_lut.init = 16'h00ca;
    CCU2D load_index_2863_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(load_index[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n17178), .S1(n40));   // src/umh_fpga_top.v(310[27:44])
    defparam load_index_2863_add_4_1.INIT0 = 16'hF000;
    defparam load_index_2863_add_4_1.INIT1 = 16'h0555;
    defparam load_index_2863_add_4_1.INJECT1_0 = "NO";
    defparam load_index_2863_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_4_lut_adj_128 (.A(n9297), .B(n9296), .C(n6814), 
         .D(spi_byte_count[1]), .Z(n8652)) /* synthesis lut_function=(!((((D)+!C)+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_128.init = 16'h0080;
    LUT4 i15244_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_frame_sequence[16]), 
         .C(n8610), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1487)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15244_2_lut_4_lut.init = 16'h0035;
    LUT4 i5951_3_lut (.A(n10162), .B(n10161), .C(spi1_sck_c_enable_183), 
         .Z(rgb_values[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5951_3_lut.init = 16'hcaca;
    LUT4 i30_3_lut_4_lut_adj_129 (.A(amplitude_phase[6]), .B(\level_active[43] [0]), 
         .C(\level_active[43] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3372)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_129.init = 16'h40f4;
    LUT4 i9897_2_lut (.A(n9589), .B(n9588), .Z(spi_frame_sequence[19])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9897_2_lut.init = 16'h8888;
    LUT4 i5955_3_lut (.A(n10166), .B(n10165), .C(spi1_sck_c_enable_182), 
         .Z(rgb_values[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5955_3_lut.init = 16'hcaca;
    LUT4 i15262_2_lut_4_lut (.A(spi_frame_sequence[10]), .B(spi_rx_shift[1]), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1505)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15262_2_lut_4_lut.init = 16'h0035;
    LUT4 i2_4_lut_adj_130 (.A(phase_active[11]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3369), .Z(us_tx_c_11)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_130.init = 16'h4800;
    LUT4 i5959_3_lut (.A(n10170), .B(n10169), .C(spi1_sck_c_enable_181), 
         .Z(rgb_values[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5959_3_lut.init = 16'hcaca;
    CCU2D time_divider_2862_add_4_7 (.A0(time_divider[5]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17177), .S0(n30));   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862_add_4_7.INIT0 = 16'hfaaa;
    defparam time_divider_2862_add_4_7.INIT1 = 16'h0000;
    defparam time_divider_2862_add_4_7.INJECT1_0 = "NO";
    defparam time_divider_2862_add_4_7.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_131 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[12]), 
         .C(n26_adj_3445), .D(phase_acc[31]), .Z(us_tx_c_12)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_131.init = 16'h2080;
    LUT4 i5963_3_lut (.A(n10174), .B(n10173), .C(spi1_sck_c_enable_180), 
         .Z(rgb_values[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5963_3_lut.init = 16'hcaca;
    LUT4 i1_2_lut_3_lut_4_lut_adj_132 (.A(n9297), .B(n9296), .C(n6814), 
         .D(spi_byte_count[1]), .Z(n8565)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_132.init = 16'h8000;
    LUT4 i5967_3_lut (.A(n10178), .B(n10177), .C(spi1_sck_c_enable_179), 
         .Z(rgb_values[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5967_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_133 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[60]), 
         .C(n26_adj_3478), .D(phase_acc[31]), .Z(us_tx_c_60)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_133.init = 16'h2080;
    CCU2D time_divider_2862_add_4_5 (.A0(time_divider[3]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[4]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n17176), .COUT(n17177), .S0(n32_adj_3440), 
          .S1(n31));   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862_add_4_5.INIT0 = 16'hfaaa;
    defparam time_divider_2862_add_4_5.INIT1 = 16'hfaaa;
    defparam time_divider_2862_add_4_5.INJECT1_0 = "NO";
    defparam time_divider_2862_add_4_5.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_134 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3444), 
         .C(phase_active[13]), .D(phase_acc[31]), .Z(us_tx_c_13)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_134.init = 16'h0880;
    LUT4 load_index_6__I_0_i13_2_lut_3_lut_4_lut (.A(load_index[0]), .B(load_index[1]), 
         .C(load_index[2]), .D(n12), .Z(n13)) /* synthesis lut_function=(((C+(D))+!B)+!A) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam load_index_6__I_0_i13_2_lut_3_lut_4_lut.init = 16'hfff7;
    LUT4 i5971_3_lut (.A(n10182), .B(n10181), .C(spi1_sck_c_enable_178), 
         .Z(rgb_values[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5971_3_lut.init = 16'hcaca;
    LUT4 i5975_3_lut (.A(n10186), .B(n10185), .C(spi1_sck_c_enable_177), 
         .Z(rgb_values[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5975_3_lut.init = 16'hcaca;
    CCU2D time_divider_2862_add_4_3 (.A0(time_divider[1]), .B0(GND_net), 
          .C0(GND_net), .D0(GND_net), .A1(time_divider[2]), .B1(GND_net), 
          .C1(GND_net), .D1(GND_net), .CIN(n17175), .COUT(n17176), .S0(n34_adj_3441), 
          .S1(n33));   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862_add_4_3.INIT0 = 16'hfaaa;
    defparam time_divider_2862_add_4_3.INIT1 = 16'hfaaa;
    defparam time_divider_2862_add_4_3.INJECT1_0 = "NO";
    defparam time_divider_2862_add_4_3.INJECT1_1 = "NO";
    CCU2D time_divider_2862_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(time_divider[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n17175), .S1(n35_adj_3442));   // src/umh_fpga_top.v(283[25:44])
    defparam time_divider_2862_add_4_1.INIT0 = 16'hF000;
    defparam time_divider_2862_add_4_1.INIT1 = 16'h0555;
    defparam time_divider_2862_add_4_1.INJECT1_0 = "NO";
    defparam time_divider_2862_add_4_1.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1630_2_lut_4_lut (.A(rgb_values[2]), .B(spi_rx_shift[1]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_525)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1630_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5979_3_lut (.A(n10190), .B(n10189), .C(spi1_sck_c_enable_176), 
         .Z(rgb_values[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5979_3_lut.init = 16'hcaca;
    CCU2D add_2850_31 (.A0(phase_acc[30]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[31]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17173), .S0(phase_acc_next[30]), .S1(phase_acc_next[31]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_31.INIT0 = 16'h5aaa;
    defparam add_2850_31.INIT1 = 16'h5aaa;
    defparam add_2850_31.INJECT1_0 = "NO";
    defparam add_2850_31.INJECT1_1 = "NO";
    LUT4 i15776_2_lut_4_lut (.A(rgb_values[2]), .B(spi_rx_shift[1]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_907)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15776_2_lut_4_lut.init = 16'h0035;
    LUT4 i9748_2_lut (.A(n9401), .B(n9400), .Z(spi_expected_length[25])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9748_2_lut.init = 16'h8888;
    LUT4 i5983_3_lut (.A(n10194), .B(n10193), .C(spi1_sck_c_enable_175), 
         .Z(rgb_values[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5983_3_lut.init = 16'hcaca;
    LUT4 spi_byte_count_7__bdd_2_lut_16255_3_lut (.A(n9785), .B(n9784), 
         .C(n20622), .Z(n20623)) /* synthesis lut_function=(!(A (B+!(C))+!A !(C))) */ ;
    defparam spi_byte_count_7__bdd_2_lut_16255_3_lut.init = 16'h7070;
    LUT4 i2_4_lut_adj_135 (.A(phase_active[61]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3479), .Z(us_tx_c_61)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_135.init = 16'h4800;
    LUT4 i1_4_lut_adj_136 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[14]), 
         .C(n26_adj_3448), .D(phase_acc[31]), .Z(us_tx_c_14)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_136.init = 16'h2080;
    LUT4 i9855_2_lut (.A(n9405), .B(n9404), .Z(spi_expected_length[26])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9855_2_lut.init = 16'h8888;
    CCU2D add_2850_29 (.A0(phase_acc[28]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[29]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17172), .COUT(n17173), .S0(phase_acc_next[28]), 
          .S1(phase_acc_next[29]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_29.INIT0 = 16'h5aaa;
    defparam add_2850_29.INIT1 = 16'h5aaa;
    defparam add_2850_29.INJECT1_0 = "NO";
    defparam add_2850_29.INJECT1_1 = "NO";
    LUT4 i5987_3_lut (.A(n10198), .B(n10197), .C(spi1_sck_c_enable_121), 
         .Z(rgb_values[1])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5987_3_lut.init = 16'hcaca;
    LUT4 i9852_2_lut (.A(n9409), .B(n9408), .Z(spi_expected_length[27])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9852_2_lut.init = 16'h8888;
    LUT4 i9670_2_lut_4_lut (.A(n9801), .B(n9800), .C(n9797), .D(n9796), 
         .Z(n13789)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;
    defparam i9670_2_lut_4_lut.init = 16'hf888;
    LUT4 i9684_2_lut (.A(n9413), .B(n9412), .Z(spi_expected_length[28])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9684_2_lut.init = 16'h8888;
    CCU2D add_2850_27 (.A0(phase_acc[26]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[27]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17171), .COUT(n17172), .S0(phase_acc_next[26]), 
          .S1(phase_acc_next[27]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_27.INIT0 = 16'h5aaa;
    defparam add_2850_27.INIT1 = 16'h5aaa;
    defparam add_2850_27.INJECT1_0 = "NO";
    defparam add_2850_27.INJECT1_1 = "NO";
    FD1S3BX spi_bit_count_i0_5045_5046_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_bit_count_2__N_948), .Q(n9257)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_bit_count_i0_5045_5046_set.GSR = "ENABLED";
    LUT4 fpga_cs_n_N_339_I_0_1629_2_lut_4_lut (.A(rgb_values[3]), .B(spi_rx_shift[2]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_523)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1629_2_lut_4_lut.init = 16'h00ca;
    LUT4 i2_3_lut_4_lut_adj_137 (.A(n9805), .B(n9804), .C(n13789), .D(spi_byte_count[13]), 
         .Z(n18524)) /* synthesis lut_function=(A (B+(C+(D)))+!A (C+(D))) */ ;
    defparam i2_3_lut_4_lut_adj_137.init = 16'hfff8;
    FD1P3DX rgb_values_i0_5041_5042_reset (.D(n9253), .SP(spi1_sck_c_enable_191), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_913), .Q(n9254)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i0_5041_5042_reset.GSR = "ENABLED";
    FD1S3BX spi_frame_sequence_i28_5413_5414_set (.D(n21164), .CK(spi1_sck_c), 
            .PD(spi_frame_sequence_31__N_1352), .Q(n9625)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam spi_frame_sequence_i28_5413_5414_set.GSR = "ENABLED";
    LUT4 i5615_3_lut (.A(n9826), .B(n9825), .C(spi1_sck_c_enable_76), 
         .Z(rgb_values[94])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5615_3_lut.init = 16'hcaca;
    CCU2D add_2850_25 (.A0(phase_acc[24]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[25]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17170), .COUT(n17171), .S0(phase_acc_next[24]), 
          .S1(phase_acc_next[25]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_25.INIT0 = 16'h5aaa;
    defparam add_2850_25.INIT1 = 16'h5aaa;
    defparam add_2850_25.INJECT1_0 = "NO";
    defparam add_2850_25.INJECT1_1 = "NO";
    CCU2D add_2850_23 (.A0(phase_acc[22]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[23]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17169), .COUT(n17170), .S0(phase_acc_next[22]), 
          .S1(phase_acc_next[23]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_23.INIT0 = 16'h5aaa;
    defparam add_2850_23.INIT1 = 16'h5aaa;
    defparam add_2850_23.INJECT1_0 = "NO";
    defparam add_2850_23.INJECT1_1 = "NO";
    LUT4 i5647_3_lut (.A(n9858), .B(n9857), .C(spi1_sck_c_enable_68), 
         .Z(rgb_values[86])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5647_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1624_2_lut_4_lut (.A(rgb_values[8]), .B(rgb_values[0]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_513)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1624_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1964_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[9]), 
         .Z(spi_expected_length_31__N_1582)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1964_2_lut.init = 16'h4444;
    LUT4 i15416_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[9]), 
         .Z(spi_expected_length_31__N_1700)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15416_2_lut.init = 16'h1111;
    LUT4 i15758_2_lut_4_lut (.A(rgb_values[8]), .B(rgb_values[0]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_889)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15758_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_4_lut_adj_138 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[62]), 
         .C(n26_adj_3482), .D(phase_acc[31]), .Z(us_tx_c_62)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_138.init = 16'h2080;
    LUT4 i1_2_lut_3_lut_adj_139 (.A(n9277), .B(n9276), .C(n7_adj_3395), 
         .Z(n18643)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;
    defparam i1_2_lut_3_lut_adj_139.init = 16'h0808;
    LUT4 i15548_2_lut_4_lut (.A(rgb_values[78]), .B(rgb_values[70]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_679)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15548_2_lut_4_lut.init = 16'h0035;
    FD1P3DX rgb_values_i18_5917_5918_reset (.D(n10129), .SP(spi1_sck_c_enable_192), 
            .CK(spi1_sck_c), .CD(rgb_values_95__N_859), .Q(n10130)) /* synthesis lse_init_val=0 */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam rgb_values_i18_5917_5918_reset.GSR = "ENABLED";
    LUT4 i15121_2_lut_3_lut_4_lut (.A(n9433), .B(n9432), .C(n7166), .D(fpga_cs_n_c), 
         .Z(spi_bit_count_2__N_950)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B (C+(D)))+!A (C+(D)))) */ ;   // src/umh_fpga_top.v(237[30:50])
    defparam i15121_2_lut_3_lut_4_lut.init = 16'h0087;
    LUT4 i5619_3_lut (.A(n9830), .B(n9829), .C(spi1_sck_c_enable_75), 
         .Z(rgb_values[93])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5619_3_lut.init = 16'hcaca;
    FD1S1D i5040 (.D(n21165), .CK(rgb_values_95__N_529), .CD(rgb_values_95__N_913), 
           .Q(spi1_sck_c_enable_191));   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5040.GSR = "ENABLED";
    LUT4 i15773_2_lut_4_lut (.A(rgb_values[3]), .B(spi_rx_shift[2]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_904)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15773_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1744_2_lut_3_lut_4_lut (.A(n9433), .B(n9432), 
         .C(n7166), .D(fpga_cs_n_c), .Z(spi_bit_count_2__N_944)) /* synthesis lut_function=(!(A (B (C+(D))+!B ((D)+!C))+!A ((D)+!C))) */ ;   // src/umh_fpga_top.v(237[30:50])
    defparam fpga_cs_n_N_339_I_0_1744_2_lut_3_lut_4_lut.init = 16'h0078;
    LUT4 i2_4_lut_adj_140 (.A(phase_active[63]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3483), .Z(us_tx_c_63)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_140.init = 16'h4800;
    LUT4 i9866_2_lut_3_lut (.A(n9433), .B(n9432), .C(n7166), .Z(rgb_values_95__N_630)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i9866_2_lut_3_lut.init = 16'h8080;
    LUT4 i9853_2_lut (.A(n9417), .B(n9416), .Z(spi_expected_length[29])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9853_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1554_2_lut_4_lut (.A(rgb_values[78]), .B(rgb_values[70]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_373)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1554_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_2_lut_3_lut_adj_141 (.A(n8872), .B(n9293), .C(n9292), .Z(n8873)) /* synthesis lut_function=(A (B (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i1_2_lut_3_lut_adj_141.init = 16'h8080;
    LUT4 fpga_cs_n_N_339_I_0_1972_2_lut_4_lut (.A(spi_expected_length_31__N_2085[1]), 
         .B(spi_expected_length[1]), .C(n37_adj_3400), .D(fpga_cs_n_c), 
         .Z(spi_expected_length_31__N_1598)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1972_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_2_lut_4_lut_adj_142 (.A(n9713), .B(n9712), .C(n9721), .D(n9720), 
         .Z(n5)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;   // src/umh_fpga_top.v(216[37:63])
    defparam i1_2_lut_4_lut_adj_142.init = 16'hf888;
    LUT4 i1_4_lut_adj_143 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[64]), 
         .C(n26_adj_3486), .D(phase_acc[31]), .Z(us_tx_c_64)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_143.init = 16'h2080;
    LUT4 i14662_2_lut_4_lut (.A(n9705), .B(n9704), .C(n9725), .D(n9724), 
         .Z(n18782)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14662_2_lut_4_lut.init = 16'h8000;
    LUT4 i9896_2_lut (.A(n9593), .B(n9592), .Z(spi_frame_sequence[20])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9896_2_lut.init = 16'h8888;
    LUT4 i15440_2_lut_4_lut (.A(spi_expected_length_31__N_2085[1]), .B(spi_expected_length[1]), 
         .C(n37_adj_3400), .D(fpga_cs_n_c), .Z(spi_expected_length_31__N_1724)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15440_2_lut_4_lut.init = 16'h0035;
    LUT4 i5651_3_lut (.A(n9862), .B(n9861), .C(spi1_sck_c_enable_67), 
         .Z(rgb_values[85])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5651_3_lut.init = 16'hcaca;
    CCU2D add_2850_21 (.A0(phase_acc[20]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[21]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17168), .COUT(n17169), .S0(phase_acc_next[20]), 
          .S1(phase_acc_next[21]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_21.INIT0 = 16'h5555;
    defparam add_2850_21.INIT1 = 16'h5555;
    defparam add_2850_21.INJECT1_0 = "NO";
    defparam add_2850_21.INJECT1_1 = "NO";
    LUT4 spi_byte_count_4__bdd_2_lut_16186_4_lut (.A(n9781), .B(n9780), 
         .C(n9789), .D(n9788), .Z(n20621)) /* synthesis lut_function=(A (B+(C (D)))+!A (C (D))) */ ;
    defparam spi_byte_count_4__bdd_2_lut_16186_4_lut.init = 16'hf888;
    LUT4 i2_4_lut_adj_144 (.A(phase_active[65]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3487), .Z(us_tx_c_65)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_144.init = 16'h4800;
    LUT4 i1_2_lut_3_lut_4_lut_adj_145 (.A(n9297), .B(n9296), .C(n6814), 
         .D(spi_byte_count[1]), .Z(n8610)) /* synthesis lut_function=(A (B+((D)+!C))+!A ((D)+!C)) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_145.init = 16'hff8f;
    LUT4 fpga_cs_n_N_339_I_0_1628_2_lut_4_lut (.A(rgb_values[4]), .B(spi_rx_shift[3]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_521)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1628_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15770_2_lut_4_lut (.A(rgb_values[4]), .B(spi_rx_shift[3]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_901)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15770_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1904_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_frame_sequence[5]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1398)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1904_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15277_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_frame_sequence[5]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1520)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15277_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1623_2_lut_4_lut (.A(rgb_values[9]), .B(rgb_values[1]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_511)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1623_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9677_2_lut (.A(n9421), .B(n9420), .Z(spi_expected_length[30])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9677_2_lut.init = 16'h8888;
    LUT4 i1_2_lut_3_lut_4_lut_adj_146 (.A(n9297), .B(n9296), .C(n6814), 
         .D(spi_byte_count[1]), .Z(n8632)) /* synthesis lut_function=(A (B+!(C (D)))+!A !(C (D))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_146.init = 16'h8fff;
    LUT4 i1_4_lut_adj_147 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[66]), 
         .C(n26_adj_3490), .D(phase_acc[31]), .Z(us_tx_c_66)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_147.init = 16'h2080;
    LUT4 i3395_2_lut_4_lut (.A(n9277), .B(n9276), .C(n9641), .D(n9640), 
         .Z(n7607)) /* synthesis lut_function=(!(((C (D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 202[63])
    defparam i3395_2_lut_4_lut.init = 16'h0888;
    LUT4 fpga_cs_n_N_339_I_0_1898_2_lut_4_lut (.A(spi_frame_sequence[11]), 
         .B(spi_rx_shift[2]), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1386)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1898_2_lut_4_lut.init = 16'h00ca;
    LUT4 i13097_2_lut_4_lut (.A(n9261), .B(n9260), .C(n7), .D(n8_adj_3389), 
         .Z(n17074)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13097_2_lut_4_lut.init = 16'h0008;
    LUT4 i2_4_lut_adj_148 (.A(phase_active[67]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3491), .Z(us_tx_c_67)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_148.init = 16'h4800;
    LUT4 i13099_2_lut_4_lut (.A(n9441), .B(n9440), .C(n7), .D(n8_adj_3389), 
         .Z(n17070)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13099_2_lut_4_lut.init = 16'h0008;
    LUT4 i13098_2_lut_4_lut (.A(n9437), .B(n9436), .C(n7), .D(n8_adj_3389), 
         .Z(n17072)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13098_2_lut_4_lut.init = 16'h0008;
    LUT4 i15259_2_lut_4_lut (.A(spi_frame_sequence[11]), .B(spi_rx_shift[2]), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1502)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15259_2_lut_4_lut.init = 16'h0035;
    LUT4 i15755_2_lut_4_lut (.A(rgb_values[9]), .B(rgb_values[1]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_886)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15755_2_lut_4_lut.init = 16'h0035;
    LUT4 i15551_2_lut_4_lut (.A(rgb_values[77]), .B(rgb_values[69]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_682)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15551_2_lut_4_lut.init = 16'h0035;
    LUT4 i5623_3_lut (.A(n9834), .B(n9833), .C(spi1_sck_c_enable_74), 
         .Z(rgb_values[92])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5623_3_lut.init = 16'hcaca;
    LUT4 i13102_2_lut_4_lut (.A(n9449), .B(n9448), .C(n7), .D(n8_adj_3389), 
         .Z(n17066)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13102_2_lut_4_lut.init = 16'h0008;
    LUT4 i2_4_lut_adj_149 (.A(n26_adj_3495), .B(phase_active[68]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_68)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_149.init = 16'h2080;
    CCU2D add_2850_19 (.A0(phase_acc[18]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[19]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17167), .COUT(n17168), .S0(phase_acc_next[18]), 
          .S1(phase_acc_next[19]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_19.INIT0 = 16'h5555;
    defparam add_2850_19.INIT1 = 16'h5555;
    defparam add_2850_19.INJECT1_0 = "NO";
    defparam add_2850_19.INJECT1_1 = "NO";
    LUT4 i13101_2_lut_4_lut (.A(n9445), .B(n9444), .C(n7), .D(n8_adj_3389), 
         .Z(n17068)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13101_2_lut_4_lut.init = 16'h0008;
    LUT4 i13096_2_lut_4_lut (.A(n9457), .B(n9456), .C(n7), .D(n8_adj_3389), 
         .Z(n17062)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13096_2_lut_4_lut.init = 16'h0008;
    LUT4 fpga_cs_n_N_339_I_0_1555_2_lut_4_lut (.A(rgb_values[77]), .B(rgb_values[69]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_375)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1555_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1900_2_lut_4_lut (.A(spi_frame_sequence[9]), 
         .B(spi_rx_shift[0]), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1390)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1900_2_lut_4_lut.init = 16'h00ca;
    LUT4 i13103_2_lut_4_lut (.A(n9453), .B(n9452), .C(n7), .D(n8_adj_3389), 
         .Z(n17064)) /* synthesis lut_function=(!(((C+(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(200[44] 203[90])
    defparam i13103_2_lut_4_lut.init = 16'h0008;
    LUT4 i1_4_lut_adj_150 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3494), 
         .C(phase_active[69]), .D(phase_acc[31]), .Z(us_tx_c_69)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_150.init = 16'h0880;
    LUT4 i1_2_lut_3_lut_4_lut_adj_151 (.A(time_divider_5__N_2270[1]), .B(n8892), 
         .C(time_divider[4]), .D(time_divider[2]), .Z(time_half_N_2907)) /* synthesis lut_function=(A (B+(C+(D)))+!A !(B+(C+(D)))) */ ;
    defparam i1_2_lut_3_lut_4_lut_adj_151.init = 16'haaa9;
    LUT4 i9895_2_lut (.A(n9597), .B(n9596), .Z(spi_frame_sequence[21])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9895_2_lut.init = 16'h8888;
    LUT4 i15265_2_lut_4_lut (.A(spi_frame_sequence[9]), .B(spi_rx_shift[0]), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1508)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15265_2_lut_4_lut.init = 16'h0035;
    LUT4 i5655_3_lut (.A(n9866), .B(n9865), .C(spi1_sck_c_enable_66), 
         .Z(rgb_values[84])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5655_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_152 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[70]), 
         .C(n26_adj_3498), .D(phase_acc[31]), .Z(us_tx_c_70)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_152.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1973_2_lut_4_lut (.A(spi_expected_length_31__N_2085[0]), 
         .B(spi_expected_length[0]), .C(n37_adj_3400), .D(fpga_cs_n_c), 
         .Z(spi_expected_length_31__N_1600)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1973_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15443_2_lut_4_lut (.A(spi_expected_length_31__N_2085[0]), .B(spi_expected_length[0]), 
         .C(n37_adj_3400), .D(fpga_cs_n_c), .Z(spi_expected_length_31__N_1727)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15443_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1905_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_frame_sequence[4]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1400)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1905_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_4_lut_adj_153 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3379), 
         .C(phase_active[15]), .D(phase_acc[31]), .Z(us_tx_c_15)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_153.init = 16'h0880;
    LUT4 i1_4_lut_adj_154 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3492), 
         .C(phase_active[71]), .D(phase_acc[31]), .Z(us_tx_c_71)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_154.init = 16'h0880;
    LUT4 fpga_cs_n_N_339_I_0_1901_2_lut_4_lut (.A(spi_frame_sequence[8]), 
         .B(spi1_mosi_c), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1392)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1901_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15280_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_frame_sequence[4]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1523)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15280_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_4_lut_adj_155 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[72]), 
         .C(n26_adj_3500), .D(phase_acc[31]), .Z(us_tx_c_72)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_155.init = 16'h2080;
    LUT4 i15554_2_lut_4_lut (.A(rgb_values[76]), .B(rgb_values[68]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_685)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15554_2_lut_4_lut.init = 16'h0035;
    LUT4 i15268_2_lut_4_lut (.A(spi_frame_sequence[8]), .B(spi1_mosi_c), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1511)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15268_2_lut_4_lut.init = 16'h0035;
    LUT4 i2_4_lut_adj_156 (.A(phase_active[73]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3447), .Z(us_tx_c_73)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_156.init = 16'h4800;
    CCU2D add_2850_17 (.A0(phase_acc[16]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[17]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17166), .COUT(n17167), .S0(phase_acc_next[16]), 
          .S1(phase_acc_next[17]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_17.INIT0 = 16'h5555;
    defparam add_2850_17.INIT1 = 16'h5aaa;
    defparam add_2850_17.INJECT1_0 = "NO";
    defparam add_2850_17.INJECT1_1 = "NO";
    LUT4 i1_4_lut_adj_157 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[74]), 
         .C(n26_adj_3502), .D(phase_acc[31]), .Z(us_tx_c_74)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_157.init = 16'h2080;
    LUT4 i9894_2_lut (.A(n9601), .B(n9600), .Z(spi_frame_sequence[22])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9894_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1556_2_lut_4_lut (.A(rgb_values[76]), .B(rgb_values[68]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_377)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1556_2_lut_4_lut.init = 16'h00ca;
    LUT4 i3004_3_lut_4_lut (.A(n58_adj_3416), .B(n18637), .C(spi_channel_field[0]), 
         .D(spi_channel_field[1]), .Z(spi_channel_field_1__N_1773[1])) /* synthesis lut_function=(A (D)+!A !(B (C (D)+!C !(D))+!B !(D))) */ ;
    defparam i3004_3_lut_4_lut.init = 16'hbf40;
    LUT4 i2_4_lut_adj_158 (.A(phase_active[75]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3497), .Z(us_tx_c_75)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_158.init = 16'h4800;
    LUT4 mux_2587_i1_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi1_mosi_c), 
         .D(spi_level_pending[0]), .Z(spi_level_pending_7__N_1785[0])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i1_3_lut_4_lut.init = 16'hfd20;
    LUT4 i9844_2_lut (.A(n9337), .B(n9336), .Z(spi_expected_length[9])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9844_2_lut.init = 16'h8888;
    LUT4 i9764_2_lut (.A(n9333), .B(n9332), .Z(spi_expected_length[8])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9764_2_lut.init = 16'h8888;
    LUT4 i9771_2_lut (.A(n9329), .B(n9328), .Z(spi_expected_length[7])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9771_2_lut.init = 16'h8888;
    LUT4 i9856_2_lut (.A(n9325), .B(n9324), .Z(spi_expected_length[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9856_2_lut.init = 16'h8888;
    LUT4 i15557_2_lut_4_lut (.A(rgb_values[75]), .B(rgb_values[67]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_688)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15557_2_lut_4_lut.init = 16'h0035;
    LUT4 i9782_2_lut (.A(n9353), .B(n9352), .Z(spi_expected_length[13])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9782_2_lut.init = 16'h8888;
    LUT4 i9784_2_lut (.A(n9349), .B(n9348), .Z(spi_expected_length[12])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9784_2_lut.init = 16'h8888;
    LUT4 i9871_2_lut (.A(n9345), .B(n9344), .Z(spi_expected_length[11])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9871_2_lut.init = 16'h8888;
    LUT4 i9773_2_lut (.A(n9341), .B(n9340), .Z(spi_expected_length[10])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9773_2_lut.init = 16'h8888;
    LUT4 mux_2587_i2_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi_rx_shift[0]), 
         .D(spi_level_pending[1]), .Z(spi_level_pending_7__N_1785[1])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i2_3_lut_4_lut.init = 16'hfd20;
    LUT4 i9781_2_lut (.A(n9357), .B(n9356), .Z(spi_expected_length[14])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9781_2_lut.init = 16'h8888;
    LUT4 mux_2587_i3_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi_rx_shift[1]), 
         .D(spi_level_pending[2]), .Z(spi_level_pending_7__N_1785[2])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i3_3_lut_4_lut.init = 16'hfd20;
    LUT4 fpga_cs_n_N_339_I_0_1557_2_lut_4_lut (.A(rgb_values[75]), .B(rgb_values[67]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_379)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1557_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9780_2_lut (.A(n9361), .B(n9360), .Z(spi_expected_length[15])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9780_2_lut.init = 16'h8888;
    LUT4 mux_2587_i4_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi_rx_shift[2]), 
         .D(spi_level_pending[3]), .Z(spi_level_pending_7__N_1785[3])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i4_3_lut_4_lut.init = 16'hfd20;
    LUT4 i9893_2_lut (.A(n9605), .B(n9604), .Z(spi_frame_sequence[23])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9893_2_lut.init = 16'h8888;
    LUT4 mux_2587_i5_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi_rx_shift[3]), 
         .D(spi_level_pending[4]), .Z(spi_level_pending_7__N_1785[4])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i5_3_lut_4_lut.init = 16'hfd20;
    LUT4 mux_2587_i6_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi_rx_shift[4]), 
         .D(spi_level_pending[5]), .Z(spi_level_pending_7__N_1785[5])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i6_3_lut_4_lut.init = 16'hfd20;
    LUT4 i30_3_lut_4_lut_adj_159 (.A(amplitude_phase[6]), .B(\level_active[36] [0]), 
         .C(\level_active[36] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3493)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_159.init = 16'h40f4;
    CCU2D add_2850_15 (.A0(phase_acc[14]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[15]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17165), .COUT(n17166), .S0(phase_acc_next[14]), 
          .S1(phase_acc_next[15]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_15.INIT0 = 16'h5aaa;
    defparam add_2850_15.INIT1 = 16'h5555;
    defparam add_2850_15.INJECT1_0 = "NO";
    defparam add_2850_15.INJECT1_1 = "NO";
    LUT4 mux_2587_i7_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi_rx_shift[5]), 
         .D(spi_level_pending[6]), .Z(spi_level_pending_7__N_1785[6])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i7_3_lut_4_lut.init = 16'hfd20;
    LUT4 mux_2587_i8_3_lut_4_lut (.A(n8872), .B(spi_channel_field[0]), .C(spi_rx_shift[6]), 
         .D(spi_level_pending[7]), .Z(spi_level_pending_7__N_1785[7])) /* synthesis lut_function=(A (B (D)+!B (C))+!A (D)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam mux_2587_i8_3_lut_4_lut.init = 16'hfd20;
    LUT4 i9874_2_lut (.A(n9425), .B(n9424), .Z(spi_expected_length[31])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9874_2_lut.init = 16'h8888;
    LUT4 i2380_2_lut (.A(frame_toggle_spi_N_2870), .B(accepted_sequence_spi_31__N_2262), 
         .Z(fpga_cs_n_c_enable_34)) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(268[14] 270[8])
    defparam i2380_2_lut.init = 16'h8888;
    LUT4 sub_2258_inv_0_i1_1_lut (.A(status_bit_index[0]), .Z(spi1_miso_N_2860[0])) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(171[51:73])
    defparam sub_2258_inv_0_i1_1_lut.init = 16'h5555;
    LUT4 i3034_1_lut (.A(status_bit_index[1]), .Z(spi1_miso_N_2860[1])) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(176[14:40])
    defparam i3034_1_lut.init = 16'h5555;
    LUT4 i3028_1_lut (.A(status_bit_index[2]), .Z(spi1_miso_N_2860[2])) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(176[14:40])
    defparam i3028_1_lut.init = 16'h5555;
    LUT4 i3032_1_lut (.A(status_bit_index[4]), .Z(spi1_miso_N_2860[4])) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(176[14:40])
    defparam i3032_1_lut.init = 16'h5555;
    LUT4 i9882_2_lut (.A(n9609), .B(n9608), .Z(spi_frame_sequence[24])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9882_2_lut.init = 16'h8888;
    LUT4 i30_3_lut_4_lut_adj_160 (.A(amplitude_phase[6]), .B(\level_active[38] [0]), 
         .C(\level_active[38] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3496)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_160.init = 16'h40f4;
    LUT4 i9880_2_lut (.A(n9613), .B(n9612), .Z(spi_frame_sequence[25])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9880_2_lut.init = 16'h8888;
    LUT4 i15539_2_lut_4_lut (.A(rgb_values[81]), .B(rgb_values[73]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_670)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15539_2_lut_4_lut.init = 16'h0035;
    LUT4 i9854_2_lut (.A(n9617), .B(n9616), .Z(spi_frame_sequence[26])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9854_2_lut.init = 16'h8888;
    LUT4 i15560_2_lut_4_lut (.A(rgb_values[74]), .B(rgb_values[66]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_691)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15560_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1551_2_lut_4_lut (.A(rgb_values[81]), .B(rgb_values[73]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_367)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1551_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1558_2_lut_4_lut (.A(rgb_values[74]), .B(rgb_values[66]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_381)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1558_2_lut_4_lut.init = 16'h00ca;
    CCU2D add_2850_13 (.A0(phase_acc[12]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[13]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17164), .COUT(n17165), .S0(phase_acc_next[12]), 
          .S1(phase_acc_next[13]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_13.INIT0 = 16'h5aaa;
    defparam add_2850_13.INIT1 = 16'h5555;
    defparam add_2850_13.INJECT1_0 = "NO";
    defparam add_2850_13.INJECT1_1 = "NO";
    LUT4 fpga_cs_n_N_339_I_0_1745_2_lut (.A(fpga_cs_n_c), .B(spi_bit_count_2__N_953[1]), 
         .Z(spi_bit_count_2__N_946)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1745_2_lut.init = 16'h4444;
    LUT4 i15124_2_lut (.A(fpga_cs_n_c), .B(spi_bit_count_2__N_953[1]), .Z(spi_bit_count_2__N_956)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15124_2_lut.init = 16'h1111;
    LUT4 i15563_2_lut_4_lut (.A(rgb_values[73]), .B(rgb_values[65]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_694)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15563_2_lut_4_lut.init = 16'h0035;
    CCU2D add_2850_11 (.A0(phase_acc[10]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(phase_acc[11]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17163), .COUT(n17164), .S0(phase_acc_next[10]), 
          .S1(phase_acc_next[11]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_11.INIT0 = 16'h5555;
    defparam add_2850_11.INIT1 = 16'h5555;
    defparam add_2850_11.INJECT1_0 = "NO";
    defparam add_2850_11.INJECT1_1 = "NO";
    LUT4 i9817_2_lut (.A(n9625), .B(n9624), .Z(spi_frame_sequence[28])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9817_2_lut.init = 16'h8888;
    LUT4 i5627_3_lut (.A(n9838), .B(n9837), .C(spi1_sck_c_enable_73), 
         .Z(rgb_values[91])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5627_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1559_2_lut_4_lut (.A(rgb_values[73]), .B(rgb_values[65]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_383)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1559_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15542_2_lut_4_lut (.A(rgb_values[80]), .B(rgb_values[72]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_673)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15542_2_lut_4_lut.init = 16'h0035;
    LUT4 i15566_2_lut_4_lut (.A(rgb_values[72]), .B(rgb_values[64]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_697)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15566_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1560_2_lut_4_lut (.A(rgb_values[72]), .B(rgb_values[64]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_385)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1560_2_lut_4_lut.init = 16'h00ca;
    LUT4 i13073_2_lut (.A(mic_sample_count[2]), .B(n17100), .Z(n28_adj_3405)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // src/umh_fpga_top.v(328[33:56])
    defparam i13073_2_lut.init = 16'h6666;
    LUT4 i9814_2_lut (.A(n9629), .B(n9628), .Z(spi_frame_sequence[29])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9814_2_lut.init = 16'h8888;
    LUT4 i6122_1_lut (.A(spi1_sck_c_enable_192), .Z(spi1_sck_c_enable_78)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6122_1_lut.init = 16'h5555;
    CCU2D add_2850_9 (.A0(phase_acc[8]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_acc[9]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17162), .COUT(n17163), .S0(phase_acc_next[8]), .S1(phase_acc_next[9]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_9.INIT0 = 16'h5aaa;
    defparam add_2850_9.INIT1 = 16'h5555;
    defparam add_2850_9.INJECT1_0 = "NO";
    defparam add_2850_9.INJECT1_1 = "NO";
    LUT4 i6121_1_lut (.A(spi1_sck_c_enable_191), .Z(spi1_sck_c_enable_79)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6121_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1552_2_lut_4_lut (.A(rgb_values[80]), .B(rgb_values[72]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_369)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1552_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6120_1_lut (.A(spi1_sck_c_enable_190), .Z(spi1_sck_c_enable_80)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6120_1_lut.init = 16'h5555;
    LUT4 i15569_2_lut_4_lut (.A(rgb_values[71]), .B(rgb_values[63]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_700)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15569_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1561_2_lut_4_lut (.A(rgb_values[71]), .B(rgb_values[63]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_387)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1561_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6119_1_lut (.A(spi1_sck_c_enable_189), .Z(spi1_sck_c_enable_81)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6119_1_lut.init = 16'h5555;
    LUT4 i6118_1_lut (.A(spi1_sck_c_enable_188), .Z(spi1_sck_c_enable_82)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6118_1_lut.init = 16'h5555;
    LUT4 i6117_1_lut (.A(spi1_sck_c_enable_187), .Z(spi1_sck_c_enable_83)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6117_1_lut.init = 16'h5555;
    LUT4 i6116_1_lut (.A(spi1_sck_c_enable_186), .Z(spi1_sck_c_enable_84)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6116_1_lut.init = 16'h5555;
    LUT4 i5631_3_lut (.A(n9842), .B(n9841), .C(spi1_sck_c_enable_72), 
         .Z(rgb_values[90])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5631_3_lut.init = 16'hcaca;
    LUT4 i6115_1_lut (.A(spi1_sck_c_enable_185), .Z(spi1_sck_c_enable_85)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6115_1_lut.init = 16'h5555;
    LUT4 i6114_1_lut (.A(spi1_sck_c_enable_184), .Z(spi1_sck_c_enable_86)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6114_1_lut.init = 16'h5555;
    LUT4 i6113_1_lut (.A(spi1_sck_c_enable_183), .Z(spi1_sck_c_enable_87)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6113_1_lut.init = 16'h5555;
    LUT4 i6112_1_lut (.A(spi1_sck_c_enable_182), .Z(spi1_sck_c_enable_88)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6112_1_lut.init = 16'h5555;
    LUT4 i15572_2_lut_4_lut (.A(rgb_values[70]), .B(rgb_values[62]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_703)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15572_2_lut_4_lut.init = 16'h0035;
    LUT4 i6111_1_lut (.A(spi1_sck_c_enable_181), .Z(spi1_sck_c_enable_89)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6111_1_lut.init = 16'h5555;
    LUT4 i6110_1_lut (.A(spi1_sck_c_enable_180), .Z(spi1_sck_c_enable_90)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6110_1_lut.init = 16'h5555;
    LUT4 i6109_1_lut (.A(spi1_sck_c_enable_179), .Z(spi1_sck_c_enable_91)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6109_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1562_2_lut_4_lut (.A(rgb_values[70]), .B(rgb_values[62]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_389)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1562_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6108_1_lut (.A(spi1_sck_c_enable_178), .Z(spi1_sck_c_enable_92)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6108_1_lut.init = 16'h5555;
    LUT4 i6107_1_lut (.A(spi1_sck_c_enable_177), .Z(spi1_sck_c_enable_93)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6107_1_lut.init = 16'h5555;
    LUT4 i9813_2_lut (.A(n9633), .B(n9632), .Z(spi_frame_sequence[30])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9813_2_lut.init = 16'h8888;
    LUT4 i15575_2_lut_4_lut (.A(rgb_values[69]), .B(rgb_values[61]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_706)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15575_2_lut_4_lut.init = 16'h0035;
    CCU2D add_2850_7 (.A0(phase_acc[6]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_acc[7]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17161), .COUT(n17162), .S0(phase_acc_next[6]), .S1(phase_acc_next[7]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_7.INIT0 = 16'h5555;
    defparam add_2850_7.INIT1 = 16'h5aaa;
    defparam add_2850_7.INJECT1_0 = "NO";
    defparam add_2850_7.INJECT1_1 = "NO";
    LUT4 i6106_1_lut (.A(spi1_sck_c_enable_176), .Z(spi1_sck_c_enable_94)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6106_1_lut.init = 16'h5555;
    LUT4 i6105_1_lut (.A(spi1_sck_c_enable_175), .Z(spi1_sck_c_enable_95)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6105_1_lut.init = 16'h5555;
    LUT4 i6104_1_lut (.A(spi1_sck_c_enable_121), .Z(spi1_sck_c_enable_96)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6104_1_lut.init = 16'h5555;
    LUT4 i5635_3_lut (.A(n9846), .B(n9845), .C(spi1_sck_c_enable_71), 
         .Z(rgb_values[89])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5635_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1563_2_lut_4_lut (.A(rgb_values[69]), .B(rgb_values[61]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_391)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1563_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15545_2_lut_4_lut (.A(rgb_values[79]), .B(rgb_values[71]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_676)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15545_2_lut_4_lut.init = 16'h0035;
    spi_mic_stream mic_stream_i (.GND_net(GND_net), .sck_N_3318(sck_N_3318), 
            .spi_mic_cs_n_c(spi_mic_cs_n_c), .mic_latest({mic_latest}), 
            .spi_mic_miso_c(spi_mic_miso_c)) /* synthesis syn_module_defined=1 */ ;   // src/umh_fpga_top.v(348[16] 351[2])
    LUT4 fpga_cs_n_N_339_I_0_1553_2_lut_4_lut (.A(rgb_values[79]), .B(rgb_values[71]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_371)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1553_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1906_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_frame_sequence[3]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1402)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1906_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9812_2_lut (.A(n9637), .B(n9636), .Z(spi_frame_sequence[31])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9812_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1627_2_lut_4_lut (.A(rgb_values[5]), .B(spi_rx_shift[4]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_519)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1627_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15767_2_lut_4_lut (.A(rgb_values[5]), .B(spi_rx_shift[4]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_898)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15767_2_lut_4_lut.init = 16'h0035;
    LUT4 i3417_2_lut_4_lut (.A(n8881), .B(spi_byte_count[4]), .C(spi_byte_count[5]), 
         .D(spi_byte_count[0]), .Z(n7629)) /* synthesis lut_function=((B+((D)+!C))+!A) */ ;
    defparam i3417_2_lut_4_lut.init = 16'hffdf;
    LUT4 i3647_2_lut_4_lut (.A(n8881), .B(spi_byte_count[4]), .C(spi_byte_count[5]), 
         .D(spi_byte_count[0]), .Z(n7859)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i3647_2_lut_4_lut.init = 16'h2000;
    LUT4 fpga_cs_n_N_339_I_0_1861_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_expected_length_31__N_2085[0]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1280)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1861_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15347_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_expected_length_31__N_2085[0]), 
         .C(n7629), .D(fpga_cs_n_c), .Z(spi_extension_length_15__N_1343)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15347_2_lut_4_lut.init = 16'h0035;
    LUT4 i15283_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_frame_sequence[3]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1526)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15283_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1829_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_update_flags[0]), 
         .C(n7626), .D(fpga_cs_n_c), .Z(spi_update_flags_15__N_1184)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1829_2_lut_4_lut.init = 16'h00ca;
    CCU2D add_2850_5 (.A0(phase_acc[4]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_acc[5]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17160), .COUT(n17161), .S0(phase_acc_next[4]), .S1(phase_acc_next[5]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_5.INIT0 = 16'h5555;
    defparam add_2850_5.INIT1 = 16'h5aaa;
    defparam add_2850_5.INJECT1_0 = "NO";
    defparam add_2850_5.INJECT1_1 = "NO";
    LUT4 i15299_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_update_flags[0]), .C(n7626), 
         .D(fpga_cs_n_c), .Z(spi_update_flags_15__N_1247)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15299_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1902_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_frame_sequence[7]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1394)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1902_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15271_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_frame_sequence[7]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1514)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15271_2_lut_4_lut.init = 16'h0035;
    LUT4 i15891_2_lut (.A(frame_toggle_spi_N_2870), .B(accepted_sequence_spi_31__N_2262), 
         .Z(fpga_cs_n_c_enable_4)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(255[8] 271[4])
    defparam i15891_2_lut.init = 16'h4444;
    LUT4 i15578_2_lut_4_lut (.A(rgb_values[68]), .B(rgb_values[60]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_709)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15578_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_2_lut_adj_161 (.A(stop_toggle_spi), .B(stop_toggle_spi_N_2878), 
         .Z(stop_toggle_spi_N_2877)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i1_2_lut_adj_161.init = 16'h6666;
    LUT4 i6069_1_lut (.A(spi1_sck_c_enable_77), .Z(spi1_sck_c_enable_97)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6069_1_lut.init = 16'h5555;
    LUT4 i6067_1_lut (.A(spi1_sck_c_enable_76), .Z(spi1_sck_c_enable_98)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6067_1_lut.init = 16'h5555;
    LUT4 i6066_1_lut (.A(spi1_sck_c_enable_75), .Z(spi1_sck_c_enable_99)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6066_1_lut.init = 16'h5555;
    LUT4 i6065_1_lut (.A(spi1_sck_c_enable_74), .Z(spi1_sck_c_enable_100)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6065_1_lut.init = 16'h5555;
    LUT4 i6064_1_lut (.A(spi1_sck_c_enable_73), .Z(spi1_sck_c_enable_101)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6064_1_lut.init = 16'h5555;
    LUT4 i6063_1_lut (.A(spi1_sck_c_enable_72), .Z(spi1_sck_c_enable_102)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6063_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1564_2_lut_4_lut (.A(rgb_values[68]), .B(rgb_values[60]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_393)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1564_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6062_1_lut (.A(spi1_sck_c_enable_71), .Z(spi1_sck_c_enable_103)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6062_1_lut.init = 16'h5555;
    LUT4 i6061_1_lut (.A(spi1_sck_c_enable_70), .Z(spi1_sck_c_enable_104)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6061_1_lut.init = 16'h5555;
    LUT4 i15581_2_lut_4_lut (.A(rgb_values[67]), .B(rgb_values[59]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_712)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15581_2_lut_4_lut.init = 16'h0035;
    LUT4 i6060_1_lut (.A(spi1_sck_c_enable_69), .Z(spi1_sck_c_enable_105)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6060_1_lut.init = 16'h5555;
    LUT4 i6059_1_lut (.A(spi1_sck_c_enable_68), .Z(spi1_sck_c_enable_106)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6059_1_lut.init = 16'h5555;
    LUT4 i6058_1_lut (.A(spi1_sck_c_enable_67), .Z(spi1_sck_c_enable_107)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6058_1_lut.init = 16'h5555;
    LUT4 i6057_1_lut (.A(spi1_sck_c_enable_66), .Z(spi1_sck_c_enable_108)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6057_1_lut.init = 16'h5555;
    LUT4 i6056_1_lut (.A(spi1_sck_c_enable_65), .Z(spi1_sck_c_enable_109)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6056_1_lut.init = 16'h5555;
    CCU2D add_2850_3 (.A0(phase_acc[2]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_acc[3]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17159), .COUT(n17160), .S0(phase_acc_next[2]), .S1(phase_acc_next[3]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_3.INIT0 = 16'h5aaa;
    defparam add_2850_3.INIT1 = 16'h5aaa;
    defparam add_2850_3.INJECT1_0 = "NO";
    defparam add_2850_3.INJECT1_1 = "NO";
    LUT4 i6053_1_lut (.A(spi1_sck_c_enable_64), .Z(spi1_sck_c_enable_110)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6053_1_lut.init = 16'h5555;
    LUT4 i6050_1_lut (.A(spi1_sck_c_enable_63), .Z(spi1_sck_c_enable_111)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6050_1_lut.init = 16'h5555;
    LUT4 i1_4_lut_adj_162 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[80]), 
         .C(n26_adj_3507), .D(phase_acc[31]), .Z(us_tx_c_80)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_162.init = 16'h2080;
    LUT4 i6049_1_lut (.A(spi1_sck_c_enable_62), .Z(spi1_sck_c_enable_112)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6049_1_lut.init = 16'h5555;
    LUT4 i6048_1_lut (.A(spi1_sck_c_enable_61), .Z(spi1_sck_c_enable_113)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6048_1_lut.init = 16'h5555;
    LUT4 i6047_1_lut (.A(spi1_sck_c_enable_60), .Z(spi1_sck_c_enable_114)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6047_1_lut.init = 16'h5555;
    LUT4 i6046_1_lut (.A(spi1_sck_c_enable_59), .Z(spi1_sck_c_enable_115)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6046_1_lut.init = 16'h5555;
    LUT4 i6045_1_lut (.A(spi1_sck_c_enable_58), .Z(spi1_sck_c_enable_116)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6045_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1909_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_frame_sequence[0]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1408)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1909_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1565_2_lut_4_lut (.A(rgb_values[67]), .B(rgb_values[59]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_395)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1565_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6044_1_lut (.A(spi1_sck_c_enable_57), .Z(spi1_sck_c_enable_117)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6044_1_lut.init = 16'h5555;
    LUT4 i6043_1_lut (.A(spi1_sck_c_enable_56), .Z(spi1_sck_c_enable_118)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6043_1_lut.init = 16'h5555;
    LUT4 i6042_1_lut (.A(spi1_sck_c_enable_55), .Z(spi1_sck_c_enable_119)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6042_1_lut.init = 16'h5555;
    LUT4 i6041_1_lut (.A(spi1_sck_c_enable_54), .Z(spi1_sck_c_enable_120)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6041_1_lut.init = 16'h5555;
    LUT4 i6040_1_lut (.A(spi1_sck_c_enable_53), .Z(spi1_sck_c_enable_122)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6040_1_lut.init = 16'h5555;
    LUT4 i6039_1_lut (.A(spi1_sck_c_enable_52), .Z(spi1_sck_c_enable_123)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6039_1_lut.init = 16'h5555;
    LUT4 i6038_1_lut (.A(spi1_sck_c_enable_51), .Z(spi1_sck_c_enable_124)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6038_1_lut.init = 16'h5555;
    LUT4 i6037_1_lut (.A(spi1_sck_c_enable_50), .Z(spi1_sck_c_enable_125)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6037_1_lut.init = 16'h5555;
    LUT4 n14_bdd_4_lut (.A(n14_adj_3399), .B(spi_command[0]), .C(spi_command[1]), 
         .D(n10_adj_3392), .Z(stop_toggle_spi_N_2878)) /* synthesis lut_function=(!(A+(B (C+(D))+!B ((D)+!C)))) */ ;
    defparam n14_bdd_4_lut.init = 16'h0014;
    CCU2D add_2850_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(phase_acc[1]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n17159), .S1(phase_acc_next[1]));   // src/umh_fpga_top.v(139[30:54])
    defparam add_2850_1.INIT0 = 16'hF000;
    defparam add_2850_1.INIT1 = 16'h5555;
    defparam add_2850_1.INJECT1_0 = "NO";
    defparam add_2850_1.INJECT1_1 = "NO";
    LUT4 i6036_1_lut (.A(spi1_sck_c_enable_49), .Z(spi1_sck_c_enable_126)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6036_1_lut.init = 16'h5555;
    LUT4 i6035_1_lut (.A(spi1_sck_c_enable_48), .Z(spi1_sck_c_enable_127)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6035_1_lut.init = 16'h5555;
    LUT4 i6034_1_lut (.A(spi1_sck_c_enable_47), .Z(spi1_sck_c_enable_128)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6034_1_lut.init = 16'h5555;
    LUT4 i6033_1_lut (.A(spi1_sck_c_enable_46), .Z(spi1_sck_c_enable_129)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6033_1_lut.init = 16'h5555;
    LUT4 i6032_1_lut (.A(spi1_sck_c_enable_45), .Z(spi1_sck_c_enable_130)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6032_1_lut.init = 16'h5555;
    LUT4 i6031_1_lut (.A(spi1_sck_c_enable_44), .Z(spi1_sck_c_enable_131)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6031_1_lut.init = 16'h5555;
    LUT4 i6030_1_lut (.A(spi1_sck_c_enable_43), .Z(spi1_sck_c_enable_132)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6030_1_lut.init = 16'h5555;
    LUT4 i6029_1_lut (.A(spi1_sck_c_enable_42), .Z(spi1_sck_c_enable_133)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6029_1_lut.init = 16'h5555;
    LUT4 i15584_2_lut_4_lut (.A(rgb_values[66]), .B(rgb_values[58]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_715)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15584_2_lut_4_lut.init = 16'h0035;
    LUT4 i6028_1_lut (.A(spi1_sck_c_enable_41), .Z(spi1_sck_c_enable_134)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6028_1_lut.init = 16'h5555;
    LUT4 i5639_3_lut (.A(n9850), .B(n9849), .C(spi1_sck_c_enable_70), 
         .Z(rgb_values[88])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5639_3_lut.init = 16'hcaca;
    LUT4 i6027_1_lut (.A(spi1_sck_c_enable_40), .Z(spi1_sck_c_enable_135)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6027_1_lut.init = 16'h5555;
    LUT4 i15292_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_frame_sequence[0]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1535)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15292_2_lut_4_lut.init = 16'h0035;
    LUT4 i6026_1_lut (.A(spi1_sck_c_enable_39), .Z(spi1_sck_c_enable_136)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6026_1_lut.init = 16'h5555;
    LUT4 i9760_2_lut (.A(n9561), .B(n9560), .Z(spi_frame_sequence[12])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9760_2_lut.init = 16'h8888;
    LUT4 i9652_2_lut (.A(n9445), .B(n9444), .Z(spi_rx_shift[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9652_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1626_2_lut_4_lut (.A(rgb_values[6]), .B(spi_rx_shift[5]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_517)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1626_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1566_2_lut_4_lut (.A(rgb_values[66]), .B(rgb_values[58]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_397)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1566_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6025_1_lut (.A(spi1_sck_c_enable_38), .Z(spi1_sck_c_enable_137)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6025_1_lut.init = 16'h5555;
    LUT4 i6024_1_lut (.A(spi1_sck_c_enable_37), .Z(spi1_sck_c_enable_138)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6024_1_lut.init = 16'h5555;
    LUT4 i15764_2_lut_4_lut (.A(rgb_values[6]), .B(spi_rx_shift[5]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_895)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15764_2_lut_4_lut.init = 16'h0035;
    LUT4 mux_2523_i3_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[2]), 
         .D(spi_rx_shift[1]), .Z(spi_version_7__N_1107[2])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i3_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i9701_2_lut (.A(load_index[1]), .B(load_index[2]), .Z(n13820)) /* synthesis lut_function=(A (B)) */ ;
    defparam i9701_2_lut.init = 16'h8888;
    LUT4 mux_2523_i2_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[1]), 
         .D(spi_rx_shift[0]), .Z(spi_version_7__N_1107[1])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i2_3_lut_4_lut.init = 16'hf2d0;
    CCU2D add_2849_7 (.A0(n9721), .B0(n9720), .C0(GND_net), .D0(GND_net), 
          .A1(n9725), .B1(n9724), .C1(GND_net), .D1(GND_net), .CIN(n17157), 
          .S0(spi_channel_index_6__N_1731[5]), .S1(spi_channel_index_6__N_1731[6]));   // src/umh_fpga_top.v(188[13] 233[20])
    defparam add_2849_7.INIT0 = 16'h7888;
    defparam add_2849_7.INIT1 = 16'h7888;
    defparam add_2849_7.INJECT1_0 = "NO";
    defparam add_2849_7.INJECT1_1 = "NO";
    LUT4 i6023_1_lut (.A(spi1_sck_c_enable_36), .Z(spi1_sck_c_enable_139)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6023_1_lut.init = 16'h5555;
    LUT4 i6022_1_lut (.A(spi1_sck_c_enable_35), .Z(spi1_sck_c_enable_140)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6022_1_lut.init = 16'h5555;
    LUT4 mux_2523_i5_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[4]), 
         .D(spi_rx_shift[3]), .Z(spi_version_7__N_1107[4])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i5_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i15587_2_lut_4_lut (.A(rgb_values[65]), .B(rgb_values[57]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_718)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15587_2_lut_4_lut.init = 16'h0035;
    LUT4 i6979_3_lut (.A(n62_adj_3401), .B(n11180), .C(status_bit_index[5]), 
         .Z(n11181)) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(51[11:27])
    defparam i6979_3_lut.init = 16'hcaca;
    LUT4 mux_2523_i4_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[3]), 
         .D(spi_rx_shift[2]), .Z(spi_version_7__N_1107[3])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i4_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_2523_i1_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[0]), 
         .D(spi1_mosi_c), .Z(spi_version_7__N_1107[0])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i1_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_2523_i6_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[5]), 
         .D(spi_rx_shift[4]), .Z(spi_version_7__N_1107[5])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i6_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i9747_2_lut (.A(level_mem_dout[7]), .B(level_mem_dout[8]), .Z(n2315)) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(306[37:83])
    defparam i9747_2_lut.init = 16'h8888;
    LUT4 i6021_1_lut (.A(spi1_sck_c_enable_34), .Z(spi1_sck_c_enable_141)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6021_1_lut.init = 16'h5555;
    LUT4 i6020_1_lut (.A(spi1_sck_c_enable_33), .Z(spi1_sck_c_enable_142)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6020_1_lut.init = 16'h5555;
    LUT4 mux_2523_i7_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[6]), 
         .D(spi_rx_shift[5]), .Z(spi_version_7__N_1107[6])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i7_3_lut_4_lut.init = 16'hf2d0;
    LUT4 mux_2523_i8_3_lut_4_lut (.A(n14028), .B(n9019), .C(spi_version[7]), 
         .D(spi_rx_shift[6]), .Z(spi_version_7__N_1107[7])) /* synthesis lut_function=(A (B (C)+!B (D))+!A (C)) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam mux_2523_i8_3_lut_4_lut.init = 16'hf2d0;
    LUT4 i30_3_lut_4_lut_adj_163 (.A(amplitude_phase[6]), .B(\level_active[3] [0]), 
         .C(\level_active[3] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3509)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_163.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_164 (.A(amplitude_phase[6]), .B(\level_active[37] [0]), 
         .C(\level_active[37] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3449)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_164.init = 16'h40f4;
    LUT4 i6019_1_lut (.A(spi1_sck_c_enable_32), .Z(spi1_sck_c_enable_143)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6019_1_lut.init = 16'h5555;
    LUT4 equal_2342_i8_2_lut (.A(load_index[1]), .B(load_index[2]), .Z(n8_adj_3435)) /* synthesis lut_function=(A+!(B)) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2342_i8_2_lut.init = 16'hbbbb;
    LUT4 i6018_1_lut (.A(spi1_sck_c_enable_31), .Z(spi1_sck_c_enable_144)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6018_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1789_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_command[0]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1072)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1789_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1567_2_lut_4_lut (.A(rgb_values[65]), .B(rgb_values[57]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_399)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1567_2_lut_4_lut.init = 16'h00ca;
    LUT4 i2_4_lut_adj_165 (.A(n26_adj_3453), .B(phase_active[16]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_16)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_165.init = 16'h2080;
    CCU2D add_2849_5 (.A0(n9713), .B0(n9712), .C0(GND_net), .D0(GND_net), 
          .A1(n9717), .B1(n9716), .C1(GND_net), .D1(GND_net), .CIN(n17156), 
          .COUT(n17157), .S0(spi_channel_index_6__N_1731[3]), .S1(spi_channel_index_6__N_1731[4]));   // src/umh_fpga_top.v(188[13] 233[20])
    defparam add_2849_5.INIT0 = 16'h7888;
    defparam add_2849_5.INIT1 = 16'h7888;
    defparam add_2849_5.INJECT1_0 = "NO";
    defparam add_2849_5.INJECT1_1 = "NO";
    LUT4 i15172_2_lut_4_lut (.A(spi1_mosi_c), .B(spi_command[0]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1103)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15172_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1903_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_frame_sequence[6]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1396)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1903_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9878_2_lut (.A(n9285), .B(n9284), .Z(spi_channel_index[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9878_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1622_2_lut_4_lut (.A(rgb_values[10]), .B(rgb_values[2]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_509)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1622_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_4_lut_adj_166 (.A(spi_channel_index[2]), .B(spi_channel_index[0]), 
         .C(n18782), .D(spi_channel_index[4]), .Z(n18531)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i1_4_lut_adj_166.init = 16'hbfff;
    LUT4 i15274_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_frame_sequence[6]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1517)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15274_2_lut_4_lut.init = 16'h0035;
    LUT4 i6017_1_lut (.A(spi1_sck_c_enable_30), .Z(spi1_sck_c_enable_145)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6017_1_lut.init = 16'h5555;
    LUT4 i6016_1_lut (.A(spi1_sck_c_enable_29), .Z(spi1_sck_c_enable_146)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6016_1_lut.init = 16'h5555;
    LUT4 i15752_2_lut_4_lut (.A(rgb_values[10]), .B(rgb_values[2]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_883)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15752_2_lut_4_lut.init = 16'h0035;
    LUT4 i6015_1_lut (.A(spi1_sck_c_enable_28), .Z(spi1_sck_c_enable_147)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6015_1_lut.init = 16'h5555;
    LUT4 spi_byte_count_5__bdd_4_lut_16184 (.A(spi_byte_count[1]), .B(spi_byte_count[3]), 
         .C(spi_byte_count[2]), .D(spi_byte_count[4]), .Z(n20744)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A ((C+(D))+!B))) */ ;
    defparam spi_byte_count_5__bdd_4_lut_16184.init = 16'h0024;
    LUT4 i6014_1_lut (.A(spi1_sck_c_enable_27), .Z(spi1_sck_c_enable_148)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6014_1_lut.init = 16'h5555;
    LUT4 spi_byte_count_5__bdd_4_lut (.A(spi_byte_count[5]), .B(n14030), 
         .C(rgb_values_95__N_630), .D(n20744), .Z(n6814)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam spi_byte_count_5__bdd_4_lut.init = 16'h1000;
    LUT4 fpga_cs_n_N_339_I_0_1907_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_frame_sequence[2]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1404)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1907_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15286_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_frame_sequence[2]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1529)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15286_2_lut_4_lut.init = 16'h0035;
    LUT4 i15641_2_lut_4_lut (.A(rgb_values[47]), .B(rgb_values[39]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_772)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15641_2_lut_4_lut.init = 16'h0035;
    LUT4 i6013_1_lut (.A(spi1_sck_c_enable_26), .Z(spi1_sck_c_enable_149)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6013_1_lut.init = 16'h5555;
    LUT4 i6012_1_lut (.A(spi1_sck_c_enable_25), .Z(spi1_sck_c_enable_150)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6012_1_lut.init = 16'h5555;
    LUT4 i15590_2_lut_4_lut (.A(rgb_values[64]), .B(rgb_values[56]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_721)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15590_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1568_2_lut_4_lut (.A(rgb_values[64]), .B(rgb_values[56]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_401)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1568_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15593_2_lut_4_lut (.A(rgb_values[63]), .B(rgb_values[55]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_724)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15593_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1585_2_lut_4_lut (.A(rgb_values[47]), .B(rgb_values[39]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_435)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1585_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15644_2_lut_4_lut (.A(rgb_values[46]), .B(rgb_values[38]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_775)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15644_2_lut_4_lut.init = 16'h0035;
    LUT4 i9744_2_lut (.A(n9705), .B(n9704), .Z(spi_channel_index[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9744_2_lut.init = 16'h8888;
    LUT4 i9735_2_lut (.A(n9725), .B(n9724), .Z(spi_channel_index[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9735_2_lut.init = 16'h8888;
    LUT4 i9742_2_lut (.A(n9713), .B(n9712), .Z(spi_channel_index[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9742_2_lut.init = 16'h8888;
    LUT4 i9737_2_lut (.A(n9721), .B(n9720), .Z(spi_channel_index[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9737_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1586_2_lut_4_lut (.A(rgb_values[46]), .B(rgb_values[38]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_437)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1586_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5915_3_lut (.A(n10126), .B(n10125), .C(spi1_sck_c_enable_1), 
         .Z(rgb_values[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5915_3_lut.init = 16'hcaca;
    LUT4 i6011_1_lut (.A(spi1_sck_c_enable_24), .Z(spi1_sck_c_enable_151)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6011_1_lut.init = 16'h5555;
    CCU2D add_2849_3 (.A0(n9705), .B0(n9704), .C0(GND_net), .D0(GND_net), 
          .A1(n9709), .B1(n9708), .C1(GND_net), .D1(GND_net), .CIN(n17155), 
          .COUT(n17156), .S0(spi_channel_index_6__N_1731[1]), .S1(spi_channel_index_6__N_1731[2]));   // src/umh_fpga_top.v(188[13] 233[20])
    defparam add_2849_3.INIT0 = 16'h7888;
    defparam add_2849_3.INIT1 = 16'h7888;
    defparam add_2849_3.INJECT1_0 = "NO";
    defparam add_2849_3.INJECT1_1 = "NO";
    LUT4 i9743_2_lut (.A(n9709), .B(n9708), .Z(spi_channel_index[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9743_2_lut.init = 16'h8888;
    LUT4 i6010_1_lut (.A(spi1_sck_c_enable_23), .Z(spi1_sck_c_enable_152)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6010_1_lut.init = 16'h5555;
    LUT4 i5663_3_lut (.A(n9874), .B(n9873), .C(spi1_sck_c_enable_64), 
         .Z(rgb_values[82])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5663_3_lut.init = 16'hcaca;
    LUT4 i5947_3_lut (.A(n10158), .B(n10157), .C(spi1_sck_c_enable_184), 
         .Z(rgb_values[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5947_3_lut.init = 16'hcaca;
    LUT4 i15647_2_lut_4_lut (.A(rgb_values[45]), .B(rgb_values[37]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_778)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15647_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1569_2_lut_4_lut (.A(rgb_values[63]), .B(rgb_values[55]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_403)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1569_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1587_2_lut_4_lut (.A(rgb_values[45]), .B(rgb_values[37]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_439)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1587_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15650_2_lut_4_lut (.A(rgb_values[44]), .B(rgb_values[36]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_781)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15650_2_lut_4_lut.init = 16'h0035;
    LUT4 i15596_2_lut_4_lut (.A(rgb_values[62]), .B(rgb_values[54]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_727)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15596_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1570_2_lut_4_lut (.A(rgb_values[62]), .B(rgb_values[54]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_405)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1570_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1588_2_lut_4_lut (.A(rgb_values[44]), .B(rgb_values[36]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_441)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1588_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9738_2_lut (.A(n9717), .B(n9716), .Z(spi_channel_index[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9738_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_167 (.A(phase_active[77]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3499), .Z(us_tx_c_77)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_167.init = 16'h4800;
    LUT4 i15653_2_lut_4_lut (.A(rgb_values[43]), .B(rgb_values[35]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_784)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15653_2_lut_4_lut.init = 16'h0035;
    LUT4 i6009_1_lut (.A(spi1_sck_c_enable_22), .Z(spi1_sck_c_enable_153)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6009_1_lut.init = 16'h5555;
    LUT4 i15374_2_lut_3_lut (.A(spi_expected_length[23]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1658)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15374_2_lut_3_lut.init = 16'h0707;
    LUT4 i30_3_lut_4_lut_adj_168 (.A(amplitude_phase[6]), .B(\level_active[41] [0]), 
         .C(\level_active[41] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3376)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_168.init = 16'h40f4;
    LUT4 i15371_2_lut_3_lut (.A(spi_expected_length[24]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1655)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15371_2_lut_3_lut.init = 16'h0707;
    LUT4 i15368_2_lut_3_lut (.A(spi_expected_length[25]), .B(n37_adj_3400), 
         .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1652)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15368_2_lut_3_lut.init = 16'h0707;
    LUT4 fpga_cs_n_N_339_I_0_1894_2_lut_4_lut (.A(spi_frame_sequence[15]), 
         .B(spi_rx_shift[6]), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1378)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1894_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6008_1_lut (.A(spi1_sck_c_enable_21), .Z(spi1_sck_c_enable_154)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6008_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1589_2_lut_4_lut (.A(rgb_values[43]), .B(rgb_values[35]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_443)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1589_2_lut_4_lut.init = 16'h00ca;
    LUT4 i30_3_lut_4_lut_adj_169 (.A(amplitude_phase[6]), .B(\level_active[42] [0]), 
         .C(\level_active[42] [1]), .D(amplitude_phase[7]), .Z(n26)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_169.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_170 (.A(amplitude_phase[6]), .B(\level_active[28] [0]), 
         .C(\level_active[28] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3477)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_170.init = 16'h40f4;
    LUT4 i15656_2_lut_4_lut (.A(rgb_values[42]), .B(rgb_values[34]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_787)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15656_2_lut_4_lut.init = 16'h0035;
    LUT4 i9721_2_lut (.A(n9781), .B(n9780), .Z(spi_byte_count[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9721_2_lut.init = 16'h8888;
    LUT4 i9685_2_lut (.A(n9277), .B(n9276), .Z(spi_update_flags[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9685_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1590_2_lut_4_lut (.A(rgb_values[42]), .B(rgb_values[34]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_445)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1590_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1908_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_frame_sequence[1]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1406)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1908_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6007_1_lut (.A(spi1_sck_c_enable_20), .Z(spi1_sck_c_enable_155)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6007_1_lut.init = 16'h5555;
    LUT4 i6006_1_lut (.A(spi1_sck_c_enable_19), .Z(spi1_sck_c_enable_156)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6006_1_lut.init = 16'h5555;
    LUT4 i9881_2_lut (.A(n9817), .B(n9816), .Z(spi_byte_count[15])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9881_2_lut.init = 16'h8888;
    LUT4 i15659_2_lut_4_lut (.A(rgb_values[41]), .B(rgb_values[33]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_790)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15659_2_lut_4_lut.init = 16'h0035;
    LUT4 i9883_2_lut (.A(n9813), .B(n9812), .Z(spi_byte_count[14])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9883_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1591_2_lut_4_lut (.A(rgb_values[41]), .B(rgb_values[33]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_447)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1591_2_lut_4_lut.init = 16'h00ca;
    LUT4 equal_2346_i8_2_lut (.A(load_index[1]), .B(load_index[2]), .Z(n8_adj_3437)) /* synthesis lut_function=(A+(B)) */ ;   // src/umh_fpga_top.v(305[9:33])
    defparam equal_2346_i8_2_lut.init = 16'heeee;
    LUT4 i15662_2_lut_4_lut (.A(rgb_values[40]), .B(rgb_values[32]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_793)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15662_2_lut_4_lut.init = 16'h0035;
    LUT4 i9649_2_lut (.A(n9457), .B(n9456), .Z(spi_rx_shift[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9649_2_lut.init = 16'h8888;
    LUT4 i9650_2_lut (.A(n9453), .B(n9452), .Z(spi_rx_shift[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9650_2_lut.init = 16'h8888;
    LUT4 i9722_2_lut (.A(n9777), .B(n9776), .Z(spi_byte_count[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9722_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1592_2_lut_4_lut (.A(rgb_values[40]), .B(rgb_values[32]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_449)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1592_2_lut_4_lut.init = 16'h00ca;
    LUT4 i6005_1_lut (.A(spi1_sck_c_enable_18), .Z(spi1_sck_c_enable_157)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6005_1_lut.init = 16'h5555;
    LUT4 i9723_2_lut (.A(n9773), .B(n9772), .Z(spi_byte_count[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9723_2_lut.init = 16'h8888;
    LUT4 i6004_1_lut (.A(spi1_sck_c_enable_17), .Z(spi1_sck_c_enable_158)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6004_1_lut.init = 16'h5555;
    LUT4 i9724_2_lut (.A(n9769), .B(n9768), .Z(spi_byte_count[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9724_2_lut.init = 16'h8888;
    LUT4 i9725_2_lut (.A(n9765), .B(n9764), .Z(spi_byte_count[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9725_2_lut.init = 16'h8888;
    LUT4 i15665_2_lut_4_lut (.A(rgb_values[39]), .B(rgb_values[31]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_796)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15665_2_lut_4_lut.init = 16'h0035;
    LUT4 i15289_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_frame_sequence[1]), 
         .C(n8632), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1532)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15289_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1593_2_lut_4_lut (.A(rgb_values[39]), .B(rgb_values[31]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_451)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1593_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9651_2_lut (.A(n9449), .B(n9448), .Z(spi_rx_shift[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9651_2_lut.init = 16'h8888;
    LUT4 i1_4_lut_adj_171 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3452), 
         .C(phase_active[17]), .D(phase_acc[31]), .Z(us_tx_c_17)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_171.init = 16'h0880;
    LUT4 i9653_2_lut (.A(n9441), .B(n9440), .Z(spi_rx_shift[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9653_2_lut.init = 16'h8888;
    LUT4 i9873_2_lut (.A(n9437), .B(n9436), .Z(spi_rx_shift[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9873_2_lut.init = 16'h8888;
    LUT4 i9890_2_lut (.A(n9261), .B(n9260), .Z(spi_rx_shift[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9890_2_lut.init = 16'h8888;
    LUT4 i15668_2_lut_4_lut (.A(rgb_values[38]), .B(rgb_values[30]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_799)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15668_2_lut_4_lut.init = 16'h0035;
    LUT4 i6003_1_lut (.A(spi1_sck_c_enable_16), .Z(spi1_sck_c_enable_159)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6003_1_lut.init = 16'h5555;
    LUT4 i6002_1_lut (.A(spi1_sck_c_enable_15), .Z(spi1_sck_c_enable_160)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6002_1_lut.init = 16'h5555;
    LUT4 i2_4_lut_adj_172 (.A(n26_adj_3457), .B(phase_active[18]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_18)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_172.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1594_2_lut_4_lut (.A(rgb_values[38]), .B(rgb_values[30]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_453)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1594_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15671_2_lut_4_lut (.A(rgb_values[37]), .B(rgb_values[29]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_802)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15671_2_lut_4_lut.init = 16'h0035;
    LUT4 i14633_3_lut (.A(n58_adj_3416), .B(spi_byte_count[9]), .C(rgb_values_95__N_630), 
         .Z(n18752)) /* synthesis lut_function=(A+(B+!(C))) */ ;
    defparam i14633_3_lut.init = 16'hefef;
    LUT4 fpga_cs_n_N_339_I_0_1595_2_lut_4_lut (.A(rgb_values[37]), .B(rgb_values[29]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_455)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1595_2_lut_4_lut.init = 16'h00ca;
    LUT4 i9727_2_lut (.A(n9757), .B(n9756), .Z(spi_channel_field[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9727_2_lut.init = 16'h8888;
    LUT4 spi_byte_count_4__bdd_4_lut_16187 (.A(spi_byte_count[4]), .B(spi_byte_count[6]), 
         .C(n128), .D(spi_byte_count[8]), .Z(n20620)) /* synthesis lut_function=(!(A (B (C))+!A !(B+(C+(D))))) */ ;
    defparam spi_byte_count_4__bdd_4_lut_16187.init = 16'h7f7e;
    LUT4 i15674_2_lut_4_lut (.A(rgb_values[36]), .B(rgb_values[28]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_805)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15674_2_lut_4_lut.init = 16'h0035;
    LUT4 i15599_2_lut_4_lut (.A(rgb_values[61]), .B(rgb_values[53]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_730)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15599_2_lut_4_lut.init = 16'h0035;
    LUT4 i6001_1_lut (.A(spi1_sck_c_enable_14), .Z(spi1_sck_c_enable_161)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6001_1_lut.init = 16'h5555;
    LUT4 i6000_1_lut (.A(spi1_sck_c_enable_13), .Z(spi1_sck_c_enable_162)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i6000_1_lut.init = 16'h5555;
    LUT4 i1_4_lut_adj_173 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3456), 
         .C(phase_active[19]), .D(phase_acc[31]), .Z(us_tx_c_19)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_173.init = 16'h0880;
    LUT4 fpga_cs_n_N_339_I_0_1596_2_lut_4_lut (.A(rgb_values[36]), .B(rgb_values[28]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_457)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1596_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15677_2_lut_4_lut (.A(rgb_values[35]), .B(rgb_values[27]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_808)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15677_2_lut_4_lut.init = 16'h0035;
    LUT4 i5999_1_lut (.A(spi1_sck_c_enable_12), .Z(spi1_sck_c_enable_163)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5999_1_lut.init = 16'h5555;
    LUT4 i5998_1_lut (.A(spi1_sck_c_enable_11), .Z(spi1_sck_c_enable_164)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5998_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1571_2_lut_4_lut (.A(rgb_values[61]), .B(rgb_values[53]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_407)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1571_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1597_2_lut_4_lut (.A(rgb_values[35]), .B(rgb_values[27]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_459)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1597_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_2_lut_adj_174 (.A(rgb_values_95__N_630), .B(n8870), .Z(n18637)) /* synthesis lut_function=(A (B)) */ ;
    defparam i1_2_lut_adj_174.init = 16'h8888;
    LUT4 i9676_2_lut (.A(n9293), .B(n9292), .Z(spi_channel_field[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9676_2_lut.init = 16'h8888;
    LUT4 i30_3_lut_4_lut_adj_175 (.A(amplitude_phase[6]), .B(\level_active[27] [0]), 
         .C(\level_active[27] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3472)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_175.init = 16'h40f4;
    LUT4 i5997_1_lut (.A(spi1_sck_c_enable_10), .Z(spi1_sck_c_enable_165)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5997_1_lut.init = 16'h5555;
    LUT4 i5996_1_lut (.A(spi1_sck_c_enable_9), .Z(spi1_sck_c_enable_166)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5996_1_lut.init = 16'h5555;
    LUT4 i15680_2_lut_4_lut (.A(rgb_values[34]), .B(rgb_values[26]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_811)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15680_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1598_2_lut_4_lut (.A(rgb_values[34]), .B(rgb_values[26]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_461)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1598_2_lut_4_lut.init = 16'h00ca;
    LUT4 i2_4_lut_adj_176 (.A(n26_adj_3461), .B(phase_active[20]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_20)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_176.init = 16'h2080;
    LUT4 i15127_2_lut_3_lut (.A(n9257), .B(n9256), .C(fpga_cs_n_c), .Z(spi_bit_count_2__N_948)) /* synthesis lut_function=(!(A (B+(C))+!A (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i15127_2_lut_3_lut.init = 16'h0707;
    LUT4 i15602_2_lut_4_lut (.A(rgb_values[60]), .B(rgb_values[52]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_733)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15602_2_lut_4_lut.init = 16'h0035;
    LUT4 i15109_2_lut (.A(n9641), .B(n9640), .Z(spi_update_flags[1])) /* synthesis lut_function=(!(A (B))) */ ;   // src/umh_fpga_top.v(200[44] 202[63])
    defparam i15109_2_lut.init = 16'h7777;
    LUT4 i15247_2_lut_4_lut (.A(spi_frame_sequence[15]), .B(spi_rx_shift[6]), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1490)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15247_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1950_2_lut_3_lut (.A(spi_expected_length[23]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1554)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1950_2_lut_3_lut.init = 16'h0808;
    LUT4 i30_3_lut_4_lut_adj_177 (.A(amplitude_phase[6]), .B(\level_active[26] [0]), 
         .C(\level_active[26] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3473)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_177.init = 16'h40f4;
    LUT4 i1_4_lut_adj_178 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3460), 
         .C(phase_active[21]), .D(phase_acc[31]), .Z(us_tx_c_21)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_178.init = 16'h0880;
    LUT4 fpga_cs_n_N_339_I_0_1572_2_lut_4_lut (.A(rgb_values[60]), .B(rgb_values[52]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_409)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1572_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5995_1_lut (.A(spi1_sck_c_enable_8), .Z(spi1_sck_c_enable_167)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5995_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1949_2_lut_3_lut (.A(spi_expected_length[24]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1552)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1949_2_lut_3_lut.init = 16'h0808;
    LUT4 i5994_1_lut (.A(spi1_sck_c_enable_7), .Z(spi1_sck_c_enable_168)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5994_1_lut.init = 16'h5555;
    LUT4 i15683_2_lut_4_lut (.A(rgb_values[33]), .B(rgb_values[25]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_814)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15683_2_lut_4_lut.init = 16'h0035;
    LUT4 i5993_1_lut (.A(spi1_sck_c_enable_6), .Z(spi1_sck_c_enable_169)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5993_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1621_2_lut_4_lut (.A(rgb_values[11]), .B(rgb_values[3]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_507)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1621_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5992_1_lut (.A(spi1_sck_c_enable_5), .Z(spi1_sck_c_enable_170)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5992_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1599_2_lut_4_lut (.A(rgb_values[33]), .B(rgb_values[25]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_463)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1599_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15686_2_lut_4_lut (.A(rgb_values[32]), .B(rgb_values[24]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_817)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15686_2_lut_4_lut.init = 16'h0035;
    LUT4 i9680_2_lut (.A(n9433), .B(n9432), .Z(spi_bit_count[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9680_2_lut.init = 16'h8888;
    LUT4 i5991_1_lut (.A(spi1_sck_c_enable_4), .Z(spi1_sck_c_enable_171)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5991_1_lut.init = 16'h5555;
    LUT4 i5990_1_lut (.A(spi1_sck_c_enable_3), .Z(spi1_sck_c_enable_172)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5990_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1600_2_lut_4_lut (.A(rgb_values[32]), .B(rgb_values[24]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_465)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1600_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15749_2_lut_4_lut (.A(rgb_values[11]), .B(rgb_values[3]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_880)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15749_2_lut_4_lut.init = 16'h0035;
    LUT4 i9719_2_lut (.A(n9789), .B(n9788), .Z(spi_byte_count[8])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9719_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1620_2_lut_4_lut (.A(rgb_values[12]), .B(rgb_values[4]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_505)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1620_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15689_2_lut_4_lut (.A(rgb_values[31]), .B(rgb_values[23]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_820)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15689_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_2_lut_adj_179 (.A(n66), .B(n14030), .Z(n58_adj_3416)) /* synthesis lut_function=(!((B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i1_2_lut_adj_179.init = 16'h2222;
    LUT4 i5989_1_lut (.A(spi1_sck_c_enable_2), .Z(spi1_sck_c_enable_173)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5989_1_lut.init = 16'h5555;
    LUT4 i5988_1_lut (.A(spi1_sck_c_enable_1), .Z(spi1_sck_c_enable_174)) /* synthesis lut_function=(!(A)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5988_1_lut.init = 16'h5555;
    LUT4 fpga_cs_n_N_339_I_0_1601_2_lut_4_lut (.A(rgb_values[31]), .B(rgb_values[23]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_467)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1601_2_lut_4_lut.init = 16'h00ca;
    LUT4 i2_4_lut_adj_180 (.A(n26_adj_3465), .B(phase_active[22]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_22)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_180.init = 16'h2080;
    LUT4 i1_4_lut_adj_181 (.A(n60_adj_3417), .B(spi_byte_count[4]), .C(n18634), 
         .D(spi_byte_count[5]), .Z(n66)) /* synthesis lut_function=(!(A (B ((D)+!C)+!B !(C+!(D)))+!A (B ((D)+!C)+!B !(C (D))))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i1_4_lut_adj_181.init = 16'h30e2;
    LUT4 i81_3_lut (.A(spi_byte_count[1]), .B(spi_byte_count[3]), .C(spi_byte_count[2]), 
         .Z(n60_adj_3417)) /* synthesis lut_function=(!(A (B+!(C))+!A (C))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i81_3_lut.init = 16'h2525;
    LUT4 i9726_2_lut (.A(n9761), .B(n9760), .Z(spi_byte_count[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9726_2_lut.init = 16'h8888;
    LUT4 i2_3_lut (.A(n13789), .B(n7_adj_3395), .C(n13787), .Z(n13957)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i2_3_lut.init = 16'hfefe;
    LUT4 i5911_3_lut (.A(n10122), .B(n10121), .C(spi1_sck_c_enable_2), 
         .Z(rgb_values[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5911_3_lut.init = 16'hcaca;
    LUT4 i5943_3_lut (.A(n10154), .B(n10153), .C(spi1_sck_c_enable_185), 
         .Z(rgb_values[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5943_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_182 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[2]), 
         .C(n26_adj_3511), .D(phase_acc[31]), .Z(us_tx_c_2)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_182.init = 16'h2080;
    LUT4 i9915_2_lut (.A(n9621), .B(n9620), .Z(spi_frame_sequence[27])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9915_2_lut.init = 16'h8888;
    LUT4 i15746_2_lut_4_lut (.A(rgb_values[12]), .B(rgb_values[4]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_877)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15746_2_lut_4_lut.init = 16'h0035;
    LUT4 i30_3_lut_4_lut_adj_183 (.A(amplitude_phase[6]), .B(\level_active[35] [0]), 
         .C(\level_active[35] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3488)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_183.init = 16'h40f4;
    LUT4 i9662_2_lut (.A(n9297), .B(n9296), .Z(spi_byte_count[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9662_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1619_2_lut_4_lut (.A(rgb_values[13]), .B(rgb_values[5]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_503)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1619_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1895_2_lut_4_lut (.A(spi_frame_sequence[14]), 
         .B(spi_rx_shift[5]), .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1380)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;
    defparam fpga_cs_n_N_339_I_0_1895_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15250_2_lut_4_lut (.A(spi_frame_sequence[14]), .B(spi_rx_shift[5]), 
         .C(n8565), .D(fpga_cs_n_c), .Z(spi_frame_sequence_31__N_1493)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;
    defparam i15250_2_lut_4_lut.init = 16'h0035;
    LUT4 i15743_2_lut_4_lut (.A(rgb_values[13]), .B(rgb_values[5]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_874)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15743_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_4_lut_adj_184 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[78]), 
         .C(n26_adj_3506), .D(phase_acc[31]), .Z(us_tx_c_78)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_184.init = 16'h2080;
    LUT4 i1_4_lut_adj_185 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3464), 
         .C(phase_active[23]), .D(phase_acc[31]), .Z(us_tx_c_23)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_185.init = 16'h0880;
    LUT4 i2_4_lut_adj_186 (.A(phase_active[79]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3501), .Z(us_tx_c_79)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_186.init = 16'h4800;
    LUT4 i5907_3_lut (.A(n10118), .B(n10117), .C(spi1_sck_c_enable_3), 
         .Z(rgb_values[21])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5907_3_lut.init = 16'hcaca;
    LUT4 i5939_3_lut (.A(n10150), .B(n10149), .C(spi1_sck_c_enable_186), 
         .Z(rgb_values[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5939_3_lut.init = 16'hcaca;
    LUT4 load_index_6__I_0_i8_2_lut (.A(load_index[1]), .B(load_index[2]), 
         .Z(n8_adj_3436)) /* synthesis lut_function=((B)+!A) */ ;   // src/umh_fpga_top.v(307[13:32])
    defparam load_index_6__I_0_i8_2_lut.init = 16'hdddd;
    LUT4 fpga_cs_n_N_339_I_0_1961_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[12]), 
         .Z(spi_expected_length_31__N_1576)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1961_2_lut.init = 16'h4444;
    LUT4 i3_4_lut_adj_187 (.A(n18818), .B(n9009), .C(spi_command[4]), 
         .D(n9003), .Z(fpga_cs_n_c_enable_1)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i3_4_lut_adj_187.init = 16'hfffe;
    LUT4 i9645_2_lut (.A(n9473), .B(n9472), .Z(spi_command[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9645_2_lut.init = 16'h8888;
    LUT4 i2_3_lut_adj_188 (.A(spi_command[7]), .B(spi_command[5]), .C(spi_command[6]), 
         .Z(n9003)) /* synthesis lut_function=(A+(B+(C))) */ ;   // src/umh_fpga_top.v(268[18:38])
    defparam i2_3_lut_adj_188.init = 16'hfefe;
    LUT4 i9642_2_lut (.A(n9485), .B(n9484), .Z(spi_command[7])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9642_2_lut.init = 16'h8888;
    LUT4 i9644_2_lut (.A(n9477), .B(n9476), .Z(spi_command[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9644_2_lut.init = 16'h8888;
    LUT4 i15160_2_lut_4_lut (.A(spi_rx_shift[3]), .B(spi_command[4]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1091)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15160_2_lut_4_lut.init = 16'h0035;
    LUT4 i9643_2_lut (.A(n9481), .B(n9480), .Z(spi_command[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9643_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_189 (.A(phase_active[81]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3503), .Z(us_tx_c_81)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_189.init = 16'h4800;
    LUT4 i2_4_lut_adj_190 (.A(n26_adj_3469), .B(phase_active[24]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_24)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_190.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1618_2_lut_4_lut (.A(rgb_values[14]), .B(rgb_values[6]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_501)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1618_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15437_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[2]), 
         .Z(spi_expected_length_31__N_1721)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15437_2_lut.init = 16'h1111;
    LUT4 fpga_cs_n_N_339_I_0_1963_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[10]), 
         .Z(spi_expected_length_31__N_1580)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1963_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1951_2_lut_3_lut (.A(spi_expected_length[22]), 
         .B(n37_adj_3400), .C(fpga_cs_n_c), .Z(spi_expected_length_31__N_1556)) /* synthesis lut_function=(!(((C)+!B)+!A)) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1951_2_lut_3_lut.init = 16'h0808;
    LUT4 i2_4_lut_adj_191 (.A(n18842), .B(spi_version[0]), .C(n18846), 
         .D(n14_adj_3399), .Z(frame_toggle_spi_N_2870)) /* synthesis lut_function=(!(A+((C+(D))+!B))) */ ;
    defparam i2_4_lut_adj_191.init = 16'h0004;
    LUT4 i14721_4_lut (.A(n18802), .B(n18834), .C(spi_extension_length[12]), 
         .D(spi_version[2]), .Z(n18842)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14721_4_lut.init = 16'hfffe;
    LUT4 i14725_4_lut (.A(spi_extension_length[7]), .B(n18844), .C(n18792), 
         .D(spi_extension_length[9]), .Z(n18846)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14725_4_lut.init = 16'hfffe;
    LUT4 i14682_3_lut (.A(spi_version[4]), .B(spi_command[0]), .C(n9009), 
         .Z(n18802)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i14682_3_lut.init = 16'hfefe;
    LUT4 i14713_4_lut (.A(spi_extension_length[8]), .B(spi_extension_length[11]), 
         .C(spi_extension_length[10]), .D(spi_extension_length[6]), .Z(n18834)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14713_4_lut.init = 16'hfffe;
    LUT4 i14723_4_lut (.A(spi_extension_length[13]), .B(n18838), .C(spi_version[1]), 
         .D(spi_version[3]), .Z(n18844)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14723_4_lut.init = 16'hfffe;
    LUT4 i14717_4_lut (.A(spi_version[6]), .B(spi_version[7]), .C(n13994), 
         .D(spi_version[5]), .Z(n18838)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i14717_4_lut.init = 16'hfffe;
    LUT4 i9875_4_lut (.A(spi_extension_length[3]), .B(spi_extension_length[5]), 
         .C(n8_adj_3517), .D(spi_extension_length[4]), .Z(n13994)) /* synthesis lut_function=(A (B)+!A (B (C+(D)))) */ ;
    defparam i9875_4_lut.init = 16'hccc8;
    LUT4 i3_3_lut_adj_192 (.A(spi_extension_length[2]), .B(spi_expected_length_31__N_2085[0]), 
         .C(spi_expected_length_31__N_2085[1]), .Z(n8_adj_3517)) /* synthesis lut_function=(A+(B+(C))) */ ;
    defparam i3_3_lut_adj_192.init = 16'hfefe;
    LUT4 i9809_2_lut (.A(n9649), .B(n9648), .Z(spi_extension_length[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9809_2_lut.init = 16'h8888;
    LUT4 i9879_2_lut (.A(n9281), .B(n9280), .Z(spi_expected_length_31__N_2085[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9879_2_lut.init = 16'h8888;
    LUT4 i9810_2_lut (.A(n9645), .B(n9644), .Z(spi_expected_length_31__N_2085[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9810_2_lut.init = 16'h8888;
    LUT4 i9808_2_lut (.A(n9653), .B(n9652), .Z(spi_extension_length[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9808_2_lut.init = 16'h8888;
    LUT4 i9806_2_lut (.A(n9661), .B(n9660), .Z(spi_extension_length[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9806_2_lut.init = 16'h8888;
    LUT4 i9807_2_lut (.A(n9657), .B(n9656), .Z(spi_extension_length[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9807_2_lut.init = 16'h8888;
    LUT4 i9636_2_lut (.A(n9509), .B(n9508), .Z(spi_version[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9636_2_lut.init = 16'h8888;
    LUT4 i9635_2_lut (.A(n9513), .B(n9512), .Z(spi_version[7])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9635_2_lut.init = 16'h8888;
    LUT4 i9637_2_lut (.A(n9505), .B(n9504), .Z(spi_version[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9637_2_lut.init = 16'h8888;
    LUT4 i9754_2_lut (.A(n9693), .B(n9692), .Z(spi_extension_length[13])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9754_2_lut.init = 16'h8888;
    LUT4 i9641_2_lut (.A(n9489), .B(n9488), .Z(spi_version[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9641_2_lut.init = 16'h8888;
    LUT4 i9639_2_lut (.A(n9497), .B(n9496), .Z(spi_version[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9639_2_lut.init = 16'h8888;
    LUT4 i9759_2_lut (.A(n9673), .B(n9672), .Z(spi_extension_length[8])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9759_2_lut.init = 16'h8888;
    LUT4 i9756_2_lut (.A(n9685), .B(n9684), .Z(spi_extension_length[11])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9756_2_lut.init = 16'h8888;
    LUT4 i9757_2_lut (.A(n9681), .B(n9680), .Z(spi_extension_length[10])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9757_2_lut.init = 16'h8888;
    LUT4 i9805_2_lut (.A(n9665), .B(n9664), .Z(spi_extension_length[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9805_2_lut.init = 16'h8888;
    LUT4 i9638_2_lut (.A(n9501), .B(n9500), .Z(spi_version[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9638_2_lut.init = 16'h8888;
    LUT4 i9888_2_lut (.A(n9265), .B(n9264), .Z(spi_command[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9888_2_lut.init = 16'h8888;
    LUT4 i9755_2_lut (.A(n9689), .B(n9688), .Z(spi_extension_length[12])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9755_2_lut.init = 16'h8888;
    LUT4 i9640_2_lut (.A(n9493), .B(n9492), .Z(spi_version[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9640_2_lut.init = 16'h8888;
    LUT4 i9750_2_lut (.A(n9269), .B(n9268), .Z(spi_version[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9750_2_lut.init = 16'h8888;
    LUT4 i15413_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[10]), 
         .Z(spi_expected_length_31__N_1697)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15413_2_lut.init = 16'h1111;
    LUT4 fpga_cs_n_N_339_I_0_1962_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[11]), 
         .Z(spi_expected_length_31__N_1578)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1962_2_lut.init = 16'h4444;
    LUT4 i15410_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[11]), 
         .Z(spi_expected_length_31__N_1694)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15410_2_lut.init = 16'h1111;
    LUT4 i15407_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[12]), 
         .Z(spi_expected_length_31__N_1691)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15407_2_lut.init = 16'h1111;
    LUT4 fpga_cs_n_N_339_I_0_1960_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[13]), 
         .Z(spi_expected_length_31__N_1574)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1960_2_lut.init = 16'h4444;
    LUT4 i15404_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[13]), 
         .Z(spi_expected_length_31__N_1688)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15404_2_lut.init = 16'h1111;
    LUT4 i1_4_lut_adj_193 (.A(status_flags_wire_15__N_2050[4]), .B(n29), 
         .C(phase_active[39]), .D(phase_acc[31]), .Z(us_tx_c_39)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_193.init = 16'h0880;
    LUT4 i30_3_lut_4_lut_adj_194 (.A(amplitude_phase[6]), .B(\level_active[31] [0]), 
         .C(\level_active[31] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3480)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_194.init = 16'h40f4;
    LUT4 i30_3_lut_4_lut_adj_195 (.A(amplitude_phase[6]), .B(\level_active[30] [0]), 
         .C(\level_active[30] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3481)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_195.init = 16'h40f4;
    LUT4 i9804_2_lut (.A(n9669), .B(n9668), .Z(spi_extension_length[7])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9804_2_lut.init = 16'h8888;
    LUT4 i9758_2_lut (.A(n9677), .B(n9676), .Z(spi_extension_length[9])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9758_2_lut.init = 16'h8888;
    LUT4 i9648_2_lut (.A(n9461), .B(n9460), .Z(spi_command[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9648_2_lut.init = 16'h8888;
    LUT4 i9647_2_lut (.A(n9465), .B(n9464), .Z(spi_command[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9647_2_lut.init = 16'h8888;
    LUT4 i9646_2_lut (.A(n9469), .B(n9468), .Z(spi_command[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9646_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_196 (.A(n26_adj_3508), .B(phase_active[82]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_82)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_196.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1959_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[14]), 
         .Z(spi_expected_length_31__N_1572)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1959_2_lut.init = 16'h4444;
    LUT4 i15740_2_lut_4_lut (.A(rgb_values[14]), .B(rgb_values[6]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_871)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15740_2_lut_4_lut.init = 16'h0035;
    LUT4 i15605_2_lut_4_lut (.A(rgb_values[59]), .B(rgb_values[51]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_736)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15605_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1573_2_lut_4_lut (.A(rgb_values[59]), .B(rgb_values[51]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_411)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1573_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1966_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[7]), 
         .Z(spi_expected_length_31__N_1586)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1966_2_lut.init = 16'h4444;
    LUT4 i5903_3_lut (.A(n10114), .B(n10113), .C(spi1_sck_c_enable_4), 
         .Z(rgb_values[22])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5903_3_lut.init = 16'hcaca;
    LUT4 i5935_3_lut (.A(n10146), .B(n10145), .C(spi1_sck_c_enable_187), 
         .Z(rgb_values[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5935_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1617_2_lut_4_lut (.A(rgb_values[15]), .B(rgb_values[7]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_499)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1617_2_lut_4_lut.init = 16'h00ca;
    LUT4 i1_4_lut_adj_197 (.A(status_flags_wire_15__N_2050[4]), .B(us_tx_83__N_2854), 
         .C(phase_acc[31]), .D(phase_active[4]), .Z(us_tx_c_4)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(150[33] 152[44])
    defparam i1_4_lut_adj_197.init = 16'h0880;
    LUT4 i1_4_lut_adj_198 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3468), 
         .C(phase_active[25]), .D(phase_acc[31]), .Z(us_tx_c_25)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_198.init = 16'h0880;
    LUT4 i5899_3_lut (.A(n10110), .B(n10109), .C(spi1_sck_c_enable_5), 
         .Z(rgb_values[23])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5899_3_lut.init = 16'hcaca;
    LUT4 amplitude_phase_7__I_0_2147_i4_4_lut (.A(\level_active[4] [0]), .B(\level_active[4] [1]), 
         .C(amplitude_phase[7]), .D(amplitude_phase[6]), .Z(us_tx_83__N_2854)) /* synthesis lut_function=(!(A (B (C (D))+!B (C+(D)))+!A ((C)+!B))) */ ;   // src/umh_fpga_top.v(151[33:79])
    defparam amplitude_phase_7__I_0_2147_i4_4_lut.init = 16'h0c8e;
    LUT4 i5931_3_lut (.A(n10142), .B(n10141), .C(spi1_sck_c_enable_188), 
         .Z(rgb_values[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5931_3_lut.init = 16'hcaca;
    LUT4 i5895_3_lut (.A(n10106), .B(n10105), .C(spi1_sck_c_enable_6), 
         .Z(rgb_values[24])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5895_3_lut.init = 16'hcaca;
    LUT4 i5927_3_lut (.A(n10138), .B(n10137), .C(spi1_sck_c_enable_189), 
         .Z(rgb_values[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5927_3_lut.init = 16'hcaca;
    LUT4 i5891_3_lut (.A(n10102), .B(n10101), .C(spi1_sck_c_enable_7), 
         .Z(rgb_values[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5891_3_lut.init = 16'hcaca;
    LUT4 i15401_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[14]), 
         .Z(spi_expected_length_31__N_1685)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15401_2_lut.init = 16'h1111;
    LUT4 i5923_3_lut (.A(n10134), .B(n10133), .C(spi1_sck_c_enable_190), 
         .Z(rgb_values[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5923_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1958_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[15]), 
         .Z(spi_expected_length_31__N_1570)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1958_2_lut.init = 16'h4444;
    LUT4 n18752_bdd_4_lut_16211 (.A(n7_adj_3395), .B(n128), .C(spi_byte_count[5]), 
         .D(n20783), .Z(n20784)) /* synthesis lut_function=(!(A+!(B (C (D))))) */ ;
    defparam n18752_bdd_4_lut_16211.init = 16'h4000;
    LUT4 i5887_3_lut (.A(n10098), .B(n10097), .C(spi1_sck_c_enable_8), 
         .Z(rgb_values[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5887_3_lut.init = 16'hcaca;
    LUT4 i5919_3_lut (.A(n10130), .B(n10129), .C(spi1_sck_c_enable_192), 
         .Z(rgb_values[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5919_3_lut.init = 16'hcaca;
    LUT4 i5883_3_lut (.A(n10094), .B(n10093), .C(spi1_sck_c_enable_9), 
         .Z(rgb_values[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5883_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1786_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_command[3]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1066)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1786_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5879_3_lut (.A(n10090), .B(n10089), .C(spi1_sck_c_enable_10), 
         .Z(rgb_values[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5879_3_lut.init = 16'hcaca;
    LUT4 i5875_3_lut (.A(n10086), .B(n10085), .C(spi1_sck_c_enable_11), 
         .Z(rgb_values[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5875_3_lut.init = 16'hcaca;
    LUT4 i5871_3_lut (.A(n10082), .B(n10081), .C(spi1_sck_c_enable_12), 
         .Z(rgb_values[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5871_3_lut.init = 16'hcaca;
    LUT4 i5867_3_lut (.A(n10078), .B(n10077), .C(spi1_sck_c_enable_13), 
         .Z(rgb_values[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5867_3_lut.init = 16'hcaca;
    LUT4 i5863_3_lut (.A(n10074), .B(n10073), .C(spi1_sck_c_enable_14), 
         .Z(rgb_values[32])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5863_3_lut.init = 16'hcaca;
    LUT4 i5859_3_lut (.A(n10070), .B(n10069), .C(spi1_sck_c_enable_15), 
         .Z(rgb_values[33])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5859_3_lut.init = 16'hcaca;
    LUT4 i15398_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[15]), 
         .Z(spi_expected_length_31__N_1682)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15398_2_lut.init = 16'h1111;
    LUT4 i2_4_lut_adj_199 (.A(n26_adj_3473), .B(phase_active[26]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_26)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_199.init = 16'h2080;
    LUT4 i15163_2_lut_4_lut (.A(spi_rx_shift[2]), .B(spi_command[3]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1094)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15163_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1787_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_command[2]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1068)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1787_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15737_2_lut_4_lut (.A(rgb_values[15]), .B(rgb_values[7]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_868)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15737_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_4_lut_adj_200 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3472), 
         .C(phase_active[27]), .D(phase_acc[31]), .Z(us_tx_c_27)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_200.init = 16'h0880;
    LUT4 i9128_2_lut (.A(level_mem_dout[6]), .B(level_mem_dout[8]), .Z(n2316)) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(306[37:83])
    defparam i9128_2_lut.init = 16'h8888;
    LUT4 i5855_3_lut (.A(n10066), .B(n10065), .C(spi1_sck_c_enable_16), 
         .Z(rgb_values[34])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5855_3_lut.init = 16'hcaca;
    LUT4 i30_3_lut_4_lut_adj_201 (.A(amplitude_phase[6]), .B(\level_active[34] [0]), 
         .C(\level_active[34] [1]), .D(amplitude_phase[7]), .Z(n26_adj_3489)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_201.init = 16'h40f4;
    LUT4 i9872_2_lut (.A(n9553), .B(n9552), .Z(spi_frame_sequence[10])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9872_2_lut.init = 16'h8888;
    LUT4 i15608_2_lut_4_lut (.A(rgb_values[58]), .B(rgb_values[50]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_739)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15608_2_lut_4_lut.init = 16'h0035;
    LUT4 n18752_bdd_4_lut_16210 (.A(spi_byte_count[8]), .B(spi_byte_count[6]), 
         .C(spi_update_flags[0]), .D(spi_byte_count[4]), .Z(n20783)) /* synthesis lut_function=(A (B (C (D)))+!A !(B+(C+(D)))) */ ;
    defparam n18752_bdd_4_lut_16210.init = 16'h8001;
    LUT4 i9779_2_lut (.A(n9365), .B(n9364), .Z(spi_expected_length[16])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9779_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1574_2_lut_4_lut (.A(rgb_values[58]), .B(rgb_values[50]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_413)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1574_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5851_3_lut (.A(n10062), .B(n10061), .C(spi1_sck_c_enable_17), 
         .Z(rgb_values[35])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5851_3_lut.init = 16'hcaca;
    LUT4 i5847_3_lut (.A(n10058), .B(n10057), .C(spi1_sck_c_enable_18), 
         .Z(rgb_values[36])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5847_3_lut.init = 16'hcaca;
    LUT4 i5843_3_lut (.A(n10054), .B(n10053), .C(spi1_sck_c_enable_19), 
         .Z(rgb_values[37])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5843_3_lut.init = 16'hcaca;
    LUT4 i5839_3_lut (.A(n10050), .B(n10049), .C(spi1_sck_c_enable_20), 
         .Z(rgb_values[38])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5839_3_lut.init = 16'hcaca;
    LUT4 i5835_3_lut (.A(n10046), .B(n10045), .C(spi1_sck_c_enable_21), 
         .Z(rgb_values[39])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5835_3_lut.init = 16'hcaca;
    LUT4 i9778_2_lut (.A(n9369), .B(n9368), .Z(spi_expected_length[17])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9778_2_lut.init = 16'h8888;
    LUT4 i5831_3_lut (.A(n10042), .B(n10041), .C(spi1_sck_c_enable_22), 
         .Z(rgb_values[40])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5831_3_lut.init = 16'hcaca;
    LUT4 i9777_2_lut (.A(n9373), .B(n9372), .Z(spi_expected_length[18])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9777_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_202 (.A(n26_adj_3390), .B(phase_active[40]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_40)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_202.init = 16'h2080;
    LUT4 n18752_bdd_4_lut_16339 (.A(n18524), .B(n8870), .C(spi_byte_count[7]), 
         .D(n20784), .Z(n20785)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;
    defparam n18752_bdd_4_lut_16339.init = 16'h0100;
    LUT4 i5827_3_lut (.A(n10038), .B(n10037), .C(spi1_sck_c_enable_23), 
         .Z(rgb_values[41])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5827_3_lut.init = 16'hcaca;
    LUT4 i15166_2_lut_4_lut (.A(spi_rx_shift[1]), .B(spi_command[2]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1097)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15166_2_lut_4_lut.init = 16'h0035;
    LUT4 i5823_3_lut (.A(n10034), .B(n10033), .C(spi1_sck_c_enable_24), 
         .Z(rgb_values[42])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5823_3_lut.init = 16'hcaca;
    LUT4 i2_4_lut_adj_203 (.A(n26_adj_3477), .B(phase_active[28]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_28)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_203.init = 16'h2080;
    LUT4 i5819_3_lut (.A(n10030), .B(n10029), .C(spi1_sck_c_enable_25), 
         .Z(rgb_values[43])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5819_3_lut.init = 16'hcaca;
    LUT4 n18752_bdd_3_lut_16342 (.A(n18752), .B(spi_update_flags[1]), .C(n20785), 
         .Z(n6931)) /* synthesis lut_function=(!(A+(B+!(C)))) */ ;
    defparam n18752_bdd_3_lut_16342.init = 16'h1010;
    LUT4 fpga_cs_n_N_339_I_0_1616_2_lut_4_lut (.A(rgb_values[16]), .B(rgb_values[8]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_497)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1616_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5815_3_lut (.A(n10026), .B(n10025), .C(spi1_sck_c_enable_26), 
         .Z(rgb_values[44])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5815_3_lut.init = 16'hcaca;
    LUT4 i5811_3_lut (.A(n10022), .B(n10021), .C(spi1_sck_c_enable_27), 
         .Z(rgb_values[45])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5811_3_lut.init = 16'hcaca;
    LUT4 i5807_3_lut (.A(n10018), .B(n10017), .C(spi1_sck_c_enable_28), 
         .Z(rgb_values[46])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5807_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_204 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3476), 
         .C(phase_active[29]), .D(phase_acc[31]), .Z(us_tx_c_29)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_204.init = 16'h0880;
    LUT4 i15734_2_lut_4_lut (.A(rgb_values[16]), .B(rgb_values[8]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_865)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15734_2_lut_4_lut.init = 16'h0035;
    LUT4 i5803_3_lut (.A(n10014), .B(n10013), .C(spi1_sck_c_enable_29), 
         .Z(rgb_values[47])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5803_3_lut.init = 16'hcaca;
    LUT4 i15611_2_lut_4_lut (.A(rgb_values[57]), .B(rgb_values[49]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_742)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15611_2_lut_4_lut.init = 16'h0035;
    LUT4 i2_4_lut_adj_205 (.A(n26_adj_3481), .B(phase_active[30]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_30)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_205.init = 16'h2080;
    LUT4 fpga_cs_n_N_339_I_0_1575_2_lut_4_lut (.A(rgb_values[57]), .B(rgb_values[49]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_415)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1575_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15614_2_lut_4_lut (.A(rgb_values[56]), .B(rgb_values[48]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_745)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15614_2_lut_4_lut.init = 16'h0035;
    LUT4 i1_4_lut_adj_206 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3480), 
         .C(phase_active[31]), .D(phase_acc[31]), .Z(us_tx_c_31)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_206.init = 16'h0880;
    LUT4 i2_4_lut_adj_207 (.A(n26_adj_3485), .B(phase_active[32]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_32)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_207.init = 16'h2080;
    LUT4 i1_4_lut_adj_208 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3484), 
         .C(phase_active[33]), .D(phase_acc[31]), .Z(us_tx_c_33)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_208.init = 16'h0880;
    LUT4 fpga_cs_n_N_339_I_0_1576_2_lut_4_lut (.A(rgb_values[56]), .B(rgb_values[48]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_417)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1576_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5799_3_lut (.A(n10010), .B(n10009), .C(spi1_sck_c_enable_30), 
         .Z(rgb_values[48])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5799_3_lut.init = 16'hcaca;
    LUT4 i9776_2_lut (.A(n9377), .B(n9376), .Z(spi_expected_length[19])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9776_2_lut.init = 16'h8888;
    LUT4 i15617_2_lut_4_lut (.A(rgb_values[55]), .B(rgb_values[47]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_748)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15617_2_lut_4_lut.init = 16'h0035;
    LUT4 i9818_2_lut (.A(n9565), .B(n9564), .Z(spi_frame_sequence[13])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9818_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1788_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_command[1]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1070)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1788_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1577_2_lut_4_lut (.A(rgb_values[55]), .B(rgb_values[47]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_419)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1577_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15620_2_lut_4_lut (.A(rgb_values[54]), .B(rgb_values[46]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_751)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15620_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1578_2_lut_4_lut (.A(rgb_values[54]), .B(rgb_values[46]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_421)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1578_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1615_2_lut_4_lut (.A(rgb_values[17]), .B(rgb_values[9]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_495)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1615_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15731_2_lut_4_lut (.A(rgb_values[17]), .B(rgb_values[9]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_862)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15731_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1804_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[1]), 
         .Z(spi_version_7__N_1118)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1804_2_lut.init = 16'h4444;
    LUT4 i5795_3_lut (.A(n10006), .B(n10005), .C(spi1_sck_c_enable_31), 
         .Z(rgb_values[49])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5795_3_lut.init = 16'hcaca;
    LUT4 i15623_2_lut_4_lut (.A(rgb_values[53]), .B(rgb_values[45]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_754)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15623_2_lut_4_lut.init = 16'h0035;
    LUT4 i15193_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[1]), .Z(spi_version_7__N_1148)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15193_2_lut.init = 16'h1111;
    LUT4 i9775_2_lut (.A(n9381), .B(n9380), .Z(spi_expected_length[20])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9775_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1803_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[2]), 
         .Z(spi_version_7__N_1116)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1803_2_lut.init = 16'h4444;
    LUT4 i15190_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[2]), .Z(spi_version_7__N_1145)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15190_2_lut.init = 16'h1111;
    LUT4 fpga_cs_n_N_339_I_0_1579_2_lut_4_lut (.A(rgb_values[53]), .B(rgb_values[45]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_423)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1579_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5791_3_lut (.A(n10002), .B(n10001), .C(spi1_sck_c_enable_32), 
         .Z(rgb_values[50])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5791_3_lut.init = 16'hcaca;
    LUT4 i5787_3_lut (.A(n9998), .B(n9997), .C(spi1_sck_c_enable_33), 
         .Z(rgb_values[51])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5787_3_lut.init = 16'hcaca;
    LUT4 i15626_2_lut_4_lut (.A(rgb_values[52]), .B(rgb_values[44]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_757)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15626_2_lut_4_lut.init = 16'h0035;
    LUT4 i5783_3_lut (.A(n9994), .B(n9993), .C(spi1_sck_c_enable_34), 
         .Z(rgb_values[52])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5783_3_lut.init = 16'hcaca;
    LUT4 i5779_3_lut (.A(n9990), .B(n9989), .C(spi1_sck_c_enable_35), 
         .Z(rgb_values[53])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5779_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1580_2_lut_4_lut (.A(rgb_values[52]), .B(rgb_values[44]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_425)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1580_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1802_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[3]), 
         .Z(spi_version_7__N_1114)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1802_2_lut.init = 16'h4444;
    LUT4 i15629_2_lut_4_lut (.A(rgb_values[51]), .B(rgb_values[43]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_760)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15629_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1581_2_lut_4_lut (.A(rgb_values[51]), .B(rgb_values[43]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_427)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1581_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15632_2_lut_4_lut (.A(rgb_values[50]), .B(rgb_values[42]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_763)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15632_2_lut_4_lut.init = 16'h0035;
    LUT4 i5775_3_lut (.A(n9986), .B(n9985), .C(spi1_sck_c_enable_36), 
         .Z(rgb_values[54])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5775_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1582_2_lut_4_lut (.A(rgb_values[50]), .B(rgb_values[42]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_429)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1582_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5771_3_lut (.A(n9982), .B(n9981), .C(spi1_sck_c_enable_37), 
         .Z(rgb_values[55])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5771_3_lut.init = 16'hcaca;
    LUT4 i5767_3_lut (.A(n9978), .B(n9977), .C(spi1_sck_c_enable_38), 
         .Z(rgb_values[56])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5767_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1782_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_command[7]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1058)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1782_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15187_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[3]), .Z(spi_version_7__N_1142)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15187_2_lut.init = 16'h1111;
    LUT4 i5763_3_lut (.A(n9974), .B(n9973), .C(spi1_sck_c_enable_39), 
         .Z(rgb_values[57])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5763_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_209 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3376), 
         .C(phase_active[41]), .D(phase_acc[31]), .Z(us_tx_c_41)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_209.init = 16'h0880;
    LUT4 i2_4_lut_adj_210 (.A(n26_adj_3489), .B(phase_active[34]), .C(status_flags_wire_15__N_2050[4]), 
         .D(phase_acc[31]), .Z(us_tx_c_34)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i2_4_lut_adj_210.init = 16'h2080;
    LUT4 i15151_2_lut_4_lut (.A(spi_rx_shift[6]), .B(spi_command[7]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1074)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15151_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1614_2_lut_4_lut (.A(rgb_values[18]), .B(rgb_values[10]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_493)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1614_2_lut_4_lut.init = 16'h00ca;
    LUT4 i5759_3_lut (.A(n9970), .B(n9969), .C(spi1_sck_c_enable_40), 
         .Z(rgb_values[58])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5759_3_lut.init = 16'hcaca;
    LUT4 i9774_2_lut (.A(n9385), .B(n9384), .Z(spi_expected_length[21])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9774_2_lut.init = 16'h8888;
    LUT4 i14942_4_lut (.A(status_bit_index[3]), .B(status_bit_index[0]), 
         .C(n18641), .D(n15), .Z(n18888)) /* synthesis lut_function=(!(A+!(B (C+(D))))) */ ;   // src/umh_fpga_top.v(158[13:23])
    defparam i14942_4_lut.init = 16'h4440;
    LUT4 i15728_2_lut_4_lut (.A(rgb_values[18]), .B(rgb_values[10]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_859)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15728_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1801_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[4]), 
         .Z(spi_version_7__N_1112)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1801_2_lut.init = 16'h4444;
    LUT4 i15184_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[4]), .Z(spi_version_7__N_1139)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15184_2_lut.init = 16'h1111;
    LUT4 i30_3_lut_4_lut_adj_211 (.A(amplitude_phase[6]), .B(\level_active[33] [0]), 
         .C(\level_active[33] [1]), .D(amplitude_phase[7]), .Z(n25_adj_3484)) /* synthesis lut_function=(!(A ((D)+!C)+!A !(B (C+!(D))+!B !((D)+!C)))) */ ;   // src/umh_fpga_top.v(66[11:26])
    defparam i30_3_lut_4_lut_adj_211.init = 16'h40f4;
    LUT4 fpga_cs_n_N_339_I_0_1783_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_command[6]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1060)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1783_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1800_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[5]), 
         .Z(spi_version_7__N_1110)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1800_2_lut.init = 16'h4444;
    LUT4 i15181_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[5]), .Z(spi_version_7__N_1136)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15181_2_lut.init = 16'h1111;
    LUT4 i1_4_lut_adj_212 (.A(status_flags_wire_15__N_2050[4]), .B(n25_adj_3488), 
         .C(phase_active[35]), .D(phase_acc[31]), .Z(us_tx_c_35)) /* synthesis lut_function=(!(((C (D)+!C !(D))+!B)+!A)) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i1_4_lut_adj_212.init = 16'h0880;
    LUT4 fpga_cs_n_N_339_I_0_1799_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[6]), 
         .Z(spi_version_7__N_1108)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1799_2_lut.init = 16'h4444;
    LUT4 i15154_2_lut_4_lut (.A(spi_rx_shift[5]), .B(spi_command[6]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1085)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15154_2_lut_4_lut.init = 16'h0035;
    LUT4 i15635_2_lut_4_lut (.A(rgb_values[49]), .B(rgb_values[41]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_766)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15635_2_lut_4_lut.init = 16'h0035;
    LUT4 i15178_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[6]), .Z(spi_version_7__N_1133)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15178_2_lut.init = 16'h1111;
    LUT4 i15169_2_lut_4_lut (.A(spi_rx_shift[0]), .B(spi_command[1]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1100)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15169_2_lut_4_lut.init = 16'h0035;
    LUT4 fpga_cs_n_N_339_I_0_1583_2_lut_4_lut (.A(rgb_values[49]), .B(rgb_values[41]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_431)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1583_2_lut_4_lut.init = 16'h00ca;
    LUT4 fpga_cs_n_N_339_I_0_1798_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[7]), 
         .Z(spi_version_7__N_1106)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1798_2_lut.init = 16'h4444;
    LUT4 i15175_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[7]), .Z(spi_version_7__N_1122)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15175_2_lut.init = 16'h1111;
    LUT4 i15638_2_lut_4_lut (.A(rgb_values[48]), .B(rgb_values[40]), .C(n6931), 
         .D(fpga_cs_n_c), .Z(rgb_values_95__N_769)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam i15638_2_lut_4_lut.init = 16'h0035;
    LUT4 i5755_3_lut (.A(n9966), .B(n9965), .C(spi1_sck_c_enable_41), 
         .Z(rgb_values[59])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5755_3_lut.init = 16'hcaca;
    LUT4 i5751_3_lut (.A(n9962), .B(n9961), .C(spi1_sck_c_enable_42), 
         .Z(rgb_values[60])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5751_3_lut.init = 16'hcaca;
    LUT4 i5747_3_lut (.A(n9958), .B(n9957), .C(spi1_sck_c_enable_43), 
         .Z(rgb_values[61])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5747_3_lut.init = 16'hcaca;
    LUT4 i9634_2_lut (.A(n9517), .B(n9516), .Z(spi_frame_sequence[1])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9634_2_lut.init = 16'h8888;
    LUT4 i5743_3_lut (.A(n9954), .B(n9953), .C(spi1_sck_c_enable_44), 
         .Z(rgb_values[62])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5743_3_lut.init = 16'hcaca;
    LUT4 i5739_3_lut (.A(n9950), .B(n9949), .C(spi1_sck_c_enable_45), 
         .Z(rgb_values[63])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5739_3_lut.init = 16'hcaca;
    LUT4 i5735_3_lut (.A(n9946), .B(n9945), .C(spi1_sck_c_enable_46), 
         .Z(rgb_values[64])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5735_3_lut.init = 16'hcaca;
    LUT4 i1_4_lut_adj_213 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[36]), 
         .C(n26_adj_3493), .D(phase_acc[31]), .Z(us_tx_c_36)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_213.init = 16'h2080;
    LUT4 i5731_3_lut (.A(n9942), .B(n9941), .C(spi1_sck_c_enable_47), 
         .Z(rgb_values[65])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5731_3_lut.init = 16'hcaca;
    LUT4 i5727_3_lut (.A(n9938), .B(n9937), .C(spi1_sck_c_enable_48), 
         .Z(rgb_values[66])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5727_3_lut.init = 16'hcaca;
    LUT4 i9633_2_lut (.A(n9521), .B(n9520), .Z(spi_frame_sequence[2])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9633_2_lut.init = 16'h8888;
    LUT4 i5723_3_lut (.A(n9934), .B(n9933), .C(spi1_sck_c_enable_49), 
         .Z(rgb_values[67])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5723_3_lut.init = 16'hcaca;
    LUT4 i5719_3_lut (.A(n9930), .B(n9929), .C(spi1_sck_c_enable_50), 
         .Z(rgb_values[68])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5719_3_lut.init = 16'hcaca;
    LUT4 i9632_2_lut (.A(n9525), .B(n9524), .Z(spi_frame_sequence[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9632_2_lut.init = 16'h8888;
    LUT4 i5715_3_lut (.A(n9926), .B(n9925), .C(spi1_sck_c_enable_51), 
         .Z(rgb_values[69])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5715_3_lut.init = 16'hcaca;
    LUT4 i5711_3_lut (.A(n9922), .B(n9921), .C(spi1_sck_c_enable_52), 
         .Z(rgb_values[70])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5711_3_lut.init = 16'hcaca;
    LUT4 i9889_2_lut (.A(n9389), .B(n9388), .Z(spi_expected_length[22])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9889_2_lut.init = 16'h8888;
    LUT4 i5707_3_lut (.A(n9918), .B(n9917), .C(spi1_sck_c_enable_53), 
         .Z(rgb_values[71])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5707_3_lut.init = 16'hcaca;
    LUT4 i5703_3_lut (.A(n9914), .B(n9913), .C(spi1_sck_c_enable_54), 
         .Z(rgb_values[72])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5703_3_lut.init = 16'hcaca;
    LUT4 i5699_3_lut (.A(n9910), .B(n9909), .C(spi1_sck_c_enable_55), 
         .Z(rgb_values[73])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5699_3_lut.init = 16'hcaca;
    LUT4 i5695_3_lut (.A(n9906), .B(n9905), .C(spi1_sck_c_enable_56), 
         .Z(rgb_values[74])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5695_3_lut.init = 16'hcaca;
    LUT4 i5691_3_lut (.A(n9902), .B(n9901), .C(spi1_sck_c_enable_57), 
         .Z(rgb_values[75])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5691_3_lut.init = 16'hcaca;
    LUT4 i5687_3_lut (.A(n9898), .B(n9897), .C(spi1_sck_c_enable_58), 
         .Z(rgb_values[76])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5687_3_lut.init = 16'hcaca;
    LUT4 i5683_3_lut (.A(n9894), .B(n9893), .C(spi1_sck_c_enable_59), 
         .Z(rgb_values[77])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5683_3_lut.init = 16'hcaca;
    LUT4 i2_3_lut_adj_214 (.A(load_active), .B(fifo_depth[0]), .C(carrier_wrap), 
         .Z(running_N_2903)) /* synthesis lut_function=(!(A+!(B (C)))) */ ;   // src/umh_fpga_top.v(313[5] 320[8])
    defparam i2_3_lut_adj_214.init = 16'h4040;
    LUT4 phase_acc_next_31__I_0_i64_3_lut (.A(n62), .B(phase_acc[31]), .C(phase_acc_next[31]), 
         .Z(carrier_wrap)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i64_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i62_3_lut (.A(n60), .B(phase_acc[30]), .C(phase_acc_next[30]), 
         .Z(n62)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i62_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i60_3_lut (.A(n58), .B(phase_acc[29]), .C(phase_acc_next[29]), 
         .Z(n60)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i60_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i58_3_lut (.A(n56), .B(phase_acc[28]), .C(phase_acc_next[28]), 
         .Z(n58)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i58_3_lut.init = 16'h8e8e;
    LUT4 i9631_2_lut (.A(n9529), .B(n9528), .Z(spi_frame_sequence[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9631_2_lut.init = 16'h8888;
    LUT4 phase_acc_next_31__I_0_i56_3_lut (.A(n54), .B(phase_acc[27]), .C(phase_acc_next[27]), 
         .Z(n56)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i56_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i54_3_lut (.A(n52), .B(phase_acc[26]), .C(phase_acc_next[26]), 
         .Z(n54)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i54_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i52_3_lut (.A(n50), .B(phase_acc[25]), .C(phase_acc_next[25]), 
         .Z(n52)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i52_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i50_3_lut (.A(n48), .B(phase_acc[24]), .C(phase_acc_next[24]), 
         .Z(n50)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i50_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i48_3_lut (.A(n46), .B(phase_acc[23]), .C(phase_acc_next[23]), 
         .Z(n48)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i48_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i46_3_lut (.A(n44_adj_3426), .B(phase_acc[22]), 
         .C(phase_acc_next[22]), .Z(n46)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i46_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i44_3_lut (.A(n42), .B(phase_acc[21]), .C(phase_acc_next[21]), 
         .Z(n44_adj_3426)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i44_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i42_3_lut (.A(n40_adj_3381), .B(phase_acc[20]), 
         .C(phase_acc_next[20]), .Z(n42)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i42_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i40_3_lut (.A(n38), .B(phase_acc[19]), .C(phase_acc_next[19]), 
         .Z(n40_adj_3381)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i40_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i38_3_lut (.A(n36), .B(phase_acc[18]), .C(phase_acc_next[18]), 
         .Z(n38)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i38_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i36_3_lut (.A(n34_adj_3415), .B(phase_acc[17]), 
         .C(phase_acc_next[17]), .Z(n36)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i36_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i34_3_lut (.A(n32), .B(phase_acc[16]), .C(phase_acc_next[16]), 
         .Z(n34_adj_3415)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i34_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i32_3_lut (.A(n30_adj_3414), .B(phase_acc[15]), 
         .C(phase_acc_next[15]), .Z(n32)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i32_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i30_3_lut (.A(n28), .B(phase_acc[14]), .C(phase_acc_next[14]), 
         .Z(n30_adj_3414)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i30_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i28_3_lut (.A(n26_adj_3413), .B(phase_acc[13]), 
         .C(phase_acc_next[13]), .Z(n28)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i28_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i26_3_lut (.A(n24), .B(phase_acc[12]), .C(phase_acc_next[12]), 
         .Z(n26_adj_3413)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i26_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i24_3_lut (.A(n22), .B(phase_acc[11]), .C(phase_acc_next[11]), 
         .Z(n24)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i24_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i22_3_lut (.A(n20), .B(phase_acc[10]), .C(phase_acc_next[10]), 
         .Z(n22)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i22_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i20_3_lut (.A(n18), .B(phase_acc[9]), .C(phase_acc_next[9]), 
         .Z(n20)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i20_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i18_3_lut (.A(n16), .B(phase_acc[8]), .C(phase_acc_next[8]), 
         .Z(n18)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i18_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i16_3_lut (.A(n14), .B(phase_acc[7]), .C(phase_acc_next[7]), 
         .Z(n16)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i16_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i14_3_lut (.A(n12_adj_3374), .B(phase_acc[6]), 
         .C(phase_acc_next[6]), .Z(n14)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i14_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i12_3_lut (.A(n10_adj_3391), .B(phase_acc[5]), 
         .C(phase_acc_next[5]), .Z(n12_adj_3374)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i12_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i10_3_lut (.A(n8_adj_3380), .B(phase_acc[4]), 
         .C(phase_acc_next[4]), .Z(n10_adj_3391)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i10_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i8_3_lut (.A(n6), .B(phase_acc[3]), .C(phase_acc_next[3]), 
         .Z(n8_adj_3380)) /* synthesis lut_function=(A (B+!(C))+!A !((C)+!B)) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i8_3_lut.init = 16'h8e8e;
    LUT4 phase_acc_next_31__I_0_i6_4_lut (.A(phase_acc_next[1]), .B(phase_acc[2]), 
         .C(phase_acc_next[2]), .D(phase_acc[1]), .Z(n6)) /* synthesis lut_function=(!(A ((C)+!B)+!A !(B ((D)+!C)+!B !(C+!(D))))) */ ;   // src/umh_fpga_top.v(140[21:47])
    defparam phase_acc_next_31__I_0_i6_4_lut.init = 16'h4d0c;
    LUT4 i9630_2_lut (.A(n9533), .B(n9532), .Z(spi_frame_sequence[5])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9630_2_lut.init = 16'h8888;
    LUT4 i5679_3_lut (.A(n9890), .B(n9889), .C(spi1_sck_c_enable_60), 
         .Z(rgb_values[78])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5679_3_lut.init = 16'hcaca;
    LUT4 i5043_3_lut (.A(n9254), .B(n9253), .C(spi1_sck_c_enable_191), 
         .Z(rgb_values[0])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5043_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1967_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[6]), 
         .Z(spi_expected_length_31__N_1588)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1967_2_lut.init = 16'h4444;
    LUT4 i4_2_lut (.A(n7), .B(n8_adj_3389), .Z(n37_adj_3400)) /* synthesis lut_function=(A+(B)) */ ;   // src/umh_fpga_top.v(198[17:23])
    defparam i4_2_lut.init = 16'heeee;
    LUT4 i9860_2_lut (.A(n9317), .B(n9316), .Z(spi_expected_length[4])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9860_2_lut.init = 16'h8888;
    LUT4 i9857_2_lut (.A(n9322), .B(n9320), .Z(spi_expected_length[5])) /* synthesis lut_function=(A+(B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9857_2_lut.init = 16'heeee;
    LUT4 i2_3_lut_adj_215 (.A(n36_adj_3431), .B(spi_byte_count[5]), .C(n14028), 
         .Z(n8_adj_3389)) /* synthesis lut_function=(A+!(B (C))) */ ;   // src/umh_fpga_top.v(198[17:23])
    defparam i2_3_lut_adj_215.init = 16'hbfbf;
    LUT4 i3_4_lut_adj_216 (.A(time_divider[0]), .B(n18762), .C(time_divider[1]), 
         .D(time_divider_5__N_2270[1]), .Z(n8892)) /* synthesis lut_function=(A ((C+(D))+!B)+!A !(B (C (D)))) */ ;
    defparam i3_4_lut_adj_216.init = 16'hbff7;
    LUT4 i14643_2_lut (.A(time_divider[5]), .B(time_divider[3]), .Z(n18762)) /* synthesis lut_function=(A (B)) */ ;
    defparam i14643_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_217 (.A(phase_active[37]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3449), .Z(us_tx_c_37)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_217.init = 16'h4800;
    LUT4 i1_2_lut_adj_218 (.A(time_divider[4]), .B(time_divider[2]), .Z(n4_adj_3443)) /* synthesis lut_function=(A+(B)) */ ;   // src/umh_fpga_top.v(278[9:52])
    defparam i1_2_lut_adj_218.init = 16'heeee;
    LUT4 i15428_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[5]), 
         .Z(spi_expected_length_31__N_1712)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15428_2_lut.init = 16'h1111;
    LUT4 frame_toggle_sync_I_0_2_lut (.A(frame_toggle_sync), .B(frame_toggle_seen), 
         .Z(frame_toggle_seen_N_2889)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // src/umh_fpga_top.v(290[9:47])
    defparam frame_toggle_sync_I_0_2_lut.init = 16'h6666;
    LUT4 stop_toggle_sync_I_0_2_lut (.A(stop_toggle_sync), .B(stop_toggle_seen), 
         .Z(stop_toggle_seen_N_2891)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;   // src/umh_fpga_top.v(297[9:45])
    defparam stop_toggle_sync_I_0_2_lut.init = 16'h6666;
    LUT4 i3490_2_lut (.A(n13), .B(load_active), .Z(fpga_clk_c_enable_188)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(273[8] 335[4])
    defparam i3490_2_lut.init = 16'h4444;
    LUT4 i2_3_lut_adj_219 (.A(stop_toggle_seen_N_2891), .B(frame_toggle_seen_N_2889), 
         .C(running_N_2903), .Z(fpga_clk_c_enable_23)) /* synthesis lut_function=(A+(B+(C))) */ ;   // src/umh_fpga_top.v(297[5] 302[8])
    defparam i2_3_lut_adj_219.init = 16'hfefe;
    LUT4 i15988_2_lut (.A(mic_clk_c), .B(mic_divider_6__N_2705), .Z(fpga_clk_c_enable_274)) /* synthesis lut_function=(!(A+!(B))) */ ;
    defparam i15988_2_lut.init = 16'h4444;
    LUT4 i15986_4_lut (.A(mic_divider[3]), .B(n10_adj_3513), .C(mic_divider[2]), 
         .D(mic_divider[1]), .Z(mic_divider_6__N_2705)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;   // src/umh_fpga_top.v(333[24:42])
    defparam i15986_4_lut.init = 16'h1000;
    LUT4 i4_4_lut (.A(mic_divider[0]), .B(mic_divider[5]), .C(mic_divider[6]), 
         .D(mic_divider[4]), .Z(n10_adj_3513)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;
    defparam i4_4_lut.init = 16'hfffe;
    LUT4 i4_4_lut_adj_220 (.A(mic_sample_count[1]), .B(mic_sample_count[4]), 
         .C(mic_sample_count[3]), .D(mic_sample_count[2]), .Z(n10_adj_3512)) /* synthesis lut_function=(!((B+!(C (D)))+!A)) */ ;
    defparam i4_4_lut_adj_220.init = 16'h2000;
    LUT4 i34_2_lut (.A(mic_clk_c), .B(mic_divider_6__N_2705), .Z(mic_clk_N_2867)) /* synthesis lut_function=(!(A (B)+!A !(B))) */ ;
    defparam i34_2_lut.init = 16'h6666;
    LUT4 fpga_cs_n_N_339_I_0_1736_2_lut (.A(fpga_cs_n_c), .B(spi1_mosi_c), 
         .Z(spi_rx_shift_6__N_922)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1736_2_lut.init = 16'h4444;
    LUT4 i15148_2_lut (.A(fpga_cs_n_c), .B(spi1_mosi_c), .Z(spi_rx_shift_0__N_941)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15148_2_lut.init = 16'h1111;
    LUT4 i3_4_lut_adj_221 (.A(n36_adj_3431), .B(spi_byte_count[1]), .C(spi_byte_count[5]), 
         .D(n10), .Z(n9019)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // src/umh_fpga_top.v(190[17:22])
    defparam i3_4_lut_adj_221.init = 16'hfffe;
    LUT4 i9629_2_lut (.A(n9537), .B(n9536), .Z(spi_frame_sequence[6])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9629_2_lut.init = 16'h8888;
    LUT4 i2_4_lut_adj_222 (.A(phase_active[3]), .B(status_flags_wire_15__N_2050[4]), 
         .C(phase_acc[31]), .D(n25_adj_3509), .Z(us_tx_c_3)) /* synthesis lut_function=(!(A ((C+!(D))+!B)+!A !(B (C (D))))) */ ;   // src/umh_fpga_top.v(62[12:21])
    defparam i2_4_lut_adj_222.init = 16'h4800;
    LUT4 fpga_cs_n_N_339_I_0_1805_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[0]), 
         .Z(spi_version_7__N_1120)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1805_2_lut.init = 16'h4444;
    LUT4 i9902_2_lut (.A(n9569), .B(n9568), .Z(spi_frame_sequence[14])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9902_2_lut.init = 16'h8888;
    LUT4 i15196_2_lut (.A(fpga_cs_n_c), .B(spi_version_7__N_1107[0]), .Z(spi_version_7__N_1151)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15196_2_lut.init = 16'h1111;
    LUT4 i1_4_lut_adj_223 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[42]), 
         .C(n26), .D(phase_acc[31]), .Z(us_tx_c_42)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_223.init = 16'h2080;
    LUT4 i9772_2_lut (.A(n9393), .B(n9392), .Z(spi_expected_length[23])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9772_2_lut.init = 16'h8888;
    LUT4 i9901_2_lut (.A(n9573), .B(n9572), .Z(spi_frame_sequence[15])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9901_2_lut.init = 16'h8888;
    INV i16394 (.A(spi1_sck_c), .Z(spi1_sck_N_1872));   // src/umh_fpga_top.v(8[24:32])
    VLO i1 (.Z(GND_net));
    LUT4 i9736_2_lut (.A(n9273), .B(n9272), .Z(spi_frame_sequence[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9736_2_lut.init = 16'h8888;
    TSALL TSALL_INST (.TSALL(GND_net));
    LUT4 i9862_2_lut (.A(n9310), .B(n9308), .Z(spi_expected_length[2])) /* synthesis lut_function=(A+(B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9862_2_lut.init = 16'heeee;
    LUT4 i9861_2_lut (.A(n9313), .B(n9312), .Z(spi_expected_length[3])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9861_2_lut.init = 16'h8888;
    LUT4 i9628_2_lut (.A(n9541), .B(n9540), .Z(spi_frame_sequence[7])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9628_2_lut.init = 16'h8888;
    LUT4 fpga_cs_n_N_339_I_0_1584_2_lut_4_lut (.A(rgb_values[48]), .B(rgb_values[40]), 
         .C(n6931), .D(fpga_cs_n_c), .Z(rgb_values_95__N_433)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(225[30] 231[24])
    defparam fpga_cs_n_N_339_I_0_1584_2_lut_4_lut.init = 16'h00ca;
    LUT4 i3_4_lut_adj_224 (.A(spi_byte_count[0]), .B(spi_byte_count[5]), 
         .C(spi_byte_count[4]), .D(n8881), .Z(n7626)) /* synthesis lut_function=(A+(B+!(C (D)))) */ ;
    defparam i3_4_lut_adj_224.init = 16'hefff;
    LUT4 i3_4_lut_adj_225 (.A(n13987), .B(n13957), .C(rgb_values_95__N_630), 
         .D(n18634), .Z(n8881)) /* synthesis lut_function=(!(A+(B+!(C (D))))) */ ;
    defparam i3_4_lut_adj_225.init = 16'h1000;
    LUT4 i5675_3_lut (.A(n9886), .B(n9885), .C(spi1_sck_c_enable_61), 
         .Z(rgb_values[79])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5675_3_lut.init = 16'hcaca;
    LUT4 i5671_3_lut (.A(n9882), .B(n9881), .C(spi1_sck_c_enable_62), 
         .Z(rgb_values[80])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5671_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_1969_2_lut (.A(fpga_cs_n_c), .B(spi_expected_length_31__N_2085[4]), 
         .Z(spi_expected_length_31__N_1592)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_1969_2_lut.init = 16'h4444;
    LUT4 i1_4_lut_adj_226 (.A(status_flags_wire_15__N_2050[4]), .B(phase_active[38]), 
         .C(n26_adj_3496), .D(phase_acc[31]), .Z(us_tx_c_38)) /* synthesis lut_function=(!((B ((D)+!C)+!B !(C (D)))+!A)) */ ;
    defparam i1_4_lut_adj_226.init = 16'h2080;
    LUT4 i5667_3_lut (.A(n9878), .B(n9877), .C(spi1_sck_c_enable_63), 
         .Z(rgb_values[81])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i5667_3_lut.init = 16'hcaca;
    LUT4 fpga_cs_n_N_339_I_0_2012_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[0]), 
         .Z(spi_channel_index_6__N_1742)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2012_2_lut.init = 16'h4444;
    LUT4 fpga_cs_n_N_339_I_0_1784_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_command[5]), 
         .C(n37), .D(fpga_cs_n_c), .Z(spi_command_7__N_1062)) /* synthesis lut_function=(!(A (B (D)+!B (C+(D)))+!A (((D)+!C)+!B))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam fpga_cs_n_N_339_I_0_1784_2_lut_4_lut.init = 16'h00ca;
    LUT4 i15157_2_lut_4_lut (.A(spi_rx_shift[4]), .B(spi_command[5]), .C(n37), 
         .D(fpga_cs_n_c), .Z(spi_command_7__N_1088)) /* synthesis lut_function=(!(A (B+((D)+!C))+!A (B (C+(D))+!B (D)))) */ ;   // src/umh_fpga_top.v(188[13] 233[20])
    defparam i15157_2_lut_4_lut.init = 16'h0035;
    LUT4 i15464_2_lut (.A(fpga_cs_n_c), .B(spi_channel_index_6__N_1731[0]), 
         .Z(spi_channel_index_6__N_1769)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15464_2_lut.init = 16'h1111;
    LUT4 fpga_cs_n_N_339_I_0_2031_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[0]), 
         .Z(spi_level_pending_7__N_1798)) /* synthesis lut_function=(!(A+!(B))) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam fpga_cs_n_N_339_I_0_2031_2_lut.init = 16'h4444;
    LUT4 i15488_2_lut (.A(fpga_cs_n_c), .B(spi_level_pending_7__N_1785[0]), 
         .Z(spi_level_pending_7__N_1829)) /* synthesis lut_function=(!(A+(B))) */ ;
    defparam i15488_2_lut.init = 16'h1111;
    LUT4 i9877_2_lut (.A(n9289), .B(n9288), .Z(spi_level_pending[0])) /* synthesis lut_function=(A (B)) */ ;   // src/umh_fpga_top.v(239[14] 252[8])
    defparam i9877_2_lut.init = 16'h8888;
    ws2812_stream ws2812_i (.reset_count({Open_0, Open_1, Open_2, Open_3, 
            Open_4, Open_5, Open_6, reset_count[5:4], Open_7, reset_count[2:0]}), 
            .n17441(n17441), .fpga_clk_c(fpga_clk_c), .fpga_clk_c_enable_215(fpga_clk_c_enable_215), 
            .GND_net(GND_net), .rgb_values({rgb_values}), .rgb_data_c(rgb_data_c)) /* synthesis syn_module_defined=1 */ ;   // src/umh_fpga_top.v(339[15] 346[2])
    PFUMX i16259 (.BLUT(n20866), .ALUT(n20865), .C0(spi_byte_count[5]), 
          .Z(n20867));
    PUR PUR_INST (.PUR(VCC_net));
    defparam PUR_INST.RST_PULSE = 1;
    
endmodule
//
// Verilog Description of module spi_mic_stream
//

module spi_mic_stream (GND_net, sck_N_3318, spi_mic_cs_n_c, mic_latest, 
            spi_mic_miso_c) /* synthesis syn_module_defined=1 */ ;
    input GND_net;
    input sck_N_3318;
    input spi_mic_cs_n_c;
    input [31:0]mic_latest;
    output spi_mic_miso_c;
    
    wire sck_N_3318 /* synthesis is_inv_clock=1 */ ;   // src/spi_mic_stream.v(11[12:26])
    
    wire n17154;
    wire [5:0]bit_count;   // src/spi_mic_stream.v(12[11:20])
    wire [5:0]n9;
    
    wire n17153, n17152;
    wire [5:0]bit_count_5__N_3358;
    
    wire sck_N_3318_enable_32;
    wire [5:0]bit_count_5__N_3319;
    wire [31:0]shift_register;   // src/spi_mic_stream.v(11[12:26])
    
    wire n11_adj_3367;
    wire [31:0]shift_register_31__N_3285;
    
    wire sck_N_3318_enable_36, n10302, n10300, n10298, n10296, n10_adj_3368;
    
    CCU2D add_9_7 (.A0(bit_count[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), .CIN(n17154), 
          .S0(n9[5]));   // src/spi_mic_stream.v(30[22:38])
    defparam add_9_7.INIT0 = 16'h5aaa;
    defparam add_9_7.INIT1 = 16'h0000;
    defparam add_9_7.INJECT1_0 = "NO";
    defparam add_9_7.INJECT1_1 = "NO";
    CCU2D add_9_5 (.A0(bit_count[3]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[4]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17153), .COUT(n17154), .S0(n9[3]), .S1(n9[4]));   // src/spi_mic_stream.v(30[22:38])
    defparam add_9_5.INIT0 = 16'h5aaa;
    defparam add_9_5.INIT1 = 16'h5aaa;
    defparam add_9_5.INJECT1_0 = "NO";
    defparam add_9_5.INJECT1_1 = "NO";
    CCU2D add_9_3 (.A0(bit_count[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17152), .COUT(n17153), .S0(n9[1]), .S1(n9[2]));   // src/spi_mic_stream.v(30[22:38])
    defparam add_9_3.INIT0 = 16'h5aaa;
    defparam add_9_3.INIT1 = 16'h5aaa;
    defparam add_9_3.INJECT1_0 = "NO";
    defparam add_9_3.INJECT1_1 = "NO";
    CCU2D add_9_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n17152), .S1(bit_count_5__N_3358[0]));   // src/spi_mic_stream.v(30[22:38])
    defparam add_9_1.INIT0 = 16'hF000;
    defparam add_9_1.INIT1 = 16'h5555;
    defparam add_9_1.INJECT1_0 = "NO";
    defparam add_9_1.INJECT1_1 = "NO";
    FD1P3DX bit_count_i0 (.D(bit_count_5__N_3319[0]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(bit_count[0])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i0.GSR = "ENABLED";
    LUT4 i9624_4_lut (.A(mic_latest[31]), .B(spi_mic_cs_n_c), .C(shift_register[31]), 
         .D(n11_adj_3367), .Z(spi_mic_miso_c)) /* synthesis lut_function=(!(A (B+!(C+!(D)))+!A (B+!(C (D))))) */ ;   // src/spi_mic_stream.v(18[15] 19[73])
    defparam i9624_4_lut.init = 16'h3022;
    FD1S3DX bit_count_i5 (.D(bit_count_5__N_3319[5]), .CK(sck_N_3318), .CD(spi_mic_cs_n_c), 
            .Q(bit_count[5])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i5.GSR = "ENABLED";
    FD1P3DX shift_register_i1 (.D(shift_register_31__N_3285[1]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[1])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i1.GSR = "ENABLED";
    FD1P3DX shift_register_i2 (.D(shift_register_31__N_3285[2]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[2])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i2.GSR = "ENABLED";
    FD1P3DX shift_register_i3 (.D(shift_register_31__N_3285[3]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[3])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i3.GSR = "ENABLED";
    FD1P3DX shift_register_i4 (.D(shift_register_31__N_3285[4]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[4])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i4.GSR = "ENABLED";
    FD1P3DX shift_register_i5 (.D(shift_register_31__N_3285[5]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[5])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i5.GSR = "ENABLED";
    FD1P3DX shift_register_i6 (.D(shift_register_31__N_3285[6]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[6])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i6.GSR = "ENABLED";
    FD1P3DX shift_register_i7 (.D(shift_register_31__N_3285[7]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[7])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i7.GSR = "ENABLED";
    FD1P3DX shift_register_i8 (.D(shift_register_31__N_3285[8]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[8])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i8.GSR = "ENABLED";
    FD1P3DX shift_register_i9 (.D(shift_register_31__N_3285[9]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[9])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i9.GSR = "ENABLED";
    FD1P3DX shift_register_i10 (.D(shift_register_31__N_3285[10]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[10])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i10.GSR = "ENABLED";
    FD1P3DX shift_register_i11 (.D(shift_register_31__N_3285[11]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[11])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i11.GSR = "ENABLED";
    FD1P3DX shift_register_i12 (.D(shift_register_31__N_3285[12]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[12])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i12.GSR = "ENABLED";
    FD1P3DX shift_register_i13 (.D(shift_register_31__N_3285[13]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[13])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i13.GSR = "ENABLED";
    FD1P3DX shift_register_i14 (.D(shift_register_31__N_3285[14]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[14])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i14.GSR = "ENABLED";
    FD1P3DX shift_register_i15 (.D(shift_register_31__N_3285[15]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[15])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i15.GSR = "ENABLED";
    FD1P3DX shift_register_i16 (.D(shift_register_31__N_3285[16]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[16])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i16.GSR = "ENABLED";
    FD1P3DX shift_register_i17 (.D(shift_register_31__N_3285[17]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[17])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i17.GSR = "ENABLED";
    FD1P3DX shift_register_i18 (.D(shift_register_31__N_3285[18]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[18])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i18.GSR = "ENABLED";
    FD1P3DX shift_register_i19 (.D(shift_register_31__N_3285[19]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[19])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i19.GSR = "ENABLED";
    FD1P3DX shift_register_i20 (.D(shift_register_31__N_3285[20]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[20])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i20.GSR = "ENABLED";
    FD1P3DX shift_register_i21 (.D(shift_register_31__N_3285[21]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[21])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i21.GSR = "ENABLED";
    FD1P3DX shift_register_i22 (.D(shift_register_31__N_3285[22]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[22])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i22.GSR = "ENABLED";
    FD1P3DX shift_register_i23 (.D(shift_register_31__N_3285[23]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[23])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i23.GSR = "ENABLED";
    FD1P3DX shift_register_i24 (.D(shift_register_31__N_3285[24]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[24])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i24.GSR = "ENABLED";
    FD1P3DX shift_register_i25 (.D(shift_register_31__N_3285[25]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[25])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i25.GSR = "ENABLED";
    FD1P3DX shift_register_i26 (.D(shift_register_31__N_3285[26]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[26])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i26.GSR = "ENABLED";
    FD1P3DX shift_register_i27 (.D(shift_register_31__N_3285[27]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[27])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i27.GSR = "ENABLED";
    FD1P3DX shift_register_i28 (.D(shift_register_31__N_3285[28]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[28])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i28.GSR = "ENABLED";
    FD1P3DX shift_register_i29 (.D(shift_register_31__N_3285[29]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[29])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i29.GSR = "ENABLED";
    FD1P3DX shift_register_i30 (.D(shift_register_31__N_3285[30]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[30])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i30.GSR = "ENABLED";
    FD1P3DX shift_register_i31 (.D(shift_register_31__N_3285[31]), .SP(sck_N_3318_enable_32), 
            .CK(sck_N_3318), .CD(spi_mic_cs_n_c), .Q(shift_register[31])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam shift_register_i31.GSR = "ENABLED";
    FD1P3DX bit_count_i4 (.D(n10302), .SP(sck_N_3318_enable_36), .CK(sck_N_3318), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[4])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i4.GSR = "ENABLED";
    FD1P3DX bit_count_i3 (.D(n10300), .SP(sck_N_3318_enable_36), .CK(sck_N_3318), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[3])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i3.GSR = "ENABLED";
    FD1P3DX bit_count_i2 (.D(n10298), .SP(sck_N_3318_enable_36), .CK(sck_N_3318), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[2])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i2.GSR = "ENABLED";
    FD1P3DX bit_count_i1 (.D(n10296), .SP(sck_N_3318_enable_36), .CK(sck_N_3318), 
            .CD(spi_mic_cs_n_c), .Q(bit_count[1])) /* synthesis LSE_LINE_FILE_ID=6, LSE_LCOL=16, LSE_RCOL=2, LSE_LLINE=348, LSE_RLINE=351 */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam bit_count_i1.GSR = "ENABLED";
    LUT4 i9786_3_lut (.A(n9[5]), .B(n11_adj_3367), .C(bit_count[5]), .Z(bit_count_5__N_3319[5])) /* synthesis lut_function=(A (B)+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam i9786_3_lut.init = 16'hc8c8;
    LUT4 i9785_2_lut (.A(mic_latest[0]), .B(n11_adj_3367), .Z(shift_register_31__N_3285[1])) /* synthesis lut_function=(!((B)+!A)) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam i9785_2_lut.init = 16'h2222;
    LUT4 shift_register_31__I_0_19_i3_3_lut (.A(mic_latest[1]), .B(shift_register[1]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[2])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i3_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i4_3_lut (.A(mic_latest[2]), .B(shift_register[2]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[3])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i4_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i5_3_lut (.A(mic_latest[3]), .B(shift_register[3]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[4])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i5_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i6_3_lut (.A(mic_latest[4]), .B(shift_register[4]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[5])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i6_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i7_3_lut (.A(mic_latest[5]), .B(shift_register[5]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[6])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i7_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i8_3_lut (.A(mic_latest[6]), .B(shift_register[6]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[7])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i8_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i9_3_lut (.A(mic_latest[7]), .B(shift_register[7]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[8])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i9_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i10_3_lut (.A(mic_latest[8]), .B(shift_register[8]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[9])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i10_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i11_3_lut (.A(mic_latest[9]), .B(shift_register[9]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[10])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i11_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i12_3_lut (.A(mic_latest[10]), .B(shift_register[10]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[11])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i12_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i13_3_lut (.A(mic_latest[11]), .B(shift_register[11]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[12])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i13_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i14_3_lut (.A(mic_latest[12]), .B(shift_register[12]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[13])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i14_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i15_3_lut (.A(mic_latest[13]), .B(shift_register[13]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[14])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i15_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i16_3_lut (.A(mic_latest[14]), .B(shift_register[14]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[15])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i16_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i17_3_lut (.A(mic_latest[15]), .B(shift_register[15]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[16])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i17_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i18_3_lut (.A(mic_latest[16]), .B(shift_register[16]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[17])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i18_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i19_3_lut (.A(mic_latest[17]), .B(shift_register[17]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[18])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i19_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i20_3_lut (.A(mic_latest[18]), .B(shift_register[18]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[19])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i20_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i21_3_lut (.A(mic_latest[19]), .B(shift_register[19]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[20])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i21_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i22_3_lut (.A(mic_latest[20]), .B(shift_register[20]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[21])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i22_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i23_3_lut (.A(mic_latest[21]), .B(shift_register[21]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[22])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i23_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i24_3_lut (.A(mic_latest[22]), .B(shift_register[22]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[23])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i24_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i25_3_lut (.A(mic_latest[23]), .B(shift_register[23]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[24])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i25_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i26_3_lut (.A(mic_latest[24]), .B(shift_register[24]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[25])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i26_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i27_3_lut (.A(mic_latest[25]), .B(shift_register[25]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[26])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i27_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i28_3_lut (.A(mic_latest[26]), .B(shift_register[26]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[27])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i28_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i29_3_lut (.A(mic_latest[27]), .B(shift_register[27]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[28])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i29_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i30_3_lut (.A(mic_latest[28]), .B(shift_register[28]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[29])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i30_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i31_3_lut (.A(mic_latest[29]), .B(shift_register[29]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[30])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i31_3_lut.init = 16'hcaca;
    LUT4 shift_register_31__I_0_19_i32_3_lut (.A(mic_latest[30]), .B(shift_register[30]), 
         .C(n11_adj_3367), .Z(shift_register_31__N_3285[31])) /* synthesis lut_function=(A (B+!(C))+!A (B (C))) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam shift_register_31__I_0_19_i32_3_lut.init = 16'hcaca;
    LUT4 i4902_1_lut (.A(bit_count[5]), .Z(sck_N_3318_enable_36)) /* synthesis lut_function=(!(A)) */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam i4902_1_lut.init = 16'h5555;
    LUT4 i9789_2_lut (.A(n9[4]), .B(n11_adj_3367), .Z(n10302)) /* synthesis lut_function=(A (B)) */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam i9789_2_lut.init = 16'h8888;
    LUT4 i9788_2_lut (.A(n9[3]), .B(n11_adj_3367), .Z(n10300)) /* synthesis lut_function=(A (B)) */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam i9788_2_lut.init = 16'h8888;
    LUT4 i9787_2_lut (.A(n9[2]), .B(n11_adj_3367), .Z(n10298)) /* synthesis lut_function=(A (B)) */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam i9787_2_lut.init = 16'h8888;
    LUT4 i9783_2_lut (.A(n9[1]), .B(n11_adj_3367), .Z(n10296)) /* synthesis lut_function=(A (B)) */ ;   // src/spi_mic_stream.v(25[14] 31[8])
    defparam i9783_2_lut.init = 16'h8888;
    LUT4 i15900_2_lut (.A(bit_count[5]), .B(n11_adj_3367), .Z(sck_N_3318_enable_32)) /* synthesis lut_function=(!(A (B))) */ ;
    defparam i15900_2_lut.init = 16'h7777;
    LUT4 i9625_2_lut (.A(bit_count_5__N_3358[0]), .B(n11_adj_3367), .Z(bit_count_5__N_3319[0])) /* synthesis lut_function=(A+!(B)) */ ;   // src/spi_mic_stream.v(28[14] 31[8])
    defparam i9625_2_lut.init = 16'hbbbb;
    LUT4 i5_3_lut (.A(bit_count[3]), .B(n10_adj_3368), .C(bit_count[4]), 
         .Z(n11_adj_3367)) /* synthesis lut_function=(A+(B+(C))) */ ;   // src/spi_mic_stream.v(19[16:33])
    defparam i5_3_lut.init = 16'hfefe;
    LUT4 i4_4_lut (.A(bit_count[2]), .B(bit_count[5]), .C(bit_count[0]), 
         .D(bit_count[1]), .Z(n10_adj_3368)) /* synthesis lut_function=(A+(B+(C+(D)))) */ ;   // src/spi_mic_stream.v(19[16:33])
    defparam i4_4_lut.init = 16'hfffe;
    
endmodule
//
// Verilog Description of module TSALL
// module not written out since it is a black-box. 
//

//
// Verilog Description of module ws2812_stream
//

module ws2812_stream (reset_count, n17441, fpga_clk_c, fpga_clk_c_enable_215, 
            GND_net, rgb_values, rgb_data_c) /* synthesis syn_module_defined=1 */ ;
    output [12:0]reset_count;
    output n17441;
    input fpga_clk_c;
    input fpga_clk_c_enable_215;
    input GND_net;
    input [95:0]rgb_values;
    output rgb_data_c;
    
    wire fpga_clk_c /* synthesis SET_AS_NETWORK=fpga_clk_c, is_clock=1 */ ;   // src/umh_fpga_top.v(6[24:32])
    wire [5:0]bit_cell_count;   // src/ws2812_stream.v(15[11:25])
    
    wire n18784, n11, n18740, n4;
    wire [12:0]reset_count_c;   // src/ws2812_stream.v(14[12:23])
    
    wire n18826, n6, fpga_clk_c_enable_443;
    wire [5:0]n189;
    
    wire n17151, n17150, n17149, n25;
    wire [95:0]shift_register;   // src/ws2812_stream.v(17[12:26])
    
    wire n5603, n5602, n5601, n5600, n5599, n5598, n5597, n5596, 
        n5595, n5594, n5593, n5592, n5591, n5590, n5589, n5588, 
        n5587, n5586, n5585, n5584, n5583, n5582, n5581, n5580, 
        n5579, n5578, n5577, n5576, n5575, n5574, n5573, n5572, 
        n5571, n5570, n5569, n5568, n5567, n5566, n5565, n5564, 
        n5563, n5562, n5561, n5560, n5559, n5558, n17147;
    wire [12:0]n15;
    
    wire n17146, n5604, n5605, n5606, n5607, n17145, n5608, n17144, 
        n5609, n5610, n5611, n5612, n5613, n17143, n5614, n5615, 
        n5616, n5617, n17142, n5618, n5619, n5620, n5621, n5622, 
        n5623, n5624, n5625, n5626, n5627, n5628, n5629, n5630, 
        n5631, n5632, n5633, n5634, n5635, n5636, n5637, n5638, 
        n5639, n5640, n5641, n5642, n5643, n5644, n5645, n5646, 
        n5647, n5648, n5649, n5650, n5651, n5652, n24_adj_3366, 
        n10311;
    wire [5:0]high_count;   // src/ws2812_stream.v(19[12:22])
    
    wire n33, n47;
    wire [6:0]bit_number;   // src/ws2812_stream.v(16[11:21])
    wire [6:0]n1;
    
    wire fpga_clk_c_enable_442, n10293, n10287;
    wire [12:0]reset_count_12__N_3051;
    
    wire n13919, n17192, n17191, n17190, n18828;
    
    LUT4 i3_4_lut (.A(bit_cell_count[1]), .B(bit_cell_count[0]), .C(bit_cell_count[3]), 
         .D(n18784), .Z(n11)) /* synthesis lut_function=(A+(B+(C+!(D)))) */ ;
    defparam i3_4_lut.init = 16'hfeff;
    LUT4 i14664_3_lut (.A(bit_cell_count[5]), .B(bit_cell_count[4]), .C(bit_cell_count[2]), 
         .Z(n18784)) /* synthesis lut_function=(A (B (C))) */ ;
    defparam i14664_3_lut.init = 16'h8080;
    LUT4 i1_4_lut (.A(reset_count[2]), .B(reset_count[4]), .C(n17441), 
         .D(reset_count[5]), .Z(n18740)) /* synthesis lut_function=(((C+!(D))+!B)+!A) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam i1_4_lut.init = 16'hf7ff;
    LUT4 i1_2_lut (.A(reset_count[0]), .B(reset_count[1]), .Z(n4)) /* synthesis lut_function=(A+!(B)) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam i1_2_lut.init = 16'hbbbb;
    LUT4 i4_4_lut (.A(reset_count_c[12]), .B(n18826), .C(reset_count_c[10]), 
         .D(n6), .Z(n17441)) /* synthesis lut_function=(A+((C+(D))+!B)) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam i4_4_lut.init = 16'hfffb;
    LUT4 i14705_4_lut (.A(reset_count_c[11]), .B(reset_count_c[8]), .C(reset_count_c[7]), 
         .D(reset_count_c[6]), .Z(n18826)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14705_4_lut.init = 16'h8000;
    LUT4 i1_2_lut_adj_20 (.A(reset_count_c[3]), .B(reset_count_c[9]), .Z(n6)) /* synthesis lut_function=(A+(B)) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam i1_2_lut_adj_20.init = 16'heeee;
    FD1P3IX bit_cell_count__i0 (.D(n189[0]), .SP(fpga_clk_c_enable_215), 
            .CD(fpga_clk_c_enable_443), .CK(fpga_clk_c), .Q(bit_cell_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam bit_cell_count__i0.GSR = "ENABLED";
    CCU2D add_19_7 (.A0(bit_cell_count[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(GND_net), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17151), .S0(n189[5]));   // src/ws2812_stream.v(42[27:48])
    defparam add_19_7.INIT0 = 16'h5aaa;
    defparam add_19_7.INIT1 = 16'h0000;
    defparam add_19_7.INJECT1_0 = "NO";
    defparam add_19_7.INJECT1_1 = "NO";
    CCU2D add_19_5 (.A0(bit_cell_count[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17150), .COUT(n17151), .S0(n189[3]), .S1(n189[4]));   // src/ws2812_stream.v(42[27:48])
    defparam add_19_5.INIT0 = 16'h5aaa;
    defparam add_19_5.INIT1 = 16'h5aaa;
    defparam add_19_5.INJECT1_0 = "NO";
    defparam add_19_5.INJECT1_1 = "NO";
    CCU2D add_19_3 (.A0(bit_cell_count[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_cell_count[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17149), .COUT(n17150), .S0(n189[1]), .S1(n189[2]));   // src/ws2812_stream.v(42[27:48])
    defparam add_19_3.INIT0 = 16'h5aaa;
    defparam add_19_3.INIT1 = 16'h5aaa;
    defparam add_19_3.INJECT1_0 = "NO";
    defparam add_19_3.INJECT1_1 = "NO";
    CCU2D add_19_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(bit_cell_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n17149), .S1(n189[0]));   // src/ws2812_stream.v(42[27:48])
    defparam add_19_1.INIT0 = 16'hF000;
    defparam add_19_1.INIT1 = 16'h5555;
    defparam add_19_1.INJECT1_0 = "NO";
    defparam add_19_1.INJECT1_1 = "NO";
    LUT4 mux_2508_i47_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[38]), 
         .D(shift_register[45]), .Z(n5603)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i47_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i46_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[37]), 
         .D(shift_register[44]), .Z(n5602)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i46_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i45_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[36]), 
         .D(shift_register[43]), .Z(n5601)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i45_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i44_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[35]), 
         .D(shift_register[42]), .Z(n5600)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i44_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i43_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[34]), 
         .D(shift_register[41]), .Z(n5599)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i43_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i42_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[33]), 
         .D(shift_register[40]), .Z(n5598)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i42_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i46 (.D(n5603), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[46])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i46.GSR = "ENABLED";
    LUT4 mux_2508_i41_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[32]), 
         .D(shift_register[39]), .Z(n5597)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i41_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i45 (.D(n5602), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[45])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i45.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i44 (.D(n5601), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[44])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i44.GSR = "ENABLED";
    LUT4 mux_2508_i40_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[47]), 
         .D(shift_register[38]), .Z(n5596)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i40_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i43 (.D(n5600), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[43])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i43.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i42 (.D(n5599), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[42])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i42.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i41 (.D(n5598), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[41])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i41.GSR = "ENABLED";
    LUT4 mux_2508_i39_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[46]), 
         .D(shift_register[37]), .Z(n5595)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i39_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i40 (.D(n5597), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[40])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i40.GSR = "ENABLED";
    LUT4 mux_2508_i38_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[45]), 
         .D(shift_register[36]), .Z(n5594)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i38_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i39 (.D(n5596), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[39])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i39.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i38 (.D(n5595), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[38])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i38.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i37 (.D(n5594), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[37])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i37.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i36 (.D(n5593), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[36])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i36.GSR = "ENABLED";
    LUT4 mux_2508_i37_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[44]), 
         .D(shift_register[35]), .Z(n5593)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i37_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i36_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[43]), 
         .D(shift_register[34]), .Z(n5592)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i36_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i35_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[42]), 
         .D(shift_register[33]), .Z(n5591)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i35_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i34_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[41]), 
         .D(shift_register[32]), .Z(n5590)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i34_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i33_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[40]), 
         .D(shift_register[31]), .Z(n5589)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i33_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i35 (.D(n5592), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[35])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i35.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i34 (.D(n5591), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[34])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i34.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i33 (.D(n5590), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[33])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i33.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i32 (.D(n5589), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[32])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i32.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i31 (.D(n5588), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[31])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i31.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i30 (.D(n5587), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[30])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i30.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i29 (.D(n5586), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[29])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i29.GSR = "ENABLED";
    LUT4 mux_2508_i32_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[31]), 
         .D(shift_register[30]), .Z(n5588)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i32_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i28 (.D(n5585), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[28])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i28.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i27 (.D(n5584), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[27])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i27.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i26 (.D(n5583), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[26])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i26.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i25 (.D(n5582), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[25])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i25.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i24 (.D(n5581), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[24])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i24.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i23 (.D(n5580), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[23])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i23.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i22 (.D(n5579), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[22])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i22.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i21 (.D(n5578), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[21])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i21.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i20 (.D(n5577), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[20])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i20.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i19 (.D(n5576), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[19])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i19.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i18 (.D(n5575), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[18])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i18.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i17 (.D(n5574), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[17])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i17.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i16 (.D(n5573), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[16])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i16.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i15 (.D(n5572), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[15])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i15.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i14 (.D(n5571), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[14])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i14.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i13 (.D(n5570), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[13])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i13.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i12 (.D(n5569), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i12.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i11 (.D(n5568), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i11.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i10 (.D(n5567), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i10.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i9 (.D(n5566), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i9.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i8 (.D(n5565), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i8.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i7 (.D(n5564), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i7.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i6 (.D(n5563), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i6.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i5 (.D(n5562), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i5.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i4 (.D(n5561), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i4.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i3 (.D(n5560), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i3.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i2 (.D(n5559), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i2.GSR = "ENABLED";
    LUT4 mux_2508_i31_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[30]), 
         .D(shift_register[29]), .Z(n5587)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i31_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i1 (.D(n5558), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i1.GSR = "ENABLED";
    FD1P3IX bit_cell_count__i5 (.D(n189[5]), .SP(fpga_clk_c_enable_215), 
            .CD(fpga_clk_c_enable_443), .CK(fpga_clk_c), .Q(bit_cell_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam bit_cell_count__i5.GSR = "ENABLED";
    FD1P3IX bit_cell_count__i4 (.D(n189[4]), .SP(fpga_clk_c_enable_215), 
            .CD(fpga_clk_c_enable_443), .CK(fpga_clk_c), .Q(bit_cell_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam bit_cell_count__i4.GSR = "ENABLED";
    FD1P3IX bit_cell_count__i3 (.D(n189[3]), .SP(fpga_clk_c_enable_215), 
            .CD(fpga_clk_c_enable_443), .CK(fpga_clk_c), .Q(bit_cell_count[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam bit_cell_count__i3.GSR = "ENABLED";
    FD1P3IX bit_cell_count__i2 (.D(n189[2]), .SP(fpga_clk_c_enable_215), 
            .CD(fpga_clk_c_enable_443), .CK(fpga_clk_c), .Q(bit_cell_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam bit_cell_count__i2.GSR = "ENABLED";
    FD1P3IX bit_cell_count__i1 (.D(n189[1]), .SP(fpga_clk_c_enable_215), 
            .CD(fpga_clk_c_enable_443), .CK(fpga_clk_c), .Q(bit_cell_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam bit_cell_count__i1.GSR = "ENABLED";
    LUT4 mux_2508_i30_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[29]), 
         .D(shift_register[28]), .Z(n5586)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i30_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i29_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[28]), 
         .D(shift_register[27]), .Z(n5585)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i29_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i28_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[27]), 
         .D(shift_register[26]), .Z(n5584)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i28_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i27_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[26]), 
         .D(shift_register[25]), .Z(n5583)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i27_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i26_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[25]), 
         .D(shift_register[24]), .Z(n5582)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i26_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i25_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[24]), 
         .D(shift_register[23]), .Z(n5581)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i25_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i24_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[15]), 
         .D(shift_register[22]), .Z(n5580)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i24_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i23_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[14]), 
         .D(shift_register[21]), .Z(n5579)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i23_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i22_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[13]), 
         .D(shift_register[20]), .Z(n5578)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i22_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i21_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[12]), 
         .D(shift_register[19]), .Z(n5577)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i21_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i20_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[11]), 
         .D(shift_register[18]), .Z(n5576)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i20_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i19_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[10]), 
         .D(shift_register[17]), .Z(n5575)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i19_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i18_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[9]), 
         .D(shift_register[16]), .Z(n5574)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i18_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i17_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[8]), 
         .D(shift_register[15]), .Z(n5573)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i17_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i16_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[23]), 
         .D(shift_register[14]), .Z(n5572)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i16_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i15_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[22]), 
         .D(shift_register[13]), .Z(n5571)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i15_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i14_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[21]), 
         .D(shift_register[12]), .Z(n5570)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i14_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i13_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[20]), 
         .D(shift_register[11]), .Z(n5569)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i13_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i12_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[19]), 
         .D(shift_register[10]), .Z(n5568)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i12_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i11_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[18]), 
         .D(shift_register[9]), .Z(n5567)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i11_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i10_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[17]), 
         .D(shift_register[8]), .Z(n5566)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i10_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i9_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[16]), 
         .D(shift_register[7]), .Z(n5565)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i9_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i8_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[7]), 
         .D(shift_register[6]), .Z(n5564)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i8_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i7_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[6]), 
         .D(shift_register[5]), .Z(n5563)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i7_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i6_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[5]), 
         .D(shift_register[4]), .Z(n5562)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i6_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_9_13 (.A0(reset_count_c[11]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[12]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17147), .S0(n15[11]), .S1(n15[12]));   // src/ws2812_stream.v(30[28:46])
    defparam add_9_13.INIT0 = 16'h5aaa;
    defparam add_9_13.INIT1 = 16'h5aaa;
    defparam add_9_13.INJECT1_0 = "NO";
    defparam add_9_13.INJECT1_1 = "NO";
    LUT4 mux_2508_i5_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[4]), 
         .D(shift_register[3]), .Z(n5561)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i5_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i4_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[3]), 
         .D(shift_register[2]), .Z(n5560)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i4_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_9_11 (.A0(reset_count_c[9]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[10]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17146), .COUT(n17147), .S0(n15[9]), .S1(n15[10]));   // src/ws2812_stream.v(30[28:46])
    defparam add_9_11.INIT0 = 16'h5aaa;
    defparam add_9_11.INIT1 = 16'h5aaa;
    defparam add_9_11.INJECT1_0 = "NO";
    defparam add_9_11.INJECT1_1 = "NO";
    LUT4 mux_2508_i3_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[2]), 
         .D(shift_register[1]), .Z(n5559)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i3_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i2_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[1]), 
         .D(shift_register[0]), .Z(n5558)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i2_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i48_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[39]), 
         .D(shift_register[46]), .Z(n5604)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i48_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i49_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[48]), 
         .D(shift_register[47]), .Z(n5605)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i49_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i50_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[49]), 
         .D(shift_register[48]), .Z(n5606)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i50_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i51_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[50]), 
         .D(shift_register[49]), .Z(n5607)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i51_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_9_9 (.A0(reset_count_c[7]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count_c[8]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17145), .COUT(n17146), .S0(n15[7]), .S1(n15[8]));   // src/ws2812_stream.v(30[28:46])
    defparam add_9_9.INIT0 = 16'h5aaa;
    defparam add_9_9.INIT1 = 16'h5aaa;
    defparam add_9_9.INJECT1_0 = "NO";
    defparam add_9_9.INJECT1_1 = "NO";
    LUT4 mux_2508_i52_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[51]), 
         .D(shift_register[50]), .Z(n5608)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i52_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_9_7 (.A0(reset_count[5]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count_c[6]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17144), .COUT(n17145), .S0(n15[5]), .S1(n15[6]));   // src/ws2812_stream.v(30[28:46])
    defparam add_9_7.INIT0 = 16'h5aaa;
    defparam add_9_7.INIT1 = 16'h5aaa;
    defparam add_9_7.INJECT1_0 = "NO";
    defparam add_9_7.INJECT1_1 = "NO";
    LUT4 mux_2508_i53_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[52]), 
         .D(shift_register[51]), .Z(n5609)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i53_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i54_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[53]), 
         .D(shift_register[52]), .Z(n5610)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i54_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i55_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[54]), 
         .D(shift_register[53]), .Z(n5611)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i55_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i56_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[55]), 
         .D(shift_register[54]), .Z(n5612)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i56_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i57_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[64]), 
         .D(shift_register[55]), .Z(n5613)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i57_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_9_5 (.A0(reset_count_c[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(reset_count[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17143), .COUT(n17144), .S0(n15[3]), .S1(n15[4]));   // src/ws2812_stream.v(30[28:46])
    defparam add_9_5.INIT0 = 16'h5aaa;
    defparam add_9_5.INIT1 = 16'h5aaa;
    defparam add_9_5.INJECT1_0 = "NO";
    defparam add_9_5.INJECT1_1 = "NO";
    LUT4 mux_2508_i58_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[65]), 
         .D(shift_register[56]), .Z(n5614)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i58_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i59_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[66]), 
         .D(shift_register[57]), .Z(n5615)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i59_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i60_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[67]), 
         .D(shift_register[58]), .Z(n5616)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i60_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i61_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[68]), 
         .D(shift_register[59]), .Z(n5617)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i61_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_9_3 (.A0(reset_count[1]), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[2]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .CIN(n17142), .COUT(n17143), .S0(n15[1]), .S1(n15[2]));   // src/ws2812_stream.v(30[28:46])
    defparam add_9_3.INIT0 = 16'h5aaa;
    defparam add_9_3.INIT1 = 16'h5aaa;
    defparam add_9_3.INJECT1_0 = "NO";
    defparam add_9_3.INJECT1_1 = "NO";
    LUT4 mux_2508_i62_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[69]), 
         .D(shift_register[60]), .Z(n5618)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i62_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i63_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[70]), 
         .D(shift_register[61]), .Z(n5619)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i63_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i64_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[71]), 
         .D(shift_register[62]), .Z(n5620)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i64_3_lut_4_lut.init = 16'hf1e0;
    CCU2D add_9_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), .D0(GND_net), 
          .A1(reset_count[0]), .B1(GND_net), .C1(GND_net), .D1(GND_net), 
          .COUT(n17142), .S1(n15[0]));   // src/ws2812_stream.v(30[28:46])
    defparam add_9_1.INIT0 = 16'hF000;
    defparam add_9_1.INIT1 = 16'h5555;
    defparam add_9_1.INJECT1_0 = "NO";
    defparam add_9_1.INJECT1_1 = "NO";
    LUT4 mux_2508_i65_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[56]), 
         .D(shift_register[63]), .Z(n5621)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i65_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i66_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[57]), 
         .D(shift_register[64]), .Z(n5622)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i66_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i67_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[58]), 
         .D(shift_register[65]), .Z(n5623)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i67_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i68_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[59]), 
         .D(shift_register[66]), .Z(n5624)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i68_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i69_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[60]), 
         .D(shift_register[67]), .Z(n5625)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i69_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i70_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[61]), 
         .D(shift_register[68]), .Z(n5626)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i70_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i71_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[62]), 
         .D(shift_register[69]), .Z(n5627)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i71_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i72_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[63]), 
         .D(shift_register[70]), .Z(n5628)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i72_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i73_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[72]), 
         .D(shift_register[71]), .Z(n5629)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i73_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i74_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[73]), 
         .D(shift_register[72]), .Z(n5630)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i74_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i75_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[74]), 
         .D(shift_register[73]), .Z(n5631)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i75_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i76_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[75]), 
         .D(shift_register[74]), .Z(n5632)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i76_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i77_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[76]), 
         .D(shift_register[75]), .Z(n5633)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i77_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i78_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[77]), 
         .D(shift_register[76]), .Z(n5634)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i78_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i79_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[78]), 
         .D(shift_register[77]), .Z(n5635)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i79_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i80_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[79]), 
         .D(shift_register[78]), .Z(n5636)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i80_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i81_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[88]), 
         .D(shift_register[79]), .Z(n5637)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i81_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i82_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[89]), 
         .D(shift_register[80]), .Z(n5638)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i82_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i83_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[90]), 
         .D(shift_register[81]), .Z(n5639)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i83_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i84_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[91]), 
         .D(shift_register[82]), .Z(n5640)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i84_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i85_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[92]), 
         .D(shift_register[83]), .Z(n5641)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i85_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i86_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[93]), 
         .D(shift_register[84]), .Z(n5642)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i86_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i87_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[94]), 
         .D(shift_register[85]), .Z(n5643)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i87_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i88_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[95]), 
         .D(shift_register[86]), .Z(n5644)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i88_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i89_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[80]), 
         .D(shift_register[87]), .Z(n5645)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i89_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i90_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[81]), 
         .D(shift_register[88]), .Z(n5646)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i90_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i91_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[82]), 
         .D(shift_register[89]), .Z(n5647)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i91_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i92_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[83]), 
         .D(shift_register[90]), .Z(n5648)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i92_3_lut_4_lut.init = 16'hf1e0;
    FD1P3AX shift_register_i0_i47 (.D(n5604), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[47])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i47.GSR = "ENABLED";
    LUT4 mux_2508_i93_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[84]), 
         .D(shift_register[91]), .Z(n5649)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i93_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i94_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[85]), 
         .D(shift_register[92]), .Z(n5650)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i94_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i95_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[86]), 
         .D(shift_register[93]), .Z(n5651)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i95_3_lut_4_lut.init = 16'hf1e0;
    LUT4 mux_2508_i96_3_lut_4_lut (.A(n11), .B(n25), .C(rgb_values[87]), 
         .D(shift_register[94]), .Z(n5652)) /* synthesis lut_function=(A (C)+!A (B (C)+!B (D))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam mux_2508_i96_3_lut_4_lut.init = 16'hf1e0;
    LUT4 i6103_3_lut_4_lut (.A(n11), .B(n25), .C(n24_adj_3366), .D(fpga_clk_c_enable_443), 
         .Z(n10311)) /* synthesis lut_function=(A (D)+!A (B (D)+!B !(C+!(D)))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam i6103_3_lut_4_lut.init = 16'hef00;
    FD1P3AX shift_register_i0_i48 (.D(n5605), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[48])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i48.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i49 (.D(n5606), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[49])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i49.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i50 (.D(n5607), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[50])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i50.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i51 (.D(n5608), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[51])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i51.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i52 (.D(n5609), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[52])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i52.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i53 (.D(n5610), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[53])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i53.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i54 (.D(n5611), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[54])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i54.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i55 (.D(n5612), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[55])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i55.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i56 (.D(n5613), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[56])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i56.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i57 (.D(n5614), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[57])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i57.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i58 (.D(n5615), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[58])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i58.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i59 (.D(n5616), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[59])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i59.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i60 (.D(n5617), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[60])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i60.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i61 (.D(n5618), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[61])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i61.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i62 (.D(n5619), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[62])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i62.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i63 (.D(n5620), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[63])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i63.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i64 (.D(n5621), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[64])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i64.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i65 (.D(n5622), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[65])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i65.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i66 (.D(n5623), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[66])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i66.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i67 (.D(n5624), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[67])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i67.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i68 (.D(n5625), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[68])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i68.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i69 (.D(n5626), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[69])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i69.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i70 (.D(n5627), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[70])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i70.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i71 (.D(n5628), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[71])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i71.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i72 (.D(n5629), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[72])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i72.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i73 (.D(n5630), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[73])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i73.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i74 (.D(n5631), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[74])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i74.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i75 (.D(n5632), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[75])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i75.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i76 (.D(n5633), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[76])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i76.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i77 (.D(n5634), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[77])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i77.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i78 (.D(n5635), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[78])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i78.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i79 (.D(n5636), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[79])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i79.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i80 (.D(n5637), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[80])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i80.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i81 (.D(n5638), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[81])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i81.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i82 (.D(n5639), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[82])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i82.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i83 (.D(n5640), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[83])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i83.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i84 (.D(n5641), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[84])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i84.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i85 (.D(n5642), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[85])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i85.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i86 (.D(n5643), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[86])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i86.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i87 (.D(n5644), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[87])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i87.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i88 (.D(n5645), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[88])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i88.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i89 (.D(n5646), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[89])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i89.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i90 (.D(n5647), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[90])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i90.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i91 (.D(n5648), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[91])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i91.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i92 (.D(n5649), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[92])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i92.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i93 (.D(n5650), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[93])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i93.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i94 (.D(n5651), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(shift_register[94])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i94.GSR = "ENABLED";
    FD1P3AX shift_register_i0_i95 (.D(n5652), .SP(fpga_clk_c_enable_443), 
            .CK(fpga_clk_c), .Q(high_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i95.GSR = "ENABLED";
    LUT4 i15967_3_lut (.A(n33), .B(n25), .C(bit_cell_count[5]), .Z(rgb_data_c)) /* synthesis lut_function=(!(A+(B+(C)))) */ ;
    defparam i15967_3_lut.init = 16'h0101;
    LUT4 i7060_4_lut (.A(bit_cell_count[0]), .B(n47), .C(high_count[4]), 
         .D(bit_cell_count[4]), .Z(n33)) /* synthesis lut_function=(A (B ((D)+!C)+!B !(C+!(D)))+!A (B (D)+!B !(C+!(D)))) */ ;   // src/ws2812_stream.v(19[12:22])
    defparam i7060_4_lut.init = 16'hcf08;
    LUT4 i2_3_lut (.A(bit_cell_count[2]), .B(bit_cell_count[3]), .C(bit_cell_count[1]), 
         .Z(n47)) /* synthesis lut_function=(A (B (C))) */ ;   // src/ws2812_stream.v(15[11:25])
    defparam i2_3_lut.init = 16'h8080;
    FD1P3IX bit_number_2867__i6 (.D(n1[6]), .SP(fpga_clk_c_enable_443), 
            .CD(n10311), .CK(fpga_clk_c), .Q(bit_number[6])) /* synthesis syn_use_carry_chain=1 */ ;   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867__i6.GSR = "ENABLED";
    FD1P3IX bit_number_2867__i5 (.D(n1[5]), .SP(fpga_clk_c_enable_443), 
            .CD(n10311), .CK(fpga_clk_c), .Q(bit_number[5])) /* synthesis syn_use_carry_chain=1 */ ;   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867__i5.GSR = "ENABLED";
    FD1P3IX bit_number_2867__i4 (.D(n1[4]), .SP(fpga_clk_c_enable_443), 
            .CD(n10311), .CK(fpga_clk_c), .Q(bit_number[4])) /* synthesis syn_use_carry_chain=1 */ ;   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867__i4.GSR = "ENABLED";
    FD1P3IX bit_number_2867__i3 (.D(n1[3]), .SP(fpga_clk_c_enable_443), 
            .CD(n10311), .CK(fpga_clk_c), .Q(bit_number[3])) /* synthesis syn_use_carry_chain=1 */ ;   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867__i3.GSR = "ENABLED";
    FD1P3IX bit_number_2867__i2 (.D(n1[2]), .SP(fpga_clk_c_enable_443), 
            .CD(n10311), .CK(fpga_clk_c), .Q(bit_number[2])) /* synthesis syn_use_carry_chain=1 */ ;   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867__i2.GSR = "ENABLED";
    FD1P3IX bit_number_2867__i1 (.D(n1[1]), .SP(fpga_clk_c_enable_443), 
            .CD(n10311), .CK(fpga_clk_c), .Q(bit_number[1])) /* synthesis syn_use_carry_chain=1 */ ;   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867__i1.GSR = "ENABLED";
    FD1P3IX reset_count_i12 (.D(n15[12]), .SP(fpga_clk_c_enable_442), .CD(n10293), 
            .CK(fpga_clk_c), .Q(reset_count_c[12])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i12.GSR = "ENABLED";
    FD1P3IX reset_count_i11 (.D(reset_count_12__N_3051[11]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count_c[11])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i11.GSR = "ENABLED";
    FD1P3IX reset_count_i10 (.D(n15[10]), .SP(fpga_clk_c_enable_442), .CD(n10293), 
            .CK(fpga_clk_c), .Q(reset_count_c[10])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i10.GSR = "ENABLED";
    FD1P3IX reset_count_i9 (.D(n15[9]), .SP(fpga_clk_c_enable_442), .CD(n10293), 
            .CK(fpga_clk_c), .Q(reset_count_c[9])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i9.GSR = "ENABLED";
    FD1P3IX reset_count_i8 (.D(reset_count_12__N_3051[8]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count_c[8])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i8.GSR = "ENABLED";
    FD1P3IX reset_count_i7 (.D(reset_count_12__N_3051[7]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count_c[7])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i7.GSR = "ENABLED";
    FD1P3IX reset_count_i6 (.D(reset_count_12__N_3051[6]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count_c[6])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i6.GSR = "ENABLED";
    FD1P3IX reset_count_i5 (.D(reset_count_12__N_3051[5]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count[5])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i5.GSR = "ENABLED";
    FD1P3IX reset_count_i4 (.D(reset_count_12__N_3051[4]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count[4])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i4.GSR = "ENABLED";
    FD1P3IX reset_count_i3 (.D(n15[3]), .SP(fpga_clk_c_enable_442), .CD(n10293), 
            .CK(fpga_clk_c), .Q(reset_count_c[3])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i3.GSR = "ENABLED";
    FD1P3IX reset_count_i2 (.D(reset_count_12__N_3051[2]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count[2])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i2.GSR = "ENABLED";
    FD1P3IX reset_count_i1 (.D(reset_count_12__N_3051[1]), .SP(fpga_clk_c_enable_442), 
            .CD(n10287), .CK(fpga_clk_c), .Q(reset_count[1])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i1.GSR = "ENABLED";
    FD1P3IX bit_number_2867__i0 (.D(n1[0]), .SP(fpga_clk_c_enable_443), 
            .CD(n10311), .CK(fpga_clk_c), .Q(bit_number[0])) /* synthesis syn_use_carry_chain=1 */ ;   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867__i0.GSR = "ENABLED";
    FD1P3IX reset_count_i0 (.D(n15[0]), .SP(fpga_clk_c_enable_442), .CD(n10293), 
            .CK(fpga_clk_c), .Q(reset_count[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam reset_count_i0.GSR = "ENABLED";
    FD1P3IX shift_register_i0_i0 (.D(rgb_values[0]), .SP(fpga_clk_c_enable_443), 
            .CD(n13919), .CK(fpga_clk_c), .Q(shift_register[0])) /* synthesis lse_init_val=0, LSE_LINE_FILE_ID=6, LSE_LCOL=15, LSE_RCOL=2, LSE_LLINE=339, LSE_RLINE=346 */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam shift_register_i0_i0.GSR = "ENABLED";
    CCU2D bit_number_2867_add_4_7 (.A0(bit_number[5]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_number[6]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17192), .S0(n1[5]), .S1(n1[6]));   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867_add_4_7.INIT0 = 16'hfaaa;
    defparam bit_number_2867_add_4_7.INIT1 = 16'hfaaa;
    defparam bit_number_2867_add_4_7.INJECT1_0 = "NO";
    defparam bit_number_2867_add_4_7.INJECT1_1 = "NO";
    CCU2D bit_number_2867_add_4_5 (.A0(bit_number[3]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_number[4]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17191), .COUT(n17192), .S0(n1[3]), .S1(n1[4]));   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867_add_4_5.INIT0 = 16'hfaaa;
    defparam bit_number_2867_add_4_5.INIT1 = 16'hfaaa;
    defparam bit_number_2867_add_4_5.INJECT1_0 = "NO";
    defparam bit_number_2867_add_4_5.INJECT1_1 = "NO";
    CCU2D bit_number_2867_add_4_3 (.A0(bit_number[1]), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_number[2]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .CIN(n17190), .COUT(n17191), .S0(n1[1]), .S1(n1[2]));   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867_add_4_3.INIT0 = 16'hfaaa;
    defparam bit_number_2867_add_4_3.INIT1 = 16'hfaaa;
    defparam bit_number_2867_add_4_3.INJECT1_0 = "NO";
    defparam bit_number_2867_add_4_3.INJECT1_1 = "NO";
    CCU2D bit_number_2867_add_4_1 (.A0(GND_net), .B0(GND_net), .C0(GND_net), 
          .D0(GND_net), .A1(bit_number[0]), .B1(GND_net), .C1(GND_net), 
          .D1(GND_net), .COUT(n17190), .S1(n1[0]));   // src/ws2812_stream.v(39[27:44])
    defparam bit_number_2867_add_4_1.INIT0 = 16'hF000;
    defparam bit_number_2867_add_4_1.INIT1 = 16'h0555;
    defparam bit_number_2867_add_4_1.INJECT1_0 = "NO";
    defparam bit_number_2867_add_4_1.INJECT1_1 = "NO";
    LUT4 i1_2_lut_3_lut_4_lut (.A(n11), .B(n24_adj_3366), .C(n18740), 
         .D(n4), .Z(fpga_clk_c_enable_442)) /* synthesis lut_function=(A (C+(D))+!A ((C+(D))+!B)) */ ;   // src/ws2812_stream.v(35[13:32])
    defparam i1_2_lut_3_lut_4_lut.init = 16'hfff1;
    LUT4 i15975_2_lut_3_lut_4_lut (.A(n11), .B(n24_adj_3366), .C(n18740), 
         .D(n4), .Z(n10287)) /* synthesis lut_function=(!(A+(B+(C+(D))))) */ ;   // src/ws2812_stream.v(35[13:32])
    defparam i15975_2_lut_3_lut_4_lut.init = 16'h0001;
    LUT4 i9766_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[5]), .Z(reset_count_12__N_3051[5])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9766_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i9765_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[4]), .Z(reset_count_12__N_3051[4])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9765_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i9763_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[2]), .Z(reset_count_12__N_3051[2])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9763_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i9770_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[11]), .Z(reset_count_12__N_3051[11])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9770_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i9769_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[8]), .Z(reset_count_12__N_3051[8])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9769_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i9768_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[7]), .Z(reset_count_12__N_3051[7])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9768_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i9762_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[1]), .Z(reset_count_12__N_3051[1])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9762_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i9767_2_lut_3_lut_4_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .D(n15[6]), .Z(reset_count_12__N_3051[6])) /* synthesis lut_function=(A (D)+!A (B ((D)+!C)+!B (D))) */ ;   // src/ws2812_stream.v(24[13:36])
    defparam i9767_2_lut_3_lut_4_lut.init = 16'hff04;
    LUT4 i15991_2_lut_3_lut_4_lut (.A(n11), .B(n18740), .C(n4), .D(fpga_clk_c_enable_443), 
         .Z(n13919)) /* synthesis lut_function=(!(A+(B+(C+!(D))))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam i15991_2_lut_3_lut_4_lut.init = 16'h0100;
    LUT4 i2_2_lut_3_lut (.A(n18740), .B(reset_count[0]), .C(reset_count[1]), 
         .Z(n25)) /* synthesis lut_function=(A+(B+!(C))) */ ;   // src/ws2812_stream.v(18[21:44])
    defparam i2_2_lut_3_lut.init = 16'hefef;
    LUT4 i6055_4_lut_4_lut (.A(fpga_clk_c_enable_442), .B(n18740), .C(reset_count[0]), 
         .D(reset_count[1]), .Z(n10293)) /* synthesis lut_function=(!((B+(C (D)+!C !(D)))+!A)) */ ;   // src/ws2812_stream.v(22[8] 44[4])
    defparam i6055_4_lut_4_lut.init = 16'h0220;
    LUT4 n18740_bdd_4_lut (.A(n18740), .B(reset_count[1]), .C(reset_count[0]), 
         .D(n11), .Z(fpga_clk_c_enable_443)) /* synthesis lut_function=(!(A+(B (C+(D))+!B !(C)))) */ ;
    defparam n18740_bdd_4_lut.init = 16'h1014;
    LUT4 i1_4_lut_adj_21 (.A(bit_number[5]), .B(bit_number[3]), .C(n18828), 
         .D(bit_number[4]), .Z(n24_adj_3366)) /* synthesis lut_function=(A+!(B (C (D)))) */ ;
    defparam i1_4_lut_adj_21.init = 16'hbfff;
    LUT4 i14707_4_lut (.A(bit_number[2]), .B(bit_number[6]), .C(bit_number[0]), 
         .D(bit_number[1]), .Z(n18828)) /* synthesis lut_function=(A (B (C (D)))) */ ;
    defparam i14707_4_lut.init = 16'h8000;
    
endmodule
//
// Verilog Description of module PUR
// module not written out since it is a black-box. 
//

