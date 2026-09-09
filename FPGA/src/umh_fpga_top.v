`timescale 1ns/1ps

// UMH v7 MachXO2 output controller.
// SPI1 uses mode 0 and returns the status that was current at CS assertion.
module umh_fpga_top (
    input  wire        fpga_clk,
    input  wire        fpga_cs_n,
    input  wire        spi1_sck,
    input  wire        spi1_mosi,
    output wire        spi1_miso,
    output wire [83:0] us_tx,
    output wire        rgb_data,
    output wire        mic_clk,
    input  wire        mic_data_0,
    input  wire        mic_data_1,
    input  wire        spi_mic_cs_n,
    input  wire        spi_mic_sck,
    output wire        spi_mic_miso
);

localparam [31:0] CARRIER_STEP = 32'd4042322;   // 40 kHz from 42.5 MHz
localparam [15:0] HEADER_BYTES = 16'd36;

// These are deliberately registers rather than inferred distributed RAM.  The
// selected device has only 2112 LUT4s; retaining the complete byte values here
// makes LSE replicate the RAM for the 84 parallel output taps.  The output
// datapath uses one phase bit and a two-bit level envelope, so retain only
// those implemented precision bits in the active bank.  The EBR still keeps
// the incoming byte so the SPI frame format remains unchanged.
(* syn_ramstyle = "registers" *) reg       phase_active[0:83];
// Zero is the disabled code; 1..3 are the implemented two-bit amplitudes.
(* syn_ramstyle = "registers" *) reg [1:0] level_active[0:83];
reg [95:0]  rgb_values;

reg [7:0] spi_rx_shift;
reg [2:0] spi_bit_count;
reg [15:0] spi_byte_count;
reg [7:0] spi_command;
reg [7:0] spi_version;
reg [15:0] spi_update_flags;
reg [15:0] spi_extension_length;
reg [31:0] spi_frame_sequence;
reg [31:0] spi_expected_length;
reg [6:0] spi_channel_index;
reg [1:0] spi_channel_field;
reg [7:0] spi_level_pending;
reg [31:0] accepted_sequence_spi;
reg        frame_toggle_spi;
reg        stop_toggle_spi;
reg        invalid_frame_spi;
reg [6:0] status_bit_index;

reg frame_toggle_meta;
reg frame_toggle_sync;
reg frame_toggle_seen;
reg stop_toggle_meta;
reg stop_toggle_sync;
reg stop_toggle_seen;
reg apply_pending;
reg running;
reg [31:0] accepted_sequence;
reg [31:0] phase_acc;
reg [31:0] fpga_time;
reg [5:0] time_divider;
reg       time_half;
reg [7:0] amplitude_phase;
reg [6:0] mic_divider;
reg mic_clock_reg;
reg [15:0] mic_shift_0;
reg [15:0] mic_shift_1;
reg [4:0] mic_sample_count;
reg [31:0] mic_latest;
reg [6:0] load_index;
reg load_active;
integer i;

wire [12:0] channel_mem_addr_a = {3'b000, spi_channel_index, 3'b001};
wire [12:0] channel_mem_addr_b = {3'b000, load_index, 3'b001};
wire [8:0] phase_mem_din = {1'b0, {spi_rx_shift[6:0], spi1_mosi}};
wire [8:0] level_mem_din = {{spi_rx_shift[6:0], spi1_mosi} != 8'd0, spi_level_pending};
wire [8:0] phase_mem_dout;
wire [8:0] level_mem_dout;
wire mem_write_phase = !fpga_cs_n && spi_update_flags[0] &&
                       (spi_bit_count == 3'd7) &&
                       (spi_byte_count >= HEADER_BYTES) &&
                       (spi_byte_count < HEADER_BYTES + 16'd336) &&
                       (spi_channel_field == 2'd1);
wire mem_write_level = !fpga_cs_n && spi_update_flags[0] &&
                       (spi_bit_count == 3'd7) &&
                       (spi_byte_count >= HEADER_BYTES) &&
                       (spi_byte_count < HEADER_BYTES + 16'd336) &&
                       (spi_channel_field == 2'd3);

// Two 9-bit EBR ports hold one phase byte and one level/enable byte per
// channel.  Port A is written by the STM32 SPI clock; port B is read while
// the FPGA clock refreshes the active output registers.
DP8KC #(.DATA_WIDTH_A(9), .DATA_WIDTH_B(9), .REGMODE_A("NOREG"),
         .REGMODE_B("NOREG"), .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"))
phase_mem_i (
    .DIA8(phase_mem_din[8]), .DIA7(phase_mem_din[7]), .DIA6(phase_mem_din[6]),
    .DIA5(phase_mem_din[5]), .DIA4(phase_mem_din[4]), .DIA3(phase_mem_din[3]),
    .DIA2(phase_mem_din[2]), .DIA1(phase_mem_din[1]), .DIA0(phase_mem_din[0]),
    .ADA12(channel_mem_addr_a[12]), .ADA11(channel_mem_addr_a[11]), .ADA10(channel_mem_addr_a[10]),
    .ADA9(channel_mem_addr_a[9]), .ADA8(channel_mem_addr_a[8]), .ADA7(channel_mem_addr_a[7]),
    .ADA6(channel_mem_addr_a[6]), .ADA5(channel_mem_addr_a[5]), .ADA4(channel_mem_addr_a[4]),
    .ADA3(channel_mem_addr_a[3]), .ADA2(channel_mem_addr_a[2]), .ADA1(channel_mem_addr_a[1]), .ADA0(channel_mem_addr_a[0]),
    .CEA(1'b1), .OCEA(1'b1), .CLKA(spi1_sck), .WEA(mem_write_phase), .CSA2(1'b0), .CSA1(1'b0), .CSA0(1'b0), .RSTA(1'b0),
    .DIB8(1'b0), .DIB7(1'b0), .DIB6(1'b0), .DIB5(1'b0), .DIB4(1'b0), .DIB3(1'b0), .DIB2(1'b0), .DIB1(1'b0), .DIB0(1'b0),
    .ADB12(channel_mem_addr_b[12]), .ADB11(channel_mem_addr_b[11]), .ADB10(channel_mem_addr_b[10]),
    .ADB9(channel_mem_addr_b[9]), .ADB8(channel_mem_addr_b[8]), .ADB7(channel_mem_addr_b[7]), .ADB6(channel_mem_addr_b[6]),
    .ADB5(channel_mem_addr_b[5]), .ADB4(channel_mem_addr_b[4]), .ADB3(channel_mem_addr_b[3]), .ADB2(channel_mem_addr_b[2]),
    .ADB1(channel_mem_addr_b[1]), .ADB0(channel_mem_addr_b[0]), .CEB(1'b1), .OCEB(1'b1), .CLKB(fpga_clk), .WEB(1'b0),
    .CSB2(1'b0), .CSB1(1'b0), .CSB0(1'b0), .RSTB(1'b0),
    .DOA8(), .DOA7(), .DOA6(), .DOA5(), .DOA4(), .DOA3(), .DOA2(), .DOA1(), .DOA0(),
    .DOB8(phase_mem_dout[8]), .DOB7(phase_mem_dout[7]), .DOB6(phase_mem_dout[6]), .DOB5(phase_mem_dout[5]),
    .DOB4(phase_mem_dout[4]), .DOB3(phase_mem_dout[3]), .DOB2(phase_mem_dout[2]), .DOB1(phase_mem_dout[1]), .DOB0(phase_mem_dout[0]));

DP8KC #(.DATA_WIDTH_A(9), .DATA_WIDTH_B(9), .REGMODE_A("NOREG"),
         .REGMODE_B("NOREG"), .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"))
level_mem_i (
    .DIA8(level_mem_din[8]), .DIA7(level_mem_din[7]), .DIA6(level_mem_din[6]),
    .DIA5(level_mem_din[5]), .DIA4(level_mem_din[4]), .DIA3(level_mem_din[3]),
    .DIA2(level_mem_din[2]), .DIA1(level_mem_din[1]), .DIA0(level_mem_din[0]),
    .ADA12(channel_mem_addr_a[12]), .ADA11(channel_mem_addr_a[11]), .ADA10(channel_mem_addr_a[10]),
    .ADA9(channel_mem_addr_a[9]), .ADA8(channel_mem_addr_a[8]), .ADA7(channel_mem_addr_a[7]),
    .ADA6(channel_mem_addr_a[6]), .ADA5(channel_mem_addr_a[5]), .ADA4(channel_mem_addr_a[4]),
    .ADA3(channel_mem_addr_a[3]), .ADA2(channel_mem_addr_a[2]), .ADA1(channel_mem_addr_a[1]), .ADA0(channel_mem_addr_a[0]),
    .CEA(1'b1), .OCEA(1'b1), .CLKA(spi1_sck), .WEA(mem_write_level), .CSA2(1'b0), .CSA1(1'b0), .CSA0(1'b0), .RSTA(1'b0),
    .DIB8(1'b0), .DIB7(1'b0), .DIB6(1'b0), .DIB5(1'b0), .DIB4(1'b0), .DIB3(1'b0), .DIB2(1'b0), .DIB1(1'b0), .DIB0(1'b0),
    .ADB12(channel_mem_addr_b[12]), .ADB11(channel_mem_addr_b[11]), .ADB10(channel_mem_addr_b[10]),
    .ADB9(channel_mem_addr_b[9]), .ADB8(channel_mem_addr_b[8]), .ADB7(channel_mem_addr_b[7]), .ADB6(channel_mem_addr_b[6]),
    .ADB5(channel_mem_addr_b[5]), .ADB4(channel_mem_addr_b[4]), .ADB3(channel_mem_addr_b[3]), .ADB2(channel_mem_addr_b[2]),
    .ADB1(channel_mem_addr_b[1]), .ADB0(channel_mem_addr_b[0]), .CEB(1'b1), .OCEB(1'b1), .CLKB(fpga_clk), .WEB(1'b0),
    .CSB2(1'b0), .CSB1(1'b0), .CSB0(1'b0), .RSTB(1'b0),
    .DOA8(), .DOA7(), .DOA6(), .DOA5(), .DOA4(), .DOA3(), .DOA2(), .DOA1(), .DOA0(),
    .DOB8(level_mem_dout[8]), .DOB7(level_mem_dout[7]), .DOB6(level_mem_dout[6]), .DOB5(level_mem_dout[5]),
    .DOB4(level_mem_dout[4]), .DOB3(level_mem_dout[3]), .DOB2(level_mem_dout[2]), .DOB1(level_mem_dout[1]), .DOB0(level_mem_dout[0]));

wire [31:0] phase_acc_next = phase_acc + CARRIER_STEP;
wire carrier_wrap = phase_acc_next < phase_acc;

genvar channel;
generate
    for (channel = 0; channel < 84; channel = channel + 1) begin : CHANNEL_OUTPUTS
        // The small device cannot afford 84 full-width phase adders.  The
        // stored phase MSB provides a 180-degree per-channel phase choice;
        // amplitude keeps its upper nibble for a 16-step envelope.
        wire carrier_bit = phase_acc[31] ^ phase_active[channel];
        // A zero level is already rejected by the greater-than comparison.
        assign us_tx[channel] = running &&
                                (level_active[channel] > amplitude_phase[7:6]) &&
                                carrier_bit;
    end
endgenerate

// Status byte order is the STM32 packed little-endian wire struct.
wire [15:0] fifo_credit = apply_pending ? 16'd0 : 16'd1;
wire [15:0] fifo_depth = apply_pending ? 16'd1 : 16'd0;
wire [15:0] status_flags_wire = (invalid_frame_spi ? 16'h0004 : 16'h0000) |
                                (running ? 16'h0010 : 16'h0000);
wire [127:0] status_word = {8'h01, 8'h00,
                             fifo_credit[7:0], fifo_credit[15:8],
                             fifo_depth[7:0], fifo_depth[15:8],
                             status_flags_wire[7:0], status_flags_wire[15:8],
                             fpga_time[7:0], fpga_time[15:8],
                             fpga_time[23:16], fpga_time[31:24],
                             accepted_sequence[7:0], accepted_sequence[15:8],
                             accepted_sequence[23:16], accepted_sequence[31:24]};

// CPOL=0, CPHA=0: present the MSB before the first rising edge and advance on falling edges.
assign spi1_miso = fpga_cs_n ? 1'b0 : status_word[127 - status_bit_index];

always @(negedge spi1_sck or posedge fpga_cs_n) begin
    if (fpga_cs_n)
        status_bit_index <= 7'd0;
    else if (status_bit_index != 7'd127)
        status_bit_index <= status_bit_index + 1'b1;
end

// The STM32 sends one complete transaction under CS.  Its current producer sends full
// channel and RGB masks; other masks are rejected so an incomplete sparse update cannot
// be mistaken for a complete output frame.
always @(posedge spi1_sck or negedge fpga_cs_n) begin
    if (!fpga_cs_n) begin
        if (spi_bit_count == 3'd7) begin
            spi_bit_count <= 3'd0;
            spi_rx_shift <= {spi_rx_shift[6:0], spi1_mosi};
            case (spi_byte_count)
                16'd0: spi_command <= {spi_rx_shift[6:0], spi1_mosi};
                16'd1: spi_version <= {spi_rx_shift[6:0], spi1_mosi};
                16'd6: spi_frame_sequence[7:0] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd7: spi_frame_sequence[15:8] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd8: spi_frame_sequence[23:16] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd9: spi_frame_sequence[31:24] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd18: spi_update_flags[7:0] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd19: spi_update_flags[15:8] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd34: spi_extension_length[7:0] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd35: begin
                    spi_extension_length[15:8] <= {spi_rx_shift[6:0], spi1_mosi};
                    spi_expected_length <= HEADER_BYTES +
                        (spi_update_flags[0] ? 32'd336 : 32'd0) +
                        (spi_update_flags[1] ? 32'd12 : 32'd0) +
                        {8'd0, {spi_rx_shift[6:0], spi1_mosi}, spi_extension_length[7:0]};
                end
                default: begin
                    if (spi_update_flags[0] &&
                        spi_byte_count >= HEADER_BYTES && spi_byte_count < HEADER_BYTES + 16'd336) begin
                        // The channel payload is four bytes per channel.  The
                        // first byte is reserved; bytes 1, 2 and 3 carry phase,
                        // level and enable.  Updating the active bank directly
                        // keeps this implementation within the 2k LUT/FF device.
                        case (spi_channel_field)
                            2'd1: begin end // phase_mem_i writes this byte
                            2'd2: spi_level_pending <= {spi_rx_shift[6:0], spi1_mosi};
                            2'd3: begin
                                if (spi_channel_index != 7'd83)
                                    spi_channel_index <= spi_channel_index + 1'b1;
                            end
                            default: begin end
                        endcase
                        if (spi_channel_field == 2'd3)
                            spi_channel_field <= 2'd0;
                        else
                            spi_channel_field <= spi_channel_field + 1'b1;
                    end else if (spi_update_flags[1] &&
                                 spi_byte_count >= HEADER_BYTES +
                                 (spi_update_flags[0] ? 16'd336 : 16'd0) &&
                                 spi_byte_count < HEADER_BYTES +
                                 (spi_update_flags[0] ? 16'd336 : 16'd0) + 16'd12) begin
                        rgb_values <= {rgb_values[87:0], {spi_rx_shift[6:0], spi1_mosi}};
                    end
                end
            endcase
            spi_byte_count <= spi_byte_count + 1'b1;
        end else begin
            spi_rx_shift <= {spi_rx_shift[6:0], spi1_mosi};
            spi_bit_count <= spi_bit_count + 1'b1;
        end
    end else begin
        spi_rx_shift <= 8'd0;
        spi_bit_count <= 3'd0;
        spi_byte_count <= 16'd0;
        spi_expected_length <= HEADER_BYTES;
        spi_command <= 8'd0;
        spi_version <= 8'd0;
        spi_update_flags <= 16'd0;
        spi_extension_length <= 16'd0;
        spi_frame_sequence <= 32'd0;
        spi_channel_index <= 7'd0;
        spi_channel_field <= 2'd0;
        spi_level_pending <= 8'd0;
    end
end

always @(posedge fpga_cs_n) begin
    if (spi_byte_count == spi_expected_length) begin
        if (spi_command == 8'h10 && spi_version == 8'h01 &&
            spi_extension_length <= 16'd32) begin
            frame_toggle_spi <= ~frame_toggle_spi;
            accepted_sequence_spi <= spi_frame_sequence;
            invalid_frame_spi <= 1'b0;
        end else if (spi_command == 8'h11 || spi_command == 8'h12) begin
            stop_toggle_spi <= ~stop_toggle_spi;
            invalid_frame_spi <= 1'b0;
        end else if (spi_command != 8'h01) begin
            invalid_frame_spi <= 1'b1;
        end
    end else if (spi_command != 8'h00) begin
        invalid_frame_spi <= 1'b1;
    end
end

always @(posedge fpga_clk) begin
    phase_acc <= phase_acc_next;
    // Alternate 42 and 43 reference-clock cycles.  Their average is 42.5,
    // producing the documented 1 MHz FPGA timebase without a long 32-bit
    // accumulator-to-counter timing path.
    if (time_divider == (time_half ? 6'd42 : 6'd41)) begin
        time_divider <= 6'd0;
        time_half <= ~time_half;
        fpga_time <= fpga_time + 1'b1;
    end else begin
        time_divider <= time_divider + 1'b1;
    end

    frame_toggle_meta <= frame_toggle_spi;
    frame_toggle_sync <= frame_toggle_meta;
    stop_toggle_meta <= stop_toggle_spi;
    stop_toggle_sync <= stop_toggle_meta;
    if (frame_toggle_sync != frame_toggle_seen) begin
        frame_toggle_seen <= frame_toggle_sync;
        apply_pending <= 1'b1;
        load_active <= 1'b1;
        load_index <= 7'd0;
        running <= 1'b0;
    end
    if (stop_toggle_sync != stop_toggle_seen) begin
        stop_toggle_seen <= stop_toggle_sync;
        running <= 1'b0;
        apply_pending <= 1'b0;
        stop_toggle_seen <= stop_toggle_sync;
    end

    if (load_active) begin
        phase_active[load_index] <= phase_mem_dout[7];
        level_active[load_index] <= level_mem_dout[8] ? level_mem_dout[7:6] : 2'd0;
        if (load_index == 7'd83)
            load_active <= 1'b0;
        else
            load_index <= load_index + 1'b1;
    end

    if (carrier_wrap) begin
        amplitude_phase <= amplitude_phase + 1'b1;
        if (apply_pending && !load_active) begin
            accepted_sequence <= accepted_sequence_spi;
            running <= 1'b1;
            apply_pending <= 1'b0;
        end
    end

    if (mic_divider == 7'd6) begin
        mic_divider <= 7'd0;
        mic_clock_reg <= ~mic_clock_reg;
        if (!mic_clock_reg) begin
            mic_shift_0 <= {mic_shift_0[14:0], mic_data_0};
            mic_shift_1 <= {mic_shift_1[14:0], mic_data_1};
            mic_sample_count <= mic_sample_count + 1'b1;
            if (mic_sample_count == 5'd15)
                mic_latest <= {mic_shift_0[14:0], mic_data_0, mic_shift_1[14:0], mic_data_1};
        end
    end else begin
        mic_divider <= mic_divider + 1'b1;
    end
end

assign mic_clk = mic_clock_reg;

ws2812_stream ws2812_i (
    .clk(fpga_clk),
    .g0(rgb_values[87:80]), .r0(rgb_values[95:88]), .b0(rgb_values[79:72]),
    .g1(rgb_values[63:56]), .r1(rgb_values[71:64]), .b1(rgb_values[55:48]),
    .g2(rgb_values[39:32]), .r2(rgb_values[47:40]), .b2(rgb_values[31:24]),
    .g3(rgb_values[15:8]), .r3(rgb_values[23:16]), .b3(rgb_values[7:0]),
    .data_out(rgb_data)
);

spi_mic_stream mic_stream_i (
    .cs_n(spi_mic_cs_n), .sck(spi_mic_sck),
    .sample_word(mic_latest), .miso(spi_mic_miso)
);

initial begin
    spi_rx_shift = 8'd0;
    spi_bit_count = 3'd0;
    spi_byte_count = 16'd0;
    spi_command = 8'd0;
    spi_version = 8'd0;
    spi_update_flags = 16'd0;
    spi_extension_length = 16'd0;
    spi_frame_sequence = 32'd0;
    spi_expected_length = HEADER_BYTES;
    spi_channel_index = 7'd0;
    spi_channel_field = 2'd0;
    spi_level_pending = 8'd0;
    accepted_sequence_spi = 32'd0;
    frame_toggle_spi = 1'b0;
    stop_toggle_spi = 1'b0;
    invalid_frame_spi = 1'b0;
    status_bit_index = 7'd0;
    frame_toggle_meta = 1'b0;
    frame_toggle_sync = 1'b0;
    frame_toggle_seen = 1'b0;
    stop_toggle_meta = 1'b0;
    stop_toggle_sync = 1'b0;
    stop_toggle_seen = 1'b0;
    apply_pending = 1'b0;
    running = 1'b0;
    accepted_sequence = 32'd0;
    phase_acc = 32'd0;
    fpga_time = 32'd0;
    time_divider = 6'd0;
    time_half = 1'b0;
    amplitude_phase = 8'd0;
    mic_divider = 7'd0;
    mic_clock_reg = 1'b0;
    mic_shift_0 = 16'd0;
    mic_shift_1 = 16'd0;
    mic_sample_count = 5'd0;
    mic_latest = 32'd0;
    rgb_values = 96'd0;
    load_index = 7'd0;
    load_active = 1'b0;
    for (i = 0; i < 84; i = i + 1) begin
        phase_active[i] = 1'b0;
        level_active[i] = 2'd0;
    end
end

endmodule
