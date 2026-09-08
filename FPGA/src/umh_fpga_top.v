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
    input  wire        spi_mic_mosi,
    output wire        spi_mic_miso
);

localparam [31:0] CARRIER_STEP = 32'd4042322;   // 40 kHz from 42.5 MHz
localparam [31:0] TIME_STEP    = 32'd101058054; // 1 MHz timebase from 42.5 MHz
localparam [15:0] HEADER_BYTES = 16'd36;

reg [7:0] phase_stage [0:83];
reg [7:0] level_stage [0:83];
reg       enable_stage[0:83];
reg [7:0] phase_active[0:83];
reg [7:0] level_active[0:83];
reg       enable_active[0:83];
reg [7:0] rgb_stage [0:11];
reg [7:0] rgb_active[0:11];

reg [7:0] spi_rx_shift;
reg [2:0] spi_bit_count;
reg [15:0] spi_byte_count;
reg [7:0] spi_command;
reg [7:0] spi_version;
reg [15:0] spi_update_flags;
reg [87:0] spi_channel_bitmap;
reg [3:0] spi_rgb_bitmap;
reg [15:0] spi_extension_length;
reg [31:0] spi_frame_sequence;
reg [31:0] spi_expected_length;
reg [31:0] accepted_sequence_spi;
reg        frame_toggle_spi;
reg        stop_toggle_spi;
reg        invalid_frame_spi;
reg [127:0] status_latched;
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
reg [31:0] time_acc;
reg [31:0] fpga_time;
reg [7:0] amplitude_phase;
reg [15:0] status_flags;
reg [6:0] mic_divider;
reg mic_clock_reg;
reg [15:0] mic_shift_0;
reg [15:0] mic_shift_1;
reg [4:0] mic_sample_count;
reg [31:0] mic_latest;
integer i;

wire [31:0] phase_acc_next = phase_acc + CARRIER_STEP;
wire [31:0] time_acc_next = time_acc + TIME_STEP;
wire carrier_wrap = phase_acc_next < phase_acc;

function [8:0] phase_sum;
    input [7:0] a;
    input [7:0] b;
    begin
        phase_sum = {1'b0, a} + {1'b0, b};
    end
endfunction

genvar channel;
generate
    for (channel = 0; channel < 84; channel = channel + 1) begin : CHANNEL_OUTPUTS
        wire [8:0] shifted_phase;
        assign shifted_phase = phase_sum(phase_acc[31:24], phase_active[channel]);
        assign us_tx[channel] = running && enable_active[channel] &&
                                (level_active[channel] > amplitude_phase) &&
                                shifted_phase[7];
    end
endgenerate

// Status byte order is the STM32 packed little-endian wire struct.
wire [15:0] fifo_credit = apply_pending ? 16'd0 : 16'd1;
wire [15:0] fifo_depth = apply_pending ? 16'd1 : 16'd0;
wire [15:0] status_flags_wire = status_flags | (invalid_frame_spi ? 16'h0004 : 16'h0000) |
                                (running ? 16'h0010 : 16'h0000);
wire [127:0] status_word = {accepted_sequence, fpga_time, status_flags_wire,
                             fifo_depth, fifo_credit, 8'h00, 8'h01};

// CPOL=0, CPHA=0: present the MSB before the first rising edge and advance on falling edges.
assign spi1_miso = fpga_cs_n ? 1'b0 : status_latched[127 - status_bit_index];
always @(negedge fpga_cs_n)
    status_latched <= status_word;

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
                16'd31: spi_rgb_bitmap <= {spi_rx_shift[3:0], spi1_mosi};
                16'd34: spi_extension_length[7:0] <= {spi_rx_shift[6:0], spi1_mosi};
                16'd35: begin
                    spi_extension_length[15:8] <= {spi_rx_shift[6:0], spi1_mosi};
                    spi_expected_length <= HEADER_BYTES +
                        (spi_update_flags[0] ? 32'd336 : 32'd0) +
                        (spi_update_flags[1] ? 32'd12 : 32'd0) +
                        {16'd0, {spi_rx_shift[6:0], spi1_mosi}};
                end
                default: begin
                    if (spi_byte_count >= 16'd20 && spi_byte_count <= 16'd30)
                        spi_channel_bitmap[(spi_byte_count - 16'd20) * 8 +: 8] <= {spi_rx_shift[6:0], spi1_mosi};
                    if (spi_byte_count >= HEADER_BYTES && spi_byte_count < HEADER_BYTES + 16'd336) begin
                        if (((spi_byte_count - HEADER_BYTES) & 16'd3) == 16'd1)
                            phase_stage[(spi_byte_count - HEADER_BYTES) >> 2] <= {spi_rx_shift[6:0], spi1_mosi};
                        else if (((spi_byte_count - HEADER_BYTES) & 16'd3) == 16'd2)
                            level_stage[(spi_byte_count - HEADER_BYTES) >> 2] <= {spi_rx_shift[6:0], spi1_mosi};
                        else if (((spi_byte_count - HEADER_BYTES) & 16'd3) == 16'd3)
                            enable_stage[(spi_byte_count - HEADER_BYTES) >> 2] <= {spi_rx_shift[6:0], spi1_mosi} != 8'd0;
                    end else if (spi_byte_count >= HEADER_BYTES + 16'd336 &&
                                 spi_byte_count < HEADER_BYTES + 16'd348) begin
                        rgb_stage[spi_byte_count - HEADER_BYTES - 16'd336] <= {spi_rx_shift[6:0], spi1_mosi};
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
        spi_channel_bitmap <= 88'd0;
        spi_rgb_bitmap <= 4'd0;
        spi_extension_length <= 16'd0;
        spi_frame_sequence <= 32'd0;
    end
end

always @(posedge fpga_cs_n) begin
    if (spi_byte_count == spi_expected_length) begin
        if (spi_command == 8'h10 && spi_version == 8'h01 &&
            ((spi_update_flags[0] == 1'b0) || (spi_channel_bitmap == {8'h0f, 80'hffffffffffffffffffff})) &&
            ((spi_update_flags[1] == 1'b0) || (spi_rgb_bitmap == 4'hf)) &&
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
    time_acc <= time_acc_next;
    if (time_acc_next < time_acc)
        fpga_time <= fpga_time + 1'b1;

    frame_toggle_meta <= frame_toggle_spi;
    frame_toggle_sync <= frame_toggle_meta;
    stop_toggle_meta <= stop_toggle_spi;
    stop_toggle_sync <= stop_toggle_meta;
    if (frame_toggle_sync != frame_toggle_seen) begin
        frame_toggle_seen <= frame_toggle_sync;
        apply_pending <= 1'b1;
    end
    if (stop_toggle_sync != stop_toggle_seen) begin
        stop_toggle_seen <= stop_toggle_sync;
        running <= 1'b0;
        apply_pending <= 1'b0;
        stop_toggle_seen <= stop_toggle_sync;
    end

    if (carrier_wrap) begin
        amplitude_phase <= amplitude_phase + 1'b1;
        if (apply_pending) begin
            for (i = 0; i < 84; i = i + 1) begin
                phase_active[i] <= phase_stage[i];
                level_active[i] <= level_stage[i];
                enable_active[i] <= enable_stage[i];
            end
            for (i = 0; i < 12; i = i + 1)
                rgb_active[i] <= rgb_stage[i];
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
    .g0(rgb_active[1]), .r0(rgb_active[0]), .b0(rgb_active[2]),
    .g1(rgb_active[4]), .r1(rgb_active[3]), .b1(rgb_active[5]),
    .g2(rgb_active[7]), .r2(rgb_active[6]), .b2(rgb_active[8]),
    .g3(rgb_active[10]), .r3(rgb_active[9]), .b3(rgb_active[11]),
    .data_out(rgb_data)
);

spi_mic_stream mic_stream_i (
    .cs_n(spi_mic_cs_n), .sck(spi_mic_sck), .mosi(spi_mic_mosi),
    .sample_word(mic_latest), .miso(spi_mic_miso)
);

initial begin
    spi_rx_shift = 8'd0;
    spi_bit_count = 3'd0;
    spi_byte_count = 16'd0;
    spi_command = 8'd0;
    spi_version = 8'd0;
    spi_update_flags = 16'd0;
    spi_channel_bitmap = 88'd0;
    spi_rgb_bitmap = 4'd0;
    spi_extension_length = 16'd0;
    spi_frame_sequence = 32'd0;
    spi_expected_length = HEADER_BYTES;
    accepted_sequence_spi = 32'd0;
    frame_toggle_spi = 1'b0;
    stop_toggle_spi = 1'b0;
    invalid_frame_spi = 1'b0;
    status_latched = 128'd0;
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
    time_acc = 32'd0;
    fpga_time = 32'd0;
    amplitude_phase = 8'd0;
    status_flags = 16'd0;
    mic_divider = 7'd0;
    mic_clock_reg = 1'b0;
    mic_shift_0 = 16'd0;
    mic_shift_1 = 16'd0;
    mic_sample_count = 5'd0;
    mic_latest = 32'd0;
    for (i = 0; i < 84; i = i + 1) begin
        phase_stage[i] = 8'd0;
        level_stage[i] = 8'd0;
        enable_stage[i] = 1'b0;
        phase_active[i] = 8'd0;
        level_active[i] = 8'd0;
        enable_active[i] = 1'b0;
    end
    for (i = 0; i < 12; i = i + 1) begin
        rgb_stage[i] = 8'd0;
        rgb_active[i] = 8'd0;
    end
end

endmodule
