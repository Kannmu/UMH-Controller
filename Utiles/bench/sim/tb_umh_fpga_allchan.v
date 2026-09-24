`timescale 1ns/1ps

module tb_umh_fpga_allchan;
reg fpga_clk_8m = 1'b0;
reg fpga_cs_n = 1'b1;
reg spi1_sck = 1'b0;
reg spi1_mosi = 1'b0;
wire spi1_miso;
wire [83:0] us_tx;
wire rgb_data;
wire mic_clk;
reg mic_data_0 = 1'b0;
reg mic_data_1 = 1'b0;
reg spi_mic_cs_n = 1'b1;
reg spi_mic_sck = 1'b0;
wire spi_mic_miso;
integer failures = 0;
integer n;

umh_fpga_top dut (.*);
always #62.5 fpga_clk_8m = ~fpga_clk_8m;

task spi_byte;
    input [7:0] value;
    output [7:0] received;
    integer bit_index;
    begin
        received = 8'd0;
        for (bit_index = 7; bit_index >= 0; bit_index = bit_index - 1) begin
            spi1_mosi = value[bit_index];
            #30 spi1_sck = 1'b1;
            #5 received[bit_index] = spi1_miso;
            #25 spi1_sck = 1'b0;
        end
    end
endtask

task begin_transaction;
    begin
        spi1_mosi = 1'b0;
        #50 fpga_cs_n = 1'b0;
        #50;
    end
endtask

task end_transaction;
    begin
        #50 fpga_cs_n = 1'b1;
        #100;
    end
endtask

task send_full_frame_marker;
    input enable_marker;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(8'h10, discard);
        spi_byte(8'h01, discard);
        for (n = 0; n < 4; n = n + 1) spi_byte(8'h00, discard);
        spi_byte(8'h01, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 8; n = n + 1) spi_byte(8'h00, discard);
        spi_byte(8'h03, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 10; n = n + 1) spi_byte(8'hff, discard);
        spi_byte(8'h0f, discard);
        spi_byte(8'h0f, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 84; n = n + 1) begin
            spi_byte(n[7:0], discard);
            spi_byte(enable_marker ? 8'h80 : 8'h00, discard);
        end
        spi_byte(8'h00, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 9; n = n + 1) spi_byte(8'h00, discard);
        end_transaction;
    end
endtask

task send_compact;
    input [7:0] command;
    input [7:0] data;
    input [31:0] seq_value;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(command, discard);
        spi_byte(8'h01, discard);
        spi_byte(data, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        spi_byte(seq_value[7:0], discard);
        spi_byte(seq_value[15:8], discard);
        spi_byte(seq_value[23:16], discard);
        spi_byte(seq_value[31:24], discard);
        for (n = 0; n < 6; n = n + 1) spi_byte(8'h00, discard);
        end_transaction;
    end
endtask

task send_short_level;
    input [7:0] data;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(8'h19, discard);
        spi_byte(8'h01, discard);
        spi_byte(data, discard);
        end_transaction;
    end
endtask

initial begin
    #1000;
    /* Firmware order: silent phase load, enter audio mode, then load the
     * real enable markers while the common level is still zero. */
    send_full_frame_marker(1'b0);
    #200000;
    if (dut.running !== 1'b1) begin
        $display("FAIL: full frame did not start the output engine");
        failures = failures + 1;
    end
    send_compact(8'h17, 8'd1, 32'd1);
    #100000;
    if (dut.audio_mode !== 1'b1) begin
        $display("FAIL: audio mode was not entered");
        failures = failures + 1;
    end
    send_full_frame_marker(1'b1);
    #200000;
    send_compact(8'h16, 8'd64, 32'd2);
    #200000;
    if (dut.pending_audio_level !== 8'd64) begin
        $display("FAIL: audio level was not adopted: %0d", dut.pending_audio_level);
        failures = failures + 1;
    end
    if (dut.event_ram.mem[dut.active_bank ? (256 + 8'h40) : 8'h40][0] !== 1'b1 ||
        dut.event_ram.mem[dut.active_bank ? (256 + 8'h80) : 8'h80][0] !== 1'b1) begin
        $display("FAIL: event table did not encode phase=0x40 level=0x40");
        failures = failures + 1;
    end
    /* 3-byte short hot-path command used by the STM32 at 20 kHz. */
    send_short_level(8'd32);
    #200000;
    if (dut.pending_audio_level !== 8'd32) begin
        $display("FAIL: short audio level was not adopted: %0d", dut.pending_audio_level);
        failures = failures + 1;
    end
    if (dut.event_ram.mem[dut.active_bank ? (256 + 8'h60) : 8'h60][0] !== 1'b1) begin
        $display("FAIL: short command did not rebuild the event table");
        failures = failures + 1;
    end
    send_compact(8'h17, 8'd0, 32'd3);
    #100000;
    if (dut.audio_mode !== 1'b0) begin
        $display("FAIL: audio mode did not exit");
        failures = failures + 1;
    end
    if (failures == 0) $display("PASS: compact focused-AM mode, level update, and event-table rebuild verified");
    else $display("FAIL: %0d checks failed", failures);
    $finish;
end
endmodule

