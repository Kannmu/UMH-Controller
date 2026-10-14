`timescale 1ns/1ps

module EHXPLLJ #(
    parameter PLLRST_ENA="DISABLED", parameter INTFB_WAKE="DISABLED",
    parameter STDBY_ENABLE="DISABLED", parameter DPHASE_SOURCE="DISABLED",
    parameter CLKOP_FPHASE=0, parameter CLKOP_CPHASE=0,
    parameter OUTDIVIDER_MUXA2="DIVA", parameter CLKOP_ENABLE="ENABLED",
    parameter CLKOP_DIV=1, parameter CLKFB_DIV=1, parameter CLKI_DIV=1,
    parameter FEEDBK_PATH="INT_DIVA", parameter PLL_LOCK_MODE=0
) (
    input CLKI, input CLKFB, input RST, input RESETM, input RESETC, input RESETD,
    input PHASESEL0, input PHASESEL1, input PHASEDIR, input PHASESTEP,
    input LOADREG, input STDBY, input PLLWAKESYNC, input ENCLKOP,
    output CLKOP, output LOCK, output CLKINTFB
);
    assign CLKOP = CLKI;
    assign CLKINTFB = CLKFB;
    assign LOCK = 1'b1;
endmodule

module tb_mic_audio;
    reg clk64 = 0;
    always #7.8 clk64 = ~clk64;

    reg cs_n = 1;
    reg sck = 0;
    reg mosi = 0;
    wire miso;
    wire [83:0] us_tx;
    wire rgb_data;
    wire mic_clk;
    reg mic_data_0 = 1'b1;
    reg mic_data_1 = 1'b1;
    wire spi_mic_miso;

    umh_fpga_top dut (
        .fpga_clk_8m(clk64), .fpga_cs_n(cs_n), .spi1_sck(sck), .spi1_mosi(mosi),
        .spi1_miso(miso), .us_tx(us_tx), .rgb_data(rgb_data), .mic_clk(mic_clk),
        .mic_data_0(mic_data_0), .mic_data_1(mic_data_1),
        .spi_mic_cs_n(1'b1), .spi_mic_sck(1'b0), .spi_mic_miso(spi_mic_miso)
    );

    task spi_byte(input [7:0] b, output [7:0] r);
        integer bit_index;
        begin
            for (bit_index = 7; bit_index >= 0; bit_index = bit_index - 1) begin
                mosi = b[bit_index]; #20;
                sck = 1; #20; r[bit_index] = miso; sck = 0; #20;
            end
        end
    endtask

    task spi_start; begin cs_n = 0; #40; end endtask
    task spi_stop;  begin #40; cs_n = 1; #40; end endtask

    reg [7:0] rx [0:48];
    reg [7:0] audio_count;
    reg signed [15:0] pcm0;
    reg signed [15:0] pcm1;
    integer i;
    integer failures = 0;

    task read_audio;
        integer k;
        reg [7:0] tx_byte;
        begin
            spi_start;
            for (k = 0; k < 49; k = k + 1) begin
                case (k)
                    0: tx_byte = 8'h16;
                    1: tx_byte = 8'h01;
                    2: tx_byte = 8'h10;
                    default: tx_byte = 8'h00;
                endcase
                spi_byte(tx_byte, rx[k]);
            end
            spi_stop;
            audio_count = rx[16];
            pcm0 = $signed({rx[17], rx[18]});
            pcm1 = $signed({rx[19], rx[20]});
        end
    endtask

    initial begin
        #200;
        /* Exercise the real PDM-density PCM decimator without waiting for the
         * 30 ms microphone wake sequence.  Both data pins high represents a
         * positive full-scale density deviation on all four MEMS channels. */
        dut.mic_warm = 1'b0;
        dut.mic_ultrasonic = 1'b1;
        dut.mic_phase = 4'd0;
        dut.pcm_decim_count = 9'd0;
        dut.pcm_ones[0] = 10'd0; dut.pcm_ones[1] = 10'd0;
        dut.pcm_ones[2] = 10'd0; dut.pcm_ones[3] = 10'd0;
        dut.pcm_wr_ptr = 6'd0; dut.pcm_rd_ptr_spi = 6'd0;
        dut.pcm_rd_sync1 = 6'd0; dut.pcm_rd_sync2 = 6'd0;
        dut.pcm_wr_sync1_spi = 6'd0; dut.pcm_wr_sync2_spi = 6'd0;
        dut.pcm_read_count_latched_spi = 5'd0;

        wait (dut.pcm_wr_ptr == 6'd2);
        #100;
        read_audio;
        if (audio_count != 8'd2) begin
            $display("FAIL: audio count expected 2, received %0d", audio_count);
            failures = failures + 1;
        end else $display("PASS: audio FIFO read returned 2 samples");

        /* 1000 PDM ones over four mics -> (1000 - 500) << 6 = 32000;
         * allow a tiny one-bit edge difference from the first partial window. */
        if (pcm0 < 16'sd30000 || pcm0 > 16'sd32767 ||
            pcm1 < 16'sd30000 || pcm1 > 16'sd32767) begin
            $display("FAIL: PCM sign/scale wrong: %0d, %0d", pcm0, pcm1);
            failures = failures + 1;
        end else $display("PASS: PCM signed scale %0d, %0d", pcm0, pcm1);

        read_audio;
        if (audio_count != 8'd0) begin
            $display("FAIL: consumed samples remained in FIFO (%0d)", audio_count);
            failures = failures + 1;
        end else $display("PASS: read transaction advanced the FIFO pointer");

        mic_data_0 = 1'b0;
        mic_data_1 = 1'b0;
        wait (dut.pcm_wr_ptr == 6'd4);
        #100;
        read_audio;
        if (audio_count != 8'd2 || pcm1 > -16'sd30000) begin
            $display("FAIL: negative PCM sign/scale count=%0d samples=%0d, %0d",
                     audio_count, pcm0, pcm1);
            failures = failures + 1;
        end else $display("PASS: negative PCM sign/scale %0d, %0d", pcm0, pcm1);

        if (failures == 0) $display("RESULT: PASS");
        else $display("RESULT: %0d FAILURES", failures);
        $finish;
    end
endmodule
