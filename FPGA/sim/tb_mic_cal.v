`timescale 1ns/1ps

/* Integration bench for the UMH v7 microphone calibration path.
 *
 * Drives the real SPI slave protocol (MIC_CONFIG 0x14, a full 84-channel
 * FRAME 0x10 and MIC_READ 0x15) with a 40 kHz square wave on the PDM pins,
 * and checks that one completed gate block is visible through MIC_READ. */
module EHXPLLJ #(
    parameter PLLRST_ENA="DISABLED", parameter INTFB_WAKE="DISABLED",
    parameter STDBY_ENABLE="DISABLED", parameter DPHASE_SOURCE="DISABLED",
    parameter CLKOP_FPHASE=0, parameter CLKOP_CPHASE=0,
    parameter OUTDIVIDER_MUXA2="DIVA", parameter CLKOP_ENABLE="ENABLED",
    parameter CLKOP_DIV=1, parameter CLKFB_DIV=1, parameter CLKI_DIV=1,
    parameter FEEDBK_PATH="INT_DIVA"
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

module tb_mic_cal;
    reg clk64 = 0;
    always #7.8 clk64 = ~clk64;            /* ~64 MHz; PLL stub passes it */

    reg         cs_n = 1;
    reg         sck = 0;
    reg         mosi = 0;
    wire        miso;
    wire [83:0] us_tx;
    wire        rgb_data;
    wire        mic_clk;
    reg         mic_data_0 = 0;
    reg         mic_data_1 = 0;
    wire        spi_mic_miso;

    umh_fpga_top dut (
        .fpga_clk_8m(clk64), .fpga_cs_n(cs_n), .spi1_sck(sck), .spi1_mosi(mosi),
        .spi1_miso(miso), .us_tx(us_tx), .rgb_data(rgb_data), .mic_clk(mic_clk),
        .mic_data_0(mic_data_0), .mic_data_1(mic_data_1),
        .spi_mic_cs_n(1'b1), .spi_mic_sck(1'b0), .spi_mic_miso(spi_mic_miso)
    );

    /* 40 kHz square wave sampled at 4 MHz on both PDM data pins. */
    reg [3:0] sub = 0;
    reg [6:0] sig_n = 0;
    always @(posedge clk64) begin
        if (sub == 4'd15) begin
            sub <= 4'd0;
            sig_n <= (sig_n == 7'd99) ? 7'd0 : sig_n + 7'd1;
        end else sub <= sub + 4'd1;
    end
    always @* begin
        mic_data_0 = (sig_n < 50);
        mic_data_1 = (sig_n < 50);
    end

    task spi_byte(input [7:0] b, output [7:0] r);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                mosi = b[i]; #20;
                sck = 1; #20; r[i] = miso; sck = 0; #20;
            end
        end
    endtask
    task spi_start; begin cs_n = 0; #40; end endtask
    task spi_stop;  begin #40; cs_n = 1; #40; end endtask

    reg [7:0] resp [0:63];
    integer   failures = 0;

    task send_frame_all_zero;
        integer i; reg [7:0] rb;
        begin
            spi_start;
            spi_byte(8'h10, rb); spi_byte(8'h01, rb);
            spi_byte(8'h01, rb); spi_byte(8'h00, rb);
            spi_byte(8'h00, rb); spi_byte(8'h00, rb);
            spi_byte(8'h00, rb); spi_byte(8'h00, rb);
            spi_byte(8'h00, rb); spi_byte(8'h00, rb);
            for (i = 0; i < 8; i = i + 1) spi_byte(8'h00, rb);
            spi_byte(8'h01, rb); spi_byte(8'h00, rb);       /* ULTRASOUND */
            for (i = 0; i < 11; i = i + 1) spi_byte(8'hFF, rb);
            spi_byte(8'h00, rb); spi_byte(8'h00, rb); spi_byte(8'h00, rb);
            spi_byte(8'h00, rb); spi_byte(8'h00, rb);
            for (i = 0; i < 84; i = i + 1) begin
                spi_byte(8'h00, rb);                          /* phase 0 */
                spi_byte(8'h80, rb);                          /* level 128 */
            end
            spi_stop;
        end
    endtask

    task send_mic_config(input [7:0] count, input [15:0] start,
                         input [15:0] step, input [7:0] width);
        integer i; reg [7:0] rb;
        begin
            spi_start;
            spi_byte(8'h14, rb); spi_byte(8'h01, rb);
            for (i = 0; i < 32; i = i + 1) spi_byte(8'h00, rb);
            spi_byte(8'h06, rb); spi_byte(8'h00, rb);
            spi_byte(count, rb);
            spi_byte(start[7:0], rb); spi_byte(start[15:8], rb);
            spi_byte(step[7:0], rb);  spi_byte(step[15:8], rb);
            spi_byte(width, rb);
            spi_stop;
        end
    endtask

    reg [7:0] tx [0:39];
    task send_mic_read(input [7:0] gate);
        integer i;
        begin
            for (i = 0; i < 40; i = i + 1) tx[i] = 8'h00;
            tx[0] = 8'h15; tx[1] = 8'h01; tx[2] = 8'h55; tx[6] = gate;
            spi_start;
            for (i = 0; i < 40; i = i + 1) spi_byte(tx[i], resp[i]);
            spi_stop;
        end
    endtask

    integer wait_cnt;
    integer mic;
    reg signed [15:0] ival, qval;
    initial begin
        #200;
        /* Skip the 30 ms boot/warm ramp for simulation; the real sequencer is
         * exercised by its own counter test. */
        dut.mic_boot_count = 32'd640_000;
        dut.mic_warm        = 1'b0;
        dut.mic_ultrasonic  = 1'b1;
        dut.mic_phase       = 4'd0;
        dut.mic_lo_n        = 7'd99;
        dut.mic_window_count= 7'd0;
        dut.mic_xor_i[0] = 7'd0; dut.mic_xor_i[1] = 7'd0;
        dut.mic_xor_i[2] = 7'd0; dut.mic_xor_i[3] = 7'd0;
        dut.mic_xor_q[0] = 7'd0; dut.mic_xor_q[1] = 7'd0;
        dut.mic_xor_q[2] = 7'd0; dut.mic_xor_q[3] = 7'd0;
        #2000;

        send_mic_config(8'd2, 16'd0, 16'd20, 8'd4);
        #2000;
        if (dut.mic_run_state !== 2'd1) begin
            $display("FAIL: MIC_CONFIG did not arm sequencer state=%0d", dut.mic_run_state);
            failures = failures + 1;
        end else $display("PASS: MIC_CONFIG armed sequencer");

        send_frame_all_zero;
        wait_cnt = 0;
        while (dut.mic_block_count == 0 && wait_cnt < 200000) begin
            @(posedge clk64); wait_cnt = wait_cnt + 1;
        end
        if (dut.mic_block_count == 0) begin
            $display("FAIL: no microphone block completed");
            failures = failures + 1;
        end else if (dut.mic_block_count != 1 || dut.mic_cfg_count != 2) begin
            $display("FAIL: unexpected block_count=%0d gate_count=%0d",
                     dut.mic_block_count, dut.mic_cfg_count);
            failures = failures + 1;
        end else $display("PASS: block_count=1 gate_count=2");

        /* Gate 0 payload. */
        send_mic_read(8'd0);
        if (resp[16] != 8'h00 || resp[17] != 8'h13) begin
            $display("FAIL: MIC_READ status=%02x%02x block=%02x%02x gates=%02x%02x",
                     resp[16], resp[17], resp[18], resp[19], resp[20], resp[21]);
            failures = failures + 1;
        end
        if (resp[18] != 8'h00 || resp[19] != 8'h01 ||
            resp[20] != 8'h00 || resp[21] != 8'h02) begin
            $display("FAIL: MIC_READ metadata mismatch: block=%02x%02x gates=%02x%02x", resp[18],resp[19],resp[20],resp[21]);
            failures = failures + 1;
        end
        for (mic = 0; mic < 4; mic = mic + 1) begin
            ival = {resp[24 + mic * 4], resp[25 + mic * 4]};
            qval = {resp[26 + mic * 4], resp[27 + mic * 4]};
            $display("MIC%0d I=%0d Q=%0d", mic, ival, qval);
            if (ival > 16'sd512 || ival < -16'sd512 ||
                qval > 16'sd512 || qval < -16'sd512) begin
                $display("FAIL: gate value out of range");
                failures = failures + 1;
            end
        end
        /* Submit a second pattern without reconfiguring: the sequencer must
         * auto re-arm and increment block_count to 2. */
        send_frame_all_zero;
        wait_cnt = 0;
        while (dut.mic_block_count < 2 && wait_cnt < 200000) begin
            @(posedge clk64); wait_cnt = wait_cnt + 1;
        end
        send_mic_read(8'd0);
        if (dut.mic_block_count != 2 || resp[19] != 8'h02) begin
            $display("FAIL: auto re-arm block_count=%0d meta=%02x%02x", dut.mic_block_count, resp[18], resp[19]);
            failures = failures + 1;
        end else $display("PASS: auto re-arm block_count=2");
        if (failures == 0) $display("RESULT: PASS");
        else $display("RESULT: %0d FAILURES", failures);
        $finish;
    end
endmodule
