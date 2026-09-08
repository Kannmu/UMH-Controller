`timescale 1ns/1ps

module tb_umh_fpga_top;
reg fpga_clk = 1'b0;
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
reg spi_mic_mosi = 1'b0;
wire spi_mic_miso;
reg [7:0] reply [0:15];
integer failures = 0;
integer k;

umh_fpga_top dut (.*);
always #11.7647 fpga_clk = ~fpga_clk;
always #163 mic_data_0 = ~mic_data_0;
always #239 mic_data_1 = ~mic_data_1;

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

task send_status;
    integer n;
    begin
        begin_transaction;
        spi_byte(8'h01, reply[0]);
        spi_byte(8'h01, reply[1]);
        for (n = 2; n < 36; n = n + 1)
            spi_byte(8'h00, reply[n < 16 ? n : 15]);
        end_transaction;
    end
endtask

task send_full_frame;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(8'h10, reply[0]);
        spi_byte(8'h01, reply[1]);
        for (n = 0; n < 4; n = n + 1) spi_byte(8'h00, discard);
        spi_byte(8'h34, discard); spi_byte(8'h12, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 8; n = n + 1) spi_byte(8'h00, discard);
        spi_byte(8'h03, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 10; n = n + 1) spi_byte(8'hff, discard);
        spi_byte(8'h0f, discard);
        spi_byte(8'h0f, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 84; n = n + 1) begin
            spi_byte(8'h00, discard);
            spi_byte(n == 0 ? 8'h40 : 8'h00, discard);
            spi_byte(n == 0 ? 8'hff : 8'h00, discard);
            spi_byte(n == 0 ? 8'h01 : 8'h00, discard);
        end
        spi_byte(8'h11, discard); spi_byte(8'h22, discard); spi_byte(8'h33, discard);
        for (n = 0; n < 9; n = n + 1) spi_byte(8'h00, discard);
        end_transaction;
    end
endtask

initial begin
    #100;
    send_status;
    if (reply[0] !== 8'h01) begin
        $display("FAIL: protocol byte was %02x", reply[0]);
        failures = failures + 1;
    end
    send_full_frame;
    #100000;
    send_status;
    if (reply[12] !== 8'h34 || reply[13] !== 8'h12) begin
        $display("FAIL: accepted sequence was %02x%02x", reply[13], reply[12]);
        failures = failures + 1;
    end
    if (dut.running !== 1'b1 || dut.enable_active[0] !== 1'b1 || dut.phase_active[0] !== 8'h40) begin
        $display("FAIL: frame did not commit at a carrier boundary");
        failures = failures + 1;
    end
    if (us_tx[0] !== 1'b0 && us_tx[0] !== 1'b1) begin
        $display("FAIL: ultrasound output is unknown");
        failures = failures + 1;
    end
    if (failures == 0) $display("PASS: SPI status, frame commit, and output configuration verified");
    else $display("FAIL: %0d checks failed", failures);
    $finish;
end
endmodule
