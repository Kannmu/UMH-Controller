`timescale 1ns/1ps

module tb_fpga_ws2812_link;
    reg fpga_clk_8m;
    reg fpga_cs_n;
    reg spi1_sck;
    reg spi1_mosi;
    wire spi1_miso;
    wire [83:0] us_tx;
    wire rgb_data;
    wire mic_clk;
    reg mic_data_0, mic_data_1;
    reg spi_mic_cs_n, spi_mic_sck;
    wire spi_mic_miso;

    // 8 MHz clock generation (125 ns period)
    initial begin
        fpga_clk_8m = 0;
        forever #62.5 fpga_clk_8m = ~fpga_clk_8m;
    end

    // SPI clock generation: 42.5 MHz max, use 20 MHz (50 ns period)
    initial begin
        spi1_sck = 0;
        forever #25 spi1_sck = ~spi1_sck;
    end

    // Instantiate FPGA top module
    umh_fpga_top dut (
        .fpga_clk_8m(fpga_clk_8m),
        .fpga_cs_n(fpga_cs_n),
        .spi1_sck(spi1_sck),
        .spi1_mosi(spi1_mosi),
        .spi1_miso(spi1_miso),
        .us_tx(us_tx),
        .rgb_data(rgb_data),
        .mic_clk(mic_clk),
        .mic_data_0(mic_data_0),
        .mic_data_1(mic_data_1),
        .spi_mic_cs_n(spi_mic_cs_n),
        .spi_mic_sck(spi_mic_sck),
        .spi_mic_miso(spi_mic_miso)
    );

    // SPI byte transmission task
    task spi_send_byte;
        input [7:0] data;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                @(negedge spi1_sck);
                spi1_mosi = data[i];
            end
        end
    endtask

    // SPI WS2812 command transmission
    task send_ws2812_command;
        input [7:0] r, g, b;
        integer i;
        begin
            fpga_cs_n = 1;
            #1000;
            fpga_cs_n = 0;
            #100;

            // Header
            spi_send_byte(8'h13);  // Command: WS2812
            spi_send_byte(8'h01);  // Version
            spi_send_byte(8'h00);  // Transaction sequence[0]
            spi_send_byte(8'h00);  // Transaction sequence[1]
            spi_send_byte(8'h00);  // Transaction sequence[2]
            spi_send_byte(8'h00);  // Transaction sequence[3]
            spi_send_byte(8'h00);  // Frame sequence[0]
            spi_send_byte(8'h00);  // Frame sequence[1]
            spi_send_byte(8'h00);  // Frame sequence[2]
            spi_send_byte(8'h00);  // Frame sequence[3]
            spi_send_byte(8'h00);  // Deadline[0-7]
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            spi_send_byte(8'h02);  // Update flags[0] = 0x02 (RGB only)
            spi_send_byte(8'h00);  // Update flags[1]
            // Bitmap (11 bytes of 0x00)
            for (i = 0; i < 11; i = i + 1) begin
                spi_send_byte(8'h00);
            end
            spi_send_byte(8'h00);  // RGB bitmap
            spi_send_byte(8'h00);  // Digital mask
            spi_send_byte(8'h00);  // Digital state
            spi_send_byte(8'h00);  // Extension length[0]
            spi_send_byte(8'h00);  // Extension length[1]

            // RGB data: 4 LEDs x 3 bytes
            spi_send_byte(r);  // LED 0 R
            spi_send_byte(g);  // LED 0 G
            spi_send_byte(b);  // LED 0 B
            spi_send_byte(r);  // LED 1 R
            spi_send_byte(g);  // LED 1 G
            spi_send_byte(b);  // LED 1 B
            spi_send_byte(r);  // LED 2 R
            spi_send_byte(g);  // LED 2 G
            spi_send_byte(b);  // LED 2 B
            spi_send_byte(r);  // LED 3 R
            spi_send_byte(g);  // LED 3 G
            spi_send_byte(b);  // LED 3 B

            #1000;
            fpga_cs_n = 1;
            #5000;
        end
    endtask

    // STOP command
    task send_stop_command;
        integer i;
        begin
            fpga_cs_n = 1;
            #1000;
            fpga_cs_n = 0;
            #100;

            spi_send_byte(8'h11);  // Command: STOP
            spi_send_byte(8'h01);  // Version
            // Transaction sequence
            for (i = 0; i < 4; i = i + 1) spi_send_byte(8'h00);
            // Frame sequence
            for (i = 0; i < 4; i = i + 1) spi_send_byte(8'h00);
            // Deadline
            for (i = 0; i < 8; i = i + 1) spi_send_byte(8'h00);
            // Update flags
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            // Bitmap (11 bytes)
            for (i = 0; i < 11; i = i + 1) spi_send_byte(8'h00);
            // RGB bitmap, digital mask, digital state
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);
            // Extension length
            spi_send_byte(8'h00);
            spi_send_byte(8'h00);

            #1000;
            fpga_cs_n = 1;
            #5000;
        end
    endtask

    // Test sequence
    initial begin
        $dumpfile("tb_fpga_ws2812_link.vcd");
        $dumpvars(0, tb_fpga_ws2812_link);
        $dumpvars(0, dut.ws2812_enable);
        $dumpvars(0, dut.rgb_hold);

        // Initialize
        fpga_cs_n = 1;
        spi1_mosi = 0;
        mic_data_0 = 0;
        mic_data_1 = 0;
        spi_mic_cs_n = 1;
        spi_mic_sck = 0;

        // Wait for PLL lock
        #10000;

        $display("Test 1: Send RED command");
        send_ws2812_command(8'd255, 8'd0, 8'd0);
        #200000;

        $display("Test 2: Send GREEN command");
        send_ws2812_command(8'd0, 8'd255, 8'd0);
        #200000;

        $display("Test 3: Send BLUE command");
        send_ws2812_command(8'd0, 8'd0, 8'd255);
        #200000;

        $display("Test 4: Send WHITE command");
        send_ws2812_command(8'd255, 8'd255, 8'd255);
        #200000;

        $display("Test 5: Send STOP command (should turn off LEDs)");
        send_stop_command();
        #200000;

        $display("Test 6: Send RED command again");
        send_ws2812_command(8'd255, 8'd0, 8'd0);
        #200000;

        $display("Simulation completed successfully");
        $finish;
    end

    // Monitor WS2812 enable and RGB values
    always @(posedge dut.fpga_clk) begin
        if (dut.ws2812_enable !== dut.ws2812_enable) begin
            $display("Time %t: WS2812 enable changed to %b", $time, dut.ws2812_enable);
        end
    end

    // Timeout watchdog
    initial begin
        #5000000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule
