`timescale 1ns/1ps

module tb_ws2812_control;
    reg clk;
    reg enable;
    reg [7:0] g, r, b;
    wire data_out;

    // Clock generation: 128 MHz (7.8125 ns period)
    initial begin
        clk = 0;
        forever #3.90625 clk = ~clk;
    end

    // Instantiate WS2812 module
    ws2812_stream dut (
        .clk(clk),
        .enable(enable),
        .g(g),
        .r(r),
        .b(b),
        .data_out(data_out)
    );

    // Test sequence
    initial begin
        $dumpfile("tb_ws2812_control.vcd");
        $dumpvars(0, tb_ws2812_control);

        // Initial state
        enable = 0;
        g = 8'd0;
        r = 8'd0;
        b = 8'd0;

        #1000;

        // Test 1: Enable with RED color
        $display("Test 1: RED (255, 0, 0)");
        r = 8'd255;
        g = 8'd0;
        b = 8'd0;
        enable = 1;
        #100000;  // Wait for one complete frame

        // Test 2: Change to GREEN
        $display("Test 2: GREEN (0, 255, 0)");
        r = 8'd0;
        g = 8'd255;
        b = 8'd0;
        #100000;

        // Test 3: Change to BLUE
        $display("Test 3: BLUE (0, 0, 255)");
        r = 8'd0;
        g = 8'd0;
        b = 8'd255;
        #100000;

        // Test 4: WHITE
        $display("Test 4: WHITE (255, 255, 255)");
        r = 8'd255;
        g = 8'd255;
        b = 8'd255;
        #100000;

        // Test 5: Disable (should turn off)
        $display("Test 5: DISABLE");
        enable = 0;
        #100000;

        // Test 6: Enable again with different color
        $display("Test 6: Re-enable with BLUE");
        r = 8'd0;
        g = 8'd0;
        b = 8'd255;
        enable = 1;
        #100000;

        $display("Simulation completed");
        $finish;
    end

    // Monitor data output transitions
    integer high_count, low_count;
    always @(posedge clk) begin
        if (data_out)
            high_count = high_count + 1;
        else
            low_count = low_count + 1;
    end

    initial begin
        high_count = 0;
        low_count = 0;
    end

endmodule
