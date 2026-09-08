`timescale 1ns/1ps

module ws2812_stream (
    input wire clk,
    input wire [7:0] g0, input wire [7:0] r0, input wire [7:0] b0,
    input wire [7:0] g1, input wire [7:0] r1, input wire [7:0] b1,
    input wire [7:0] g2, input wire [7:0] r2, input wire [7:0] b2,
    input wire [7:0] g3, input wire [7:0] r3, input wire [7:0] b3,
    output wire data_out
);

// At 42.5 MHz, a 53-cycle WS2812 bit cell is 1.247 us.  The 60 us low period
// after the 96-bit chain exceeds the reset requirement.
reg [12:0] reset_count;
reg [5:0] bit_cell_count;
reg [6:0] bit_number;
reg [95:0] shift_register;
wire transmitting = reset_count == 13'd2550;
wire [5:0] high_count = shift_register[95] ? 6'd30 : 6'd15;
assign data_out = transmitting && (bit_cell_count < high_count);

always @(posedge clk) begin
    if (!transmitting) begin
        if (reset_count == 13'd2549) begin
            reset_count <= 13'd2550;
            bit_cell_count <= 6'd0;
            bit_number <= 7'd0;
            shift_register <= {g0, r0, b0, g1, r1, b1, g2, r2, b2, g3, r3, b3};
        end else begin
            reset_count <= reset_count + 1'b1;
        end
    end else if (bit_cell_count == 6'd52) begin
        bit_cell_count <= 6'd0;
        shift_register <= {shift_register[94:0], 1'b0};
        if (bit_number == 7'd95) begin
            bit_number <= 7'd0;
            reset_count <= 13'd0;
        end else begin
            bit_number <= bit_number + 1'b1;
        end
    end else begin
        bit_cell_count <= bit_cell_count + 1'b1;
    end
end

initial begin
    reset_count = 13'd0;
    bit_cell_count = 6'd0;
    bit_number = 7'd0;
    shift_register = 96'd0;
end
endmodule
