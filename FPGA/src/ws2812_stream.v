`timescale 1ns/1ps

module ws2812_stream (
    input wire clk,
    input wire [7:0] g0, input wire [7:0] r0, input wire [7:0] b0,
    input wire [7:0] g1, input wire [7:0] r1, input wire [7:0] b1,
    input wire [7:0] g2, input wire [7:0] r2, input wire [7:0] b2,
    input wire [7:0] g3, input wire [7:0] r3, input wire [7:0] b3,
    output wire data_out
);

localparam [1:0] ST_RESET = 2'd0, ST_LOAD = 2'd1, ST_SEND = 2'd2;
localparam [12:0] RESET_CYCLES = 13'd7679;  // 60 us at 128 MHz
localparam [7:0]  BIT_CYCLES   = 8'd159;    // 1.25 us at 128 MHz

reg [1:0] state;
reg [12:0] reset_count;
reg [7:0] bit_cell_count;
reg [6:0] bit_number;
reg [95:0] shift_register;
wire [7:0] high_count = shift_register[95] ? 8'd90 : 8'd45;
assign data_out = (state == ST_SEND) && (bit_cell_count < high_count);

always @(posedge clk) begin
    case (state)
        ST_RESET: begin
            if (reset_count == RESET_CYCLES) begin
                state <= ST_LOAD;
            end else begin
                reset_count <= reset_count + 13'd1;
            end
        end
        ST_LOAD: begin
            shift_register <= {g0, r0, b0, g1, r1, b1, g2, r2, b2, g3, r3, b3};
            bit_cell_count <= 8'd0;
            bit_number <= 7'd0;
            state <= ST_SEND;
        end
        ST_SEND: begin
            if (bit_cell_count == BIT_CYCLES) begin
                bit_cell_count <= 8'd0;
                shift_register <= {shift_register[94:0], 1'b0};
                if (bit_number == 7'd95) begin
                    reset_count <= 13'd0;
                    state <= ST_RESET;
                end else begin
                    bit_number <= bit_number + 7'd1;
                end
            end else begin
                bit_cell_count <= bit_cell_count + 8'd1;
            end
        end
        default: state <= ST_RESET;
    endcase
end

initial begin
    state = ST_RESET;
    reset_count = 13'd0;
    bit_cell_count = 8'd0;
    bit_number = 7'd0;
    shift_register = 96'd0;
end
endmodule
