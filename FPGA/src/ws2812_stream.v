`timescale 1ns/1ps

module ws2812_stream (
    input wire clk,
    input wire [7:0] g, input wire [7:0] r, input wire [7:0] b,
    output wire data_out
);
    localparam [1:0] ST_RESET = 2'd0, ST_LOAD = 2'd1, ST_SEND = 2'd2;
    localparam [12:0] RESET_CYCLES = 13'd7679;
    localparam [7:0] BIT_CYCLES = 8'd159;
    reg [1:0] state;
    reg [12:0] reset_count;
    reg [7:0] bit_cell_count;
    reg [4:0] bit_number;
    reg [23:0] shift_register;
    reg data_out_reg;
    assign data_out = data_out_reg;

    always @(posedge clk) begin
        case (state)
            ST_RESET: begin
                data_out_reg <= 1'b0;
                if (reset_count == RESET_CYCLES) state <= ST_LOAD;
                else reset_count <= reset_count + 13'd1;
            end
            ST_LOAD: begin
                shift_register <= {g, r, b};
                bit_cell_count <= 8'd0;
                bit_number <= 5'd0;
                data_out_reg <= 1'b1;
                state <= ST_SEND;
            end
            ST_SEND: begin
                if (bit_cell_count == BIT_CYCLES) begin
                    bit_cell_count <= 8'd0;
                    shift_register <= {shift_register[22:0], 1'b0};
                    if (bit_number == 5'd23) begin
                        reset_count <= 13'd0;
                        data_out_reg <= 1'b0;
                        state <= ST_RESET;
                    end else begin
                        bit_number <= bit_number + 5'd1;
                        data_out_reg <= 1'b1;
                    end
                end else begin
                    bit_cell_count <= bit_cell_count + 8'd1;
                    if (shift_register[23] && bit_cell_count == 8'd89)
                        data_out_reg <= 1'b0;
                    else if (!shift_register[23] && bit_cell_count == 8'd44)
                        data_out_reg <= 1'b0;
                end
            end
            default: state <= ST_RESET;
        endcase
    end

    initial begin
        state = ST_RESET;
        reset_count = 13'd0;
        bit_cell_count = 8'd0;
        bit_number = 5'd0;
        shift_register = 24'd0;
        data_out_reg = 1'b0;
    end
endmodule
