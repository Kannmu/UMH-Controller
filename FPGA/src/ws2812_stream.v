`timescale 1ns/1ps

module ws2812_stream (
    input wire clk,
    input wire enable,
    input wire [23:0] grb,
    output wire data_out,
    output reg  frame_done
);
    localparam [1:0] ST_RESET = 2'd0, ST_LOAD = 2'd1, ST_SEND = 2'd2;

    /* RESET must stay >= 280 us (WS2812C-2020-V6): 20000 cycles = 312.5 us at 64 MHz. */
    localparam [14:0] RESET_CYCLES = 15'd20000;
    localparam [6:0] BIT_CYCLES = 7'd79;
    localparam [6:0] T0H_CYCLES = 7'd19;
    localparam [6:0] T1H_CYCLES = 7'd39;

    reg [1:0] state;
    reg [14:0] reset_count;
    reg [6:0] bit_cell_count;
    reg [4:0] bit_number;
    reg [23:0] shift_register;
    reg data_out_reg;
    assign data_out = data_out_reg;

    always @(posedge clk) begin
        if (!enable) begin
            data_out_reg <= 1'b0;
            state <= ST_RESET;
            reset_count <= 15'd0;
            frame_done <= 1'b0;
        end else begin
            frame_done <= 1'b0;
            case (state)
                ST_RESET: begin
                    data_out_reg <= 1'b0;
                    if (reset_count == RESET_CYCLES) state <= ST_LOAD;
                    else reset_count <= reset_count + 15'd1;
                end
                ST_LOAD: begin
                    shift_register <= grb;
                    bit_cell_count <= 7'd0;
                    bit_number <= 5'd0;
                    data_out_reg <= 1'b1;
                    state <= ST_SEND;
                end
                ST_SEND: begin
                    if (bit_cell_count == BIT_CYCLES) begin

                        bit_cell_count <= 7'd0;
                        shift_register <= {shift_register[22:0], 1'b0};
                        if (bit_number == 5'd23) begin

                            reset_count <= 15'd0;
                            data_out_reg <= 1'b0;
                            state <= ST_RESET;
                            frame_done <= 1'b1;
                        end else begin
                            bit_number <= bit_number + 5'd1;
                            data_out_reg <= 1'b1;
                        end
                    end else begin

                        bit_cell_count <= bit_cell_count + 7'd1;
                        if (shift_register[23]) begin

                            if (bit_cell_count == T1H_CYCLES)
                                data_out_reg <= 1'b0;
                        end else begin

                            if (bit_cell_count == T0H_CYCLES)
                                data_out_reg <= 1'b0;
                        end
                    end
                end
                default: state <= ST_RESET;
            endcase
        end
    end

    initial begin
        state = ST_RESET;
        reset_count = 15'd0;
        bit_cell_count = 7'd0;
        bit_number = 5'd0;
        shift_register = 24'd0;
        data_out_reg = 1'b0;
        frame_done = 1'b0;
    end
endmodule
