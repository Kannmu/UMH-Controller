`timescale 1ns/1ps

/* WS2812 stream controller.
 *
 * All four LEDs on this board are wired in parallel to the same DI line, so
 * the controller only ever shifts one 24-bit {G,R,B} word.  It used to take
 * four sets of GRB inputs and load LED0's; the other three were dead logic on
 * three PIO inputs each.
 *
 * WS2812C-2020-V6 requires RES >= 280 us.  At 64 MHz we use 20000 cycles
 * (=312.5 us) so a parameter update is always accepted, independent of
 * clock start-up or STM32 reset.  Each bit cell is 80 cycles = 1.25 us:
 *   T0H = 20 cycles = 312.5 ns, T0L = 60 cycles = 937.5 ns
 *   T1H = 40 cycles = 625.0 ns, T1L = 40 cycles = 625.0 ns
 * All four values are inside the datasheet windows. */
module ws2812_stream (
    input wire clk,
    input wire enable,
    input wire [23:0] grb,        /* {G[7:0], R[7:0], B[7:0]} */
    output wire data_out,
    output reg  frame_done      /* one-cycle pulse after 24 bits */
);
    localparam [1:0] ST_RESET = 2'd0, ST_LOAD = 2'd1, ST_SEND = 2'd2;
    localparam [15:0] RESET_CYCLES = 16'd20000; /* 312.5 us at 64 MHz */
    localparam [7:0] BIT_CYCLES = 8'd79;        /* 80 cycles = 1.25 us */
    localparam [7:0] T0H_CYCLES = 8'd19;        /* 20 cycles = 312.5 ns */
    localparam [7:0] T1H_CYCLES = 8'd39;        /* 40 cycles = 625.0 ns */

    reg [1:0] state;
    reg [15:0] reset_count;
    reg [7:0] bit_cell_count;
    reg [4:0] bit_number;       /* 0-23 */
    reg [23:0] shift_register;
    reg data_out_reg;
    assign data_out = data_out_reg;

    always @(posedge clk) begin
        if (!enable) begin
            data_out_reg <= 1'b0;
            state <= ST_RESET;
            reset_count <= 16'd0;
            frame_done <= 1'b0;
        end else begin
            frame_done <= 1'b0;
            case (state)
                ST_RESET: begin
                    data_out_reg <= 1'b0;
                    if (reset_count == RESET_CYCLES) state <= ST_LOAD;
                    else reset_count <= reset_count + 16'd1;
                end
                ST_LOAD: begin
                    shift_register <= grb;
                    bit_cell_count <= 8'd0;
                    bit_number <= 5'd0;
                    data_out_reg <= 1'b1;
                    state <= ST_SEND;
                end
                ST_SEND: begin
                    if (bit_cell_count == BIT_CYCLES) begin
                        /* Bit cell complete, move to next bit */
                        bit_cell_count <= 8'd0;
                        shift_register <= {shift_register[22:0], 1'b0};
                        if (bit_number == 5'd23) begin
                            /* All 24 bits sent, go to reset */
                            reset_count <= 16'd0;
                            data_out_reg <= 1'b0;
                            state <= ST_RESET;
                            frame_done <= 1'b1;
                        end else begin
                            bit_number <= bit_number + 5'd1;
                            data_out_reg <= 1'b1;
                        end
                    end else begin
                        /* Within bit cell, generate WS2812 timing */
                        bit_cell_count <= bit_cell_count + 8'd1;
                        if (shift_register[23]) begin
                            /* Send '1': T1H=625 ns high, T1L=625 ns low */
                            if (bit_cell_count == T1H_CYCLES)
                                data_out_reg <= 1'b0;
                        end else begin
                            /* Send '0': T0H=312.5 ns high, T0L=937.5 ns low */
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
        reset_count = 16'd0;
        bit_cell_count = 8'd0;
        bit_number = 5'd0;
        shift_register = 24'd0;
        data_out_reg = 1'b0;
        frame_done = 1'b0;
    end
endmodule
