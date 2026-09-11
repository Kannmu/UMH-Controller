`timescale 1ns/1ps

/* WS2812 stream controller for 4 LEDs in parallel.
 * All 4 LEDs are connected to the same DI line (parallel configuration).
 * Takes 1 set of GRB data and sends it to all LEDs simultaneously.
 * At 128 MHz, each bit cell is 1.25 us (160 cycles):
 *   T0H: ~0.35 us (45 cycles), T0L: ~0.9 us (115 cycles)
 *   T1H: ~0.7 us (90 cycles), T1L: ~0.55 us (70 cycles)
 * Reset: >50 us (6400 cycles at 128 MHz) */
module ws2812_stream (
    input wire clk,
    input wire enable,
    input wire [7:0] g0, input wire [7:0] r0, input wire [7:0] b0,  /* LED 0 */
    input wire [7:0] g1, input wire [7:0] r1, input wire [7:0] b1,  /* LED 1 (unused in parallel mode) */
    input wire [7:0] g2, input wire [7:0] r2, input wire [7:0] b2,  /* LED 2 (unused in parallel mode) */
    input wire [7:0] g3, input wire [7:0] r3, input wire [7:0] b3,  /* LED 3 (unused in parallel mode) */
    output wire data_out
);
    localparam [1:0] ST_RESET = 2'd0, ST_LOAD = 2'd1, ST_SEND = 2'd2;
    localparam [12:0] RESET_CYCLES = 13'd7679;  /* ~60 us at 128 MHz */
    localparam [7:0] BIT_CYCLES = 8'd159;       /* 1.25 us per bit */
    localparam [7:0] T0H_CYCLES = 8'd44;        /* 0.35 us */
    localparam [7:0] T1H_CYCLES = 8'd89;        /* 0.70 us */

    reg [1:0] state;
    reg [12:0] reset_count;
    reg [7:0] bit_cell_count;
    reg [4:0] bit_number;       /* 0-23 for 1 LED × 24 bits (parallel mode) */
    reg [23:0] shift_register;  /* 1 LED × 24 bits (all LEDs receive same data) */
    reg data_out_reg;
    assign data_out = data_out_reg;

    always @(posedge clk) begin
        if (!enable) begin
            data_out_reg <= 1'b0;
            state <= ST_RESET;
            reset_count <= 13'd0;
        end else begin
            case (state)
                ST_RESET: begin
                    data_out_reg <= 1'b0;
                    if (reset_count == RESET_CYCLES) state <= ST_LOAD;
                    else reset_count <= reset_count + 13'd1;
                end
                ST_LOAD: begin
                    /* Load only LED0 data - all parallel LEDs receive the same data */
                    shift_register <= {g0, r0, b0};
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
                            reset_count <= 13'd0;
                            data_out_reg <= 1'b0;
                            state <= ST_RESET;
                        end else begin
                            bit_number <= bit_number + 5'd1;
                            data_out_reg <= 1'b1;
                        end
                    end else begin
                        /* Within bit cell, generate WS2812 timing */
                        bit_cell_count <= bit_cell_count + 8'd1;
                        if (shift_register[23]) begin
                            /* Send '1': T1H=0.7us high, T1L=0.55us low */
                            if (bit_cell_count == T1H_CYCLES)
                                data_out_reg <= 1'b0;
                        end else begin
                            /* Send '0': T0H=0.35us high, T0L=0.9us low */
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
        reset_count = 13'd0;
        bit_cell_count = 8'd0;
        bit_number = 5'd0;
        shift_register = 24'd0;
        data_out_reg = 1'b0;
    end
endmodule
