`timescale 1ns/1ps

// CH347T is the SPI master.  Each CS assertion returns the most recently
// captured pair of 16-bit PDM words, MSB first, and repeats after 32 bits.
module spi_mic_stream (
    input wire cs_n,
    input wire sck,
    input wire [31:0] sample_word,
    output wire miso
);
reg [31:0] shift_register;
reg [5:0] bit_count;

// CPHA=0 requires the first bit to be visible immediately after CS goes low.
// The first falling edge then loads the remaining 31 bits and every later
// falling edge advances the shift register.  Keeping all state changes in one
// sck process avoids a multiple-driver register in LSE.
assign miso = cs_n ? 1'b0 :
              (bit_count == 6'd0 ? sample_word[31] : shift_register[31]);

always @(negedge sck or posedge cs_n) begin
    if (cs_n) begin
        shift_register <= 32'd0;
        bit_count <= 6'd0;
    end else if (bit_count == 6'd0) begin
        shift_register <= {sample_word[30:0], 1'b0};
        bit_count <= 6'd1;
    end else if (bit_count < 6'd32) begin
        shift_register <= {shift_register[30:0], 1'b0};
        bit_count <= bit_count + 1'b1;
    end
end

endmodule
