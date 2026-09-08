`timescale 1ns/1ps

// CH347T is the SPI master.  Each CS assertion returns the most recently
// captured pair of 16-bit PDM words, MSB first, and repeats after 32 bits.
module spi_mic_stream (
    input wire cs_n,
    input wire sck,
    input wire mosi,
    input wire [31:0] sample_word,
    output wire miso
);
reg [31:0] shift_register;
assign miso = cs_n ? 1'b0 : shift_register[31];

always @(negedge cs_n)
    shift_register <= sample_word;

always @(negedge sck) begin
    if (!cs_n)
        shift_register <= {shift_register[30:0], shift_register[31]};
end

endmodule
