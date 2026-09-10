`timescale 1ns/1ps

// CH347T is the SPI master. Each CS assertion returns a 12-byte snapshot,
// MSB first. Records are [source_id, sample_hi, sample_lo] for MIC0..MIC3.
module spi_mic_stream (
    input wire cs_n,
    input wire sck,
    input wire [63:0] sample_word,
    output wire miso
);
reg [95:0] shift_register;
reg [6:0] bit_count;

wire [95:0] packet = {
    8'h00, sample_word[63:48],
    8'h01, sample_word[47:32],
    8'h02, sample_word[31:16],
    8'h03, sample_word[15:0]
};

// CPHA=0 requires the first bit to be visible immediately after CS goes low.
// The first falling edge then loads the remaining 95 bits and every later
// falling edge advances the shift register.  Keeping all state changes in one
// sck process avoids a multiple-driver register in LSE.
assign miso = cs_n ? 1'b0 :
              (bit_count == 7'd0 ? packet[95] : shift_register[95]);

always @(negedge sck or posedge cs_n) begin
    if (cs_n) begin
        shift_register <= 96'd0;
        bit_count <= 7'd0;
    end else if (bit_count == 7'd0) begin
        shift_register <= {packet[94:0], 1'b0};
        bit_count <= 7'd1;
    end else if (bit_count < 7'd96) begin
        shift_register <= {shift_register[94:0], 1'b0};
        bit_count <= bit_count + 1'b1;
    end
end

endmodule
