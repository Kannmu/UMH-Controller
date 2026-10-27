`timescale 1ns/1ps

/* One 12-byte MSB-first record per CS: [id, sample_hi, sample_lo] for MIC0..MIC3. */
module spi_mic_stream (
    input wire cs_n,
    input wire sck,
    input wire [63:0] sample_word,
    output wire miso
);
reg [95:0] hold;
reg [6:0] bit_count;

wire [95:0] packet = {
    8'h00, sample_word[63:48],
    8'h01, sample_word[47:32],
    8'h02, sample_word[31:16],
    8'h03, sample_word[15:0]
};

wire [6:0] out_index = 7'd95 - bit_count;

assign miso = cs_n ? 1'b0 :
              (bit_count == 7'd0) ? packet[95] :
              (bit_count > 7'd95) ? 1'b0 : hold[out_index];

always @(negedge sck or posedge cs_n) begin
    if (cs_n) begin
        hold <= 96'd0;
        bit_count <= 7'd0;
    end else if (bit_count == 7'd0) begin
        hold <= packet;
        bit_count <= 7'd1;
    end else if (bit_count < 7'd96) begin
        bit_count <= bit_count + 1'b1;
    end
end

endmodule
