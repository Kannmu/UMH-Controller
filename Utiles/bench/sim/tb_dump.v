`timescale 1ns/1ps
module tb_dump;
initial begin
 $dumpfile("tb_audio.vcd");
 $dumpvars(0, tb_dump_i);
end
tb_umh_fpga_audio tb_dump_i();
endmodule

