`timescale 1ns/1ps
module tb_dump_all;
initial begin
 $dumpfile("tb_allchan.vcd");
 $dumpvars(0, tb_dump_i);
end
tb_umh_fpga_allchan tb_dump_i();
endmodule

