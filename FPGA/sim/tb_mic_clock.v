`timescale 1ns/1ps
module EHXPLLJ #(parameter PLLRST_ENA="DISABLED", parameter INTFB_WAKE="DISABLED",
 parameter STDBY_ENABLE="DISABLED", parameter DPHASE_SOURCE="DISABLED",
 parameter CLKOP_FPHASE=0, parameter CLKOP_CPHASE=0, parameter OUTDIVIDER_MUXA2="DIVA",
 parameter CLKOP_ENABLE="ENABLED", parameter CLKOP_DIV=1, parameter CLKFB_DIV=1,
 parameter CLKI_DIV=1, parameter FEEDBK_PATH="INT_DIVA") (
 input CLKI, input CLKFB, input RST, input RESETM, input RESETC, input RESETD,
 input PHASESEL0, input PHASESEL1, input PHASEDIR, input PHASESTEP, input LOADREG,
 input STDBY, input PLLWAKESYNC, input ENCLKOP, output CLKOP, output LOCK, output CLKINTFB);
 assign CLKOP = CLKI; assign CLKINTFB = CLKFB; assign LOCK = 1'b1;
endmodule
module tb_mic_clock;
 reg clk64=0; always #7.8 clk64=~clk64;
 reg cs_n=1,sck=0,mosi=0; wire miso; wire [83:0] us_tx; wire rgb_data,mic_clk;
 reg md0=0,md1=0; wire spi_mic_miso;
 integer edges; real freq;
 umh_fpga_top dut(.fpga_clk_8m(clk64),.fpga_cs_n(cs_n),.spi1_sck(sck),.spi1_mosi(mosi),
  .spi1_miso(miso),.us_tx(us_tx),.rgb_data(rgb_data),.mic_clk(mic_clk),
  .mic_data_0(md0),.mic_data_1(md1),.spi_mic_cs_n(1'b1),.spi_mic_sck(1'b0),.spi_mic_miso(spi_mic_miso));
 task count_edges(output integer n);
   integer t; reg prev;
   begin
     n=0; t=0; prev=mic_clk;
     while(t<100000) begin
       @(posedge clk64); t=t+1;
       if(mic_clk !== prev) begin n=n+1; prev=mic_clk; end
     end
   end
 endtask
 initial begin
   #11000000;   /* 11 ms: must be in 200 kHz warm mode */
   count_edges(edges); freq = edges / 3.125e-3; $display("warm edges=%0d freq=%0.1f kHz",edges,freq/1000.0);
   #21000000;   /* now past 30 ms: must be 4 MHz run mode */
   count_edges(edges); freq = edges / 3.125e-3; $display("run edges=%0d freq=%0.1f kHz",edges,freq/1000.0);
   if (freq > 3900000.0 && freq < 4100000.0) $display("RESULT: PASS");
   else $display("RESULT: FAIL");
   $finish;
 end
endmodule
