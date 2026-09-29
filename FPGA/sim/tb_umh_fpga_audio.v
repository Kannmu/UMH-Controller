`timescale 1ns/1ps

module tb_umh_fpga_audio;
reg fpga_clk_8m = 1'b0;
reg fpga_cs_n = 1'b1;
reg spi1_sck = 1'b0;
reg spi1_mosi = 1'b0;
wire spi1_miso;
wire [83:0] us_tx;
wire rgb_data;
wire mic_clk;
reg mic_data_0 = 1'b0;
reg mic_data_1 = 1'b0;
reg spi_mic_cs_n = 1'b1;
reg spi_mic_sck = 1'b0;
wire spi_mic_miso;
integer failures = 0;
integer n;

umh_fpga_top dut (.*);
always #62.5 fpga_clk_8m = ~fpga_clk_8m;

/* Output-domain monitor: every FIFO pop and bank swap is time-stamped in
 * 64 MHz cycles, and the pop -> swap_pending time is the build length. */
integer cycle = 0;
integer pop_count = 0, swap_count = 0;
integer build_start = -1, build_max = 0;
integer pop_cycle [0:1023];
integer swap_cycle [0:1023];
reg [7:0] pop_level [0:1023];
reg swap_pending_d = 1'b0;
reg invalid_seen = 1'b0;

always @(posedge dut.fpga_clk) begin
    cycle <= cycle + 1;
    swap_pending_d <= dut.swap_pending;
    if (dut.invalid_frame_sync) invalid_seen <= 1'b1;
    if (dut.afifo_pop) begin
        pop_level[pop_count % 1024] <= dut.afifo_q;
        pop_cycle[pop_count % 1024] <= cycle;
        pop_count <= pop_count + 1;
        build_start <= cycle;
    end
    /* FRAME and AUDIO_MODE builds have build_start = -1 and are skipped. */
    if (dut.swap_pending && !swap_pending_d && build_start >= 0) begin
        if (cycle - build_start > build_max) build_max <= cycle - build_start;
        build_start <= -1;
    end
    if (dut.swap_now_s3) begin
        swap_cycle[swap_count % 1024] <= cycle;
        swap_count <= swap_count + 1;
    end
end
task spi_byte;
    input [7:0] value;
    output [7:0] received;
    integer bit_index;
    begin
        received = 8'd0;
        for (bit_index = 7; bit_index >= 0; bit_index = bit_index - 1) begin
            spi1_mosi = value[bit_index];
            #30 spi1_sck = 1'b1;
            #5 received[bit_index] = spi1_miso;
            #25 spi1_sck = 1'b0;
        end
    end
endtask

task begin_transaction;
    begin
        spi1_mosi = 1'b0;
        #50 fpga_cs_n = 1'b0;
        #50;
    end
endtask

task end_transaction;
    begin
        #50 fpga_cs_n = 1'b1;
        #100;
    end
endtask

/* mode 0: channel 0 phase 0x40, marker 0 (silent aperture load)
 * mode 1: channel 0 phase 0x40, marker 0xff
 * mode 2: all 84 channels enabled, phase 3*n (worst-case build length) */
task send_frame;
    input [1:0] mode;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(8'h10, discard);
        spi_byte(8'h01, discard);
        for (n = 0; n < 4; n = n + 1) spi_byte(8'h00, discard);
        spi_byte(8'h01, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 8; n = n + 1) spi_byte(8'h00, discard);
        spi_byte(8'h03, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 10; n = n + 1) spi_byte(8'hff, discard);
        spi_byte(8'h0f, discard);
        spi_byte(8'h0f, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        for (n = 0; n < 84; n = n + 1) begin
            if (mode == 2'd2) begin
                spi_byte((n * 3) & 8'hff, discard);
                spi_byte(8'hff, discard);
            end else begin
                spi_byte(n == 0 ? 8'h40 : 8'h00, discard);
                spi_byte(n == 0 && mode == 2'd1 ? 8'hff : 8'h00, discard);
            end
        end
        for (n = 0; n < 12; n = n + 1) spi_byte(8'h00, discard);
        end_transaction;
    end
endtask

task send_compact;
    input [7:0] command;
    input [7:0] data;
    input [31:0] seq_value;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(command, discard);
        spi_byte(8'h01, discard);
        spi_byte(data, discard);
        spi_byte(8'h00, discard); spi_byte(8'h00, discard); spi_byte(8'h00, discard);
        spi_byte(seq_value[7:0], discard);
        spi_byte(seq_value[15:8], discard);
        spi_byte(seq_value[23:16], discard);
        spi_byte(seq_value[31:24], discard);
        for (n = 0; n < 6; n = n + 1) spi_byte(8'h00, discard);
        end_transaction;
    end
endtask

/* AUDIO_BLOCK [0x1A, 0x01, blk[0..count-1]]; returns status byte 1. */
reg [7:0] blk [0:31];
task send_block;
    input integer count;
    output [7:0] fill;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(8'h1A, discard);
        spi_byte(8'h01, fill);
        for (n = 0; n < count; n = n + 1) spi_byte(blk[n], discard);
        end_transaction;
    end
endtask
/* Header-only command (STATUS, STOP, RESET): 36 bytes, all fields zero. */
task send_header;
    input [7:0] command;
    integer n;
    reg [7:0] discard;
    begin
        begin_transaction;
        spi_byte(command, discard);
        spi_byte(8'h01, discard);
        for (n = 0; n < 34; n = n + 1) spi_byte(8'h00, discard);
        end_transaction;
    end
endtask
task fail;
    input [8*64-1:0] msg;
    begin
        $display("FAIL: %0s", msg);
        failures = failures + 1;
    end
endtask

integer i, k, base, s0, t0, d, high0, high83, edges, gap_bad, swap_bad;
reg [7:0] fill, fill2, want;
reg prev;

initial begin
    #1000;
    /* Firmware order: silent phase load, enter audio mode, then load the
     * real enable markers while the common level is still zero. */
    send_frame(2'd0);
    #200000;
    if (dut.running !== 1'b1) fail("full frame did not start the output engine");
    send_compact(8'h17, 8'd1, 32'd1);
    #100000;
    if (dut.audio_mode !== 1'b1) fail("audio mode was not entered");
    send_frame(2'd1);
    #200000;

    /* 1. Short block: four samples at level 64. */
    for (i = 0; i < 4; i = i + 1) blk[i] = 8'd64;
    send_block(4, fill);
    if (fill !== 8'd0) fail("status fill before the first block");
    #400000;
    if (pop_count !== 4) fail("short block pop count");
    if (dut.pending_audio_level !== 8'd64) fail("level 64 was not adopted");
    if (dut.event_ram.mem[dut.active_bank ? (256 + 8'h40) : 8'h40][0] !== 1'b1 ||
        dut.event_ram.mem[dut.active_bank ? (256 + 8'h80) : 8'h80][0] !== 1'b1)
        fail("event table did not encode phase 0x40 level 0x40");

    /* 2. Underrun: an empty FIFO holds the last level without rebuilds. */
    k = swap_count;
    #200000;
    if (swap_count !== k || dut.pending_audio_level !== 8'd64 || dut.running !== 1'b1)
        fail("underrun did not hold the last level");

    /* 3. Status byte 1 reports the fill level; samples pop in order. */
    for (i = 0; i < 32; i = i + 1) blk[i] = 8'd20 + i;
    send_block(32, fill);
    send_block(0, fill2);
    if (fill !== 8'd0 || fill2 < 8'd30 || fill2 > 8'd32) fail("fill report");
    #1800000;
    if (pop_count !== 36) fail("32-sample block pop count");
    for (i = 0; i < 32; i = i + 1)
        if (pop_level[4 + i] !== 8'd20 + i) fail("pop order");

    /* 4. Worst-case build (84 enabled channels) with a deep FIFO: every
     *    sample must last exactly two carrier wraps (3200 cycles) and its
     *    bank must swap in one wrap (1600 cycles) after the pop. */
    send_frame(2'd2);
    #200000;
    base = pop_count; s0 = swap_count;
    for (k = 0; k < 6; k = k + 1) begin
        for (i = 0; i < 32; i = i + 1) begin
            d = k * 32 + i;
            blk[i] = (d < 176) ? ((d * 37) % 129) : 8'd100;
        end
        send_block(32, fill);
    end
    t0 = 0;
    while (pop_count < base + 192 && t0 < 12000) begin
        #1000; t0 = t0 + 1;
    end
    #40000;
    if (pop_count !== base + 192) fail("burst pop count");
    gap_bad = 0; swap_bad = 0;
    for (i = 0; i < 192; i = i + 1) begin
        want = (i < 176) ? ((i * 37) % 129) : 8'd100;
        if (pop_level[base + i] !== want) gap_bad = gap_bad + 1000;
        if (i > 0 && (pop_cycle[base + i] - pop_cycle[base + i - 1] < 3198 ||
                      pop_cycle[base + i] - pop_cycle[base + i - 1] > 3202))
            gap_bad = gap_bad + 1;
        if (swap_cycle[s0 + i] - pop_cycle[base + i] < 1595 ||
            swap_cycle[s0 + i] - pop_cycle[base + i] > 1605)
            swap_bad = swap_bad + 1;
    end
    $display("burst: pops=%0d swaps=%0d gap_bad=%0d swap_bad=%0d build_max=%0d cycles",
             pop_count - base, swap_count - s0, gap_bad, swap_bad, build_max);
    if (gap_bad != 0) fail("burst order or pop spacing");
    if (swap_bad != 0 || swap_count - s0 !== 192) fail("burst swap timing");
    if (build_max >= 1590) fail("event rebuild longer than one carrier wrap");

    /* Duty at the held level 100: 100/256 of the time high, one pulse per
     * carrier.  Channel 83 (phase 249) wraps its falling edge, which checks
     * the slot-0 state path. */
    high0 = 0; high83 = 0; edges = 0; prev = dut.us_tx[0];
    for (i = 0; i < 16000; i = i + 1) begin
        @(posedge dut.fpga_clk);
        if (dut.us_tx[0]) high0 = high0 + 1;
        if (dut.us_tx[83]) high83 = high83 + 1;
        if (dut.us_tx[0] && !prev) edges = edges + 1;
        prev = dut.us_tx[0];
    end
    $display("duty @100: ch0 high=%0d ch83 high=%0d ch0 pulses=%0d (expect 6250, 6250, 10)",
             high0, high83, edges);
    if (high0 < 6230 || high0 > 6270 || high83 < 6230 || high83 > 6270 ||
        edges < 9 || edges > 11)
        fail("output duty at level 100");

    /* 5. Leaving audio mode drops queued samples and silences the output. */
    for (i = 0; i < 32; i = i + 1) blk[i] = 8'd90;
    send_block(32, fill);
    #20000;
    send_compact(8'h17, 8'd0, 32'd2);
    #100000;
    k = pop_count;
    #200000;
    if (dut.audio_mode !== 1'b0 || dut.afifo_fill !== 8'd0 || pop_count !== k ||
        dut.us_tx !== 84'd0)
        fail("mode exit did not flush and silence");
    send_compact(8'h17, 8'd1, 32'd3);
    #100000;
    if (dut.audio_mode !== 1'b1 || dut.afifo_fill !== 8'd0) fail("mode re-entry");

    /* 6. Legacy per-sample commands are ignored, and not flagged invalid. */
    send_compact(8'h16, 8'd64, 32'd4);
    #100000;
    if (dut.pending_audio_level !== 8'd0 || invalid_seen) fail("legacy 0x16 was not ignored");

    /* 7. STOP while streaming wins over the rebuild it races with. */
    for (i = 0; i < 32; i = i + 1) blk[i] = 8'd80;
    send_block(32, fill);
    #130000;
    send_header(8'h11);
    #200000;
    if (dut.running !== 1'b0 || dut.audio_mode !== 1'b0 || dut.us_tx !== 84'd0)
        fail("STOP during streaming");

    /* 8. Restart after STOP, then the link watchdog must stop the output
     *    28.7..30.7 ms after the last chip-select. */
    send_frame(2'd0);
    #200000;
    if (dut.running !== 1'b1) fail("restart after STOP");
    #26000000;
    if (dut.running !== 1'b1) fail("watchdog fired before 26 ms");
    #5000000;
    if (dut.running !== 1'b0 || dut.us_tx !== 84'd0) fail("watchdog did not fire by 31 ms");

    if (invalid_seen) fail("an audio transaction was flagged invalid");
    if (failures == 0) $display("RESULT: PASS");
    else $display("RESULT: FAIL (%0d)", failures);
    $finish;
end
endmodule
