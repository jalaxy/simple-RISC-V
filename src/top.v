module top(
    input clk,
    input rst,
    output signal
);
    wire icache_valid, icache_ena;
    wire [31:0] icache_data, icache_addr;
    wire dcache_r_ena, dcache_w_ena, dcache_ext, dcache_valid;
    wire [31:0] dcache_addr, dcache_data_in, dcache_data_out;
    wire [1:0] dcache_width;
    core core_inst(
        .clk(clk), .rst(rst),
        .icache_valid(icache_valid),
        .icache_data(icache_data),
        .icache_ena(icache_ena),
        .icache_addr(icache_addr),
        .dcache_r_ena(dcache_r_ena),
        .dcache_w_ena(dcache_w_ena),
        .dcache_ext(dcache_ext),
        .dcache_valid(dcache_valid),
        .dcache_addr(dcache_addr),
        .dcache_data_in(dcache_data_in),
        .dcache_data_out(dcache_data_out),
        .dcache_width(dcache_width));
    icache_synth icache_inst(
        .clk(clk),
        .ena(icache_ena),
        .addr(icache_addr),
        .valid(icache_valid),
        .data(icache_data));
    dcache dcache_inst(
        .clk(clk),
        .r_ena(dcache_r_ena),
        .w_ena(dcache_w_ena),
        .ext(dcache_ext),
        .addr(dcache_addr),
        .valid(dcache_valid),
        .width(dcache_width),
        .data_in(dcache_data_in),
        .data_out(dcache_data_out));
    assign signal = core_inst.pc == 32'h0040007c;
endmodule