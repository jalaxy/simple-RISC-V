module test(
    input clk,
    input rst,
    output [31:0] pc,
    output [31:0] gpr[0:31],

    input icache_valid,
    input [31:0] icache_data,
    input dcache_valid,
    input [31:0] dcache_data_out,
    output icache_ena,
    output [31:0] icache_addr,
    output dcache_r_ena,
    output dcache_w_ena,
    output dcache_ext,
    output [1:0] dcache_width,
    output [31:0] dcache_addr,
    output [31:0] dcache_data_in
);
    core core_inst(
        .clk(clk),
        .rst(rst),
        .icache_valid(icache_valid),
        .icache_data(icache_data),
        .dcache_valid(dcache_valid),
        .dcache_data_out(dcache_data_out),
        .icache_ena(icache_ena),
        .icache_addr(icache_addr),
        .dcache_r_ena(dcache_r_ena),
        .dcache_w_ena(dcache_w_ena),
        .dcache_ext(dcache_ext),
        .dcache_width(dcache_width),
        .dcache_addr(dcache_addr),
        .dcache_data_in(dcache_data_in));
    assign pc = core_inst.pc;
    assign gpr[0] = 32'd0;
    for (genvar igen = 1; igen < 32; igen = igen + 1) begin: gpreg_out
        assign gpr[igen] = core_inst.gpreg_inst.r[igen]; end
endmodule
