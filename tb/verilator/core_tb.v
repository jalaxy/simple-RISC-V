module core_tb(
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
    for (genvar igen = 0; igen < 32; igen = igen + 1) begin: gpreg_out
        assign gpr[igen] = core_inst.gpreg_inst.r[igen]; end
endmodule

module mul(
    input CLK,
    input [31:0] A,
    input [31:0] B,
    input CE,
    output [63:0] P
);
endmodule

module div(
    input aclk,
    input aclken,
    input [31:0] s_axis_divisor_tdata,
    input s_axis_divisor_tvalid,
    input [31:0] s_axis_dividend_tdata,
    input s_axis_dividend_tvalid,
    output [63:0] m_axis_dout_tdata,
    output m_axis_dout_tvalid
);
endmodule
