module test(
    input         clk, // passed from and to core instance
    input         rst,

    output        icache_rqst,
    output [63:0] icache_addr,
    input         icache_done,
    input  [63:0] icache_data,

    output        dcache_r_rqst,
    output [63:0] dcache_r_addr,
    output  [2:0] dcache_r_bits,
    input         dcache_r_done,
    input  [63:0] dcache_r_data,

    output        dcache_w_rqst,
    output [63:0] dcache_w_addr,
    output  [2:0] dcache_w_bits,
    output [63:0] dcache_w_data,
    input         dcache_w_done,

    output [63:0] gpr[0:31] // additional info
);
    pipeline pipeline_inst(
        .clk(clk),
        .rst(rst),
        .icache_rqst(icache_rqst),
        .icache_addr(icache_addr),
        .icache_done(icache_done),
        .icache_data(icache_data),
        .dcache_r_rqst(dcache_r_rqst),
        .dcache_r_addr(dcache_r_addr),
        .dcache_r_bits(dcache_r_bits),
        .dcache_r_done(dcache_r_done),
        .dcache_r_data(dcache_r_data),
        .dcache_w_rqst(dcache_w_rqst),
        .dcache_w_addr(dcache_w_addr),
        .dcache_w_bits(dcache_w_bits),
        .dcache_w_data(dcache_w_data),
        .dcache_w_done(dcache_w_done));
    // assign gpr[0] = 32'd0;
    // for (genvar igen = 1; igen < 32; igen = igen + 1) begin: gpreg_out
    //     assign gpr[igen] = core_inst.gpreg_inst.r[igen]; end
endmodule
