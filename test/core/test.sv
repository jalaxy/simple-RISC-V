module test(
    input clk,
    input rst,
    output [63:0] pc,
    output [63:0] gpr[0:31],

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
    input         dcache_w_done
);
    reg [63:0] pcreg;
    assign pc = pcreg;
    always @(posedge clk)
        pcreg <= rst ? 64'h400000 : pcreg + 4;
    assign icache_addr = pcreg;
    assign icache_rqst = 1;
    // core core_inst(
    //     .clk(clk),
    //     .rst(rst),
    //     .icache_rqst(icache_rqst),
    //     .icache_addr(icache_addr),
    //     .icache_done(icache_done),
    //     .icache_data(icache_data),
    //     .dcache_r_rqst(dcache_r_rqst),
    //     .dcache_r_addr(dcache_r_addr),
    //     .dcache_r_done(dcache_r_done),
    //     .dcache_r_data(dcache_r_data),
    //     .dcache_w_rqst(dcache_w_rqst),
    //     .dcache_w_addr(dcache_w_addr),
    //     .dcache_w_data(dcache_w_data),
    //     .dcache_w_ext(dcache_w_ext),
    //     .dcache_w_width(dcache_w_width),
    //     .dcache_w_done(dcache_w_done));
    // assign pc = core_inst.pc;
    // assign gpr[0] = 32'd0;
    // for (genvar igen = 1; igen < 32; igen = igen + 1) begin: gpreg_out
    //     assign gpr[igen] = core_inst.gpreg_inst.r[igen]; end
endmodule
