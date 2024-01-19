module pipeline(
    input         clk,
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
    input         dcache_w_done
);
    reg [63:0] pc;
    always @(posedge clk)
        pc <= rst ? 64'h400000 : pc + 4;
    assign icache_rqst = 1;
    assign icache_addr = pc;
endmodule