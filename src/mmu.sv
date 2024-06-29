module mmu(
    input logic clk,
    input logic rst,
    input logic [63:0]  csr_satp,

    input  logic         icache_rqst,
    input  logic [63:0]  icache_addr,
    input  logic         icache_flsh,
    output logic         icache_done,
    output logic         icache_pgft,
    output logic [127:0] icache_data,

    input  logic       [7:0] dcache_rqst,
    input  logic       [1:0] dcache_rsrv,
    input  logic             dcache_wena,
    input  logic      [63:0] dcache_addr,
    input  logic       [2:0] dcache_bits,
    output logic       [7:0] dcache_done,
    output logic       [1:0] dcache_pgft,
    output logic      [63:0] dcache_rdat,
    input  logic      [63:0] dcache_wdat,
    input  logic             dcache_flsh

    input axi_rqst_t axi_rqst,
    output axi_resp_t axi_resp
);
endmodule

module cache #(parameter rports = 2,
    parameter way = 4, parameter set = 512, parameter line = 128) (
    input logic clk, input logic rst,
    input  logic            wena,
    input  logic     [63:0] windex,
    input  logic     [63:0] wtag,
    input  logic [line-1:0] wdata,
    output logic            wdir,
    output logic [line-1:0] wdirdata,
    input  logic [rports-1:0]    [63:0] rtag,
    input  logic [rports-1:0]    [63:0] rindex,
    output logic [rports-1:0]           rmiss,
    output logic [rports-1:0][line-1:0] rdata
);
    logic      [1:0] flag[set-1:0][way-1:0]; // dirty, valid
    logic     [64:0]  tag[set-1:0][way-1:0];
    logic [line-1:0] data[set-1:0][way-1:0];
    always_comb for (int i = 0; i < rports; i++) begin
        miss[i] = 1; rdata[i] = 0;
        for (int j = 0; j < way; j++) if (rtag == 64'(tag[index[i]][j]))
            {miss[i], rdata[i]} = {1'b0, data[index[i]][j]};
    end
    logic [7:0] p[set-1:0];
    always_ff @(posedge clk) for (int i = 0; i < set; i++)
        for (int j = 0; j < way; j++) if (rst) flag[i][j] <= 0; else begin
            ;
        end
    always_ff @(posedge clk) for (int i = 0; i < set; i++)
        if (rst) p[i] <= 0; else begin
            ;
        end
endmodule
