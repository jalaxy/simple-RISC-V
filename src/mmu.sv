typedef struct packed {
    logic axi_;
} axi_rqst_t;

typedef struct packed {
    logic axi_;
} axi_resp_t;

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
    input  logic             dcache_flsh,

    input axi_rqst_t axi_rqst,
    output axi_resp_t axi_resp
);
endmodule

module cache #(parameter rports = 2,
    parameter way = 4, parameter set = 512, parameter line = 16) (
    input logic clk, input logic rst,
    input logic fill, input logic [31:0] findex,
    input logic [63:0] ftag, input logic [line-1:0][7:0] fdata,
    output logic victim, output logic [31:0] vindex,
    output logic [63:0] vtag, output logic [line-1:0][7:0] vdata,
    input  logic [rports:0]         [31:0] index, // contain the writing port
    input  logic [rports:0]         [63:0] tag,
    output logic [rports:0]                miss,
    output logic [rports:0][line-1:0][7:0] rdata,
    input logic [line-1:0] wena, input logic [line-1:0][7:0] wdata
);
    logic [63:0] tags[set-1:0][way-1:0];
    logic  [7:0] data[set-1:0][way-1:0][line-1:0];
    logic [set-1:0][way-1:0][1:0] flag; // dirty, valid
    logic [set-1:0][31:0]         p;    // fifo rear pointer
    logic [31:0] windex;
    always_comb windex = fill ? findex : index[rports];
    always_ff @(posedge clk) // dirty victim update
        if (rst) {victim, vindex, vtag, vdata} <= 0;
        else if (fill & &flag[findex][p[findex]][0]) begin
            victim <= 1'b1; vindex <= findex;
            vtag   <= tags[findex][p[findex]];
            for (int i = 0; i < line; i++) vdata[i] <= data[findex][p[findex]][i];
        end
    always_ff @(posedge clk) // flag and pointer update
        if (rst) for (int i = 0; i < set; i++)
            for (int j = 0; j < way; j++) {flag[i][j], p[i]} <= 0;
        else if (fill) begin
            flag[windex][p[windex]] <= 0;
            p[windex] <= p[windex] + 1 == way ? 0 : p[windex] + 1;
        end else if (|wena & flag[windex][p[windex]][0])
            flag[windex][p[windex]][1] <= 1;
    always_ff @(posedge clk) // data reading
        if (rst) {miss, rdata} <= 0;
        else for (int i = 0; i <= rports; i++) begin
            miss[i] <= 1; rdata[i] <= 0;
            for (int j = 0; j < way; j++) if (tag[i] == 64'(tags[index[i]][j])) begin
                miss[i] <= 0;
                for (int k = 0; k < line; k++) rdata[i][k] <= data[index[i]][j][k];
            end
        end
    always_ff @(posedge clk) for (int i = 0; i < line; i++)
        data[windex][p[windex]][i] = fill ? fdata[i] : data[rports][i];
    always_ff @(posedge clk) tags[windex][p[windex]] = fill ? ftag : tag[rports];
endmodule
