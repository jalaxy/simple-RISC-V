`define CQSZ 16
`define lgCQSZ 4
module wrapper(input logic clk, input logic rst, output logic [3:0] pos);
    logic [63:0][7:0] imem;
    logic imem_rqst, imem_done;
    logic [63:0] imem_addr, imem_data;
    always_ff @(posedge clk) if (rst) imem <= {
        32'h6f,
        32'h00120213,
        32'hfe41c8e3,
        32'h00110113,
        32'h02218133,
        32'h02000213,
        32'h00118193,
        32'hfedff06f,
        32'hffe18193,
        32'h02310133,
        32'h00118193,
        32'h02208133,
        32'h00018c63,
        32'h01000193,
        32'h45600113,
        32'h12300093
    };
    always_ff @(posedge clk) if (rst) imem_done <= 0; else imem_done <= imem_rqst;
    always_ff @(posedge clk) imem_data <= imem[imem_addr[5:0]+7-:8];
    logic [15:0][63:0] dmem;
    logic [`lgCQSZ:0] dmem_rqst, dmem_done;
    logic dmem_wena, dmem_flsh; logic [1:0] dmem_rsrv, dmem_bits;
    logic [63:0] dmem_addr, dmem_rdat, dmem_wdat;
    always_ff @(posedge clk) if (rst) dmem_done <= 0; else dmem_done <= dmem_rqst;
    always_ff @(posedge clk) dmem_rdat <= dmem[dmem_addr[3:0]];
    always_ff @(posedge clk) if (dmem_wena) dmem[dmem_addr[3:0]] <= dmem_wdat;
    pipeline pipeline_inst(clk, rst,
        imem_rqst, imem_addr, imem_flsh, imem_done, imem_data,
        dmem_rqst, dmem_rsrv, dmem_wena, dmem_addr, dmem_bits,
        dmem_done, dmem_rdat, dmem_wdat, dmem_flsh);
    always_ff @(posedge clk) for (int i = 0; i < 2; i++)
        if (pipeline_inst.wb_stage_inst.cqpop[i])
            pos <= pipeline_inst.wb_stage_inst.cqinfo[i].pc[5:2] + 3;
endmodule

module mul(input logic clk, input logic rst, input logic flush,
    input logic ena, output logic get,
    input logic [`lgCQSZ:0] rqst, input logic [4:0] op,
    input logic [63:0] a, input logic [63:0] b,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
`define latency 10
    logic [`latency-1:0][63:0] r_q;
    logic [`latency-1:0][`lgCQSZ:0] valid;
    logic [127:0] res;
    always_comb res = {64'd0, a} * {64'd0, b};
    always_ff @(posedge clk)
        if (rst | flush) valid <= {`lgCQSZ+1{`latency'd0}};
        else if (ena | ~valid[0][`lgCQSZ]) begin
            valid <= {rqst, valid[`latency-1:1]};
            r_q <= {res[63:0], r_q[`latency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = valid[0];
    always_comb get = ena | ~valid[0][`lgCQSZ];
endmodule

module div(input logic clk, input logic rst, input logic flush,
    input logic ena, output logic get,
    input logic [`lgCQSZ:0] rqst, input logic [7:0] op,
    input logic [63:0] a, input logic [63:0] b,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    always_ff @(posedge clk) r <= {1'b0, a | b};
    always_ff @(posedge clk) done <= rqst;
    always_comb e = 0;
    always_comb get = ena;
endmodule

module fpu(input logic clk, input logic rst, input logic flush,
    input logic ena, output logic get,
    input logic [`lgCQSZ:0] rqst, input logic [20:0] op,
    input logic [63:0] a, input logic [63:0] b,
    input logic [2:0] rm, input logic double,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    always_ff @(posedge clk) r <= {1'b0, a | b};
    always_ff @(posedge clk) done <= rqst;
    always_comb e = 0;
    always_comb get = ena;
endmodule

module regfile #(parameter dwidth = 64,
    parameter rports = 2, parameter wports = 1,
    parameter awidth = 6, parameter depth = 64) (
    input  logic clk, input logic rst,
    input  logic [rports-1:0][awidth-1:0] raddr,
    output logic [rports-1:0][dwidth-1:0] rvalue,
    input  logic [wports-1:0][awidth-1:0] waddr,
    input  logic [wports-1:0][dwidth-1:0] wvalue,
    input  logic [wports-1:0] wena
);
    int sel[depth-1:0];
    always_ff @(posedge clk)
        for (int i = 0; i < wports; i++) if (wena[i]) sel[waddr[i]] <= i;
    for (genvar i = 0; i < wports; i++) begin : dupregs
        logic [dwidth-1:0] regs[depth-1:0];
        always_ff @(posedge clk)
            if (wena[i]) regs[waddr[i]] <= wvalue[i];
    end
    for (genvar i = 0; i < rports; i++) begin
        logic [dwidth-1:0] dupval[wports-1:0];
        for (genvar j = 0; j < wports; j++)
            always_comb dupval[j] = dupregs[j].regs[raddr[i]];
        always_comb rvalue[i] = dupval[sel[raddr[i]]];
    end
endmodule
