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
            pos <= pipeline_inst.wb_stage_inst.cqinfo[i].pc[6:3];
endmodule

module mul(input logic clk, input logic rst, input logic flush,
    input logic ena, input mul_rqst_t [3:0] rqst,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    mul_rqst_t [3:0] buffer[`CQSZ-1:0]; logic in, out;
    logic [`lgCQSZ-1:0] front; logic [`lgCQSZ:0] num;
    mul_rqst_t [3:0] cur; logic [2:0] curnum; logic [1:0] curi;
    always_comb begin
        in = 0; curnum = 0; curi = 0;
        for (int i = 0; i < 4; i++) if (rqst[i].id[`lgCQSZ]) in = 1;
        for (int i = 0; i < 4; i++) if (cur[i].id[`lgCQSZ]) curnum++;
        for (int i = 3; i >= 0; i--) if (cur[i].id[`lgCQSZ]) curi = 2'(i);
    end
    always_comb out = |num & curnum <= 3'(ena);
    always_ff @(posedge clk) if (rst | flush) front <= 0;
        else front <= front + `lgCQSZ'(out);
    always_ff @(posedge clk) if (rst | flush) num <= 0;
        else num <= num + `lgCQSZ'(in) - `lgCQSZ'(out);
    always_ff @(posedge clk) if (in) buffer[front + num[`lgCQSZ-1:0]] <= rqst;
    always_ff @(posedge clk) if (rst | flush) cur <= 0;
        else if (out) cur <= buffer[front];
        else if (ena) cur[curi].id <= 0;
`define latency 10
    logic [`latency-1:0][63:0] r_q;
    logic [`latency-1:0][`lgCQSZ:0] id;
    logic [127:0] res;
    always_comb res = {64'd0, cur[curi].a} * {64'd0, cur[curi].b};
    always_ff @(posedge clk)
        if (rst | flush) id <= {`lgCQSZ+1{`latency'd0}};
        else if (ena | ~id[0][`lgCQSZ]) begin
            id <= {cur[curi].id, id[`latency-1:1]};
            r_q <= {res[63:0], r_q[`latency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = id[0];
endmodule

module div(input logic clk, input logic rst, input logic flush,
    input logic ena, input div_rqst_t [3:0] rqst,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    always_ff @(posedge clk) r <= {1'b0, rqst[0].a | rqst[0].b};
    always_ff @(posedge clk) done <= rqst;
    always_comb e = 0;
endmodule

module fpu(input logic clk, input logic rst, input logic flush,
    input logic ena, input fpu_rqst_t [3:0] rqst,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    always_ff @(posedge clk) r <= {1'b0, rqst[0].a | rqst[0].b};
    always_ff @(posedge clk) done <= rqst;
    always_comb e = 0;
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
