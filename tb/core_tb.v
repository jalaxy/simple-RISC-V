`timescale 1ns/1ns
module core_tb();
    reg clk, rst;
    integer i;
    core core_inst(clk, rst);
    initial begin
        rst = 0;
        #10;
        rst = 1;
        clk = 0;
        #10;
        clk = 1;
        #10;
        rst = 0;
        while (1) begin
            clk = 0;
            #10;
            clk = 1;
            #10;
        end
    end
`define N 3
    wire [31:0] pc;
    assign pc = core_inst.pc;
    wire [31:0] ir;
    assign ir = core_inst.ir;
    reg [31:0] r[1:`N];
    integer i;
    always @(*)
        for (i = 1; i <= `N; i = i + 1)
            r[i] <= core_inst.gpreg_inst.r[i];
    wire ena[3:0], jump, b_suc;
    assign ena[3]= core_inst.ena_if;
    assign ena[2] = core_inst.ena_ex;
    assign ena[1] = core_inst.ena_ma;
    assign ena[0] = core_inst.ena_wb;
    assign jump = core_inst.jump;
    assign b_suc = core_inst.b_suc;
    wire [3:0] flags;
    assign flags = core_inst.flags;
    wire [2:0] funct3_ex;
    assign funct3_ex = core_inst.funct3_ex;
endmodule