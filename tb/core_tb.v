`timescale 1ns/1ns
module core_tb();
    reg clk, rst;
    core core_inst(clk, rst);
    initial begin
        rst = 0;
        #10;
        rst = 1;
        clk = 0;
        #10;
        clk = 1;
        #10;
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
`define N 16
    wire [31:0] pc;
    assign pc = core_inst.pc;
    wire [31:0] ir;
    assign ir = core_inst.ir;
    reg [31:0] r[1:`N];
    integer i;
    always @(*)
        for (i = 1; i <= `N; i = i + 1)
            r[i] <= core_inst.gpreg_inst.r[i];
    wire ma_ex_stall;
    assign ma_ex_stall = core_inst.ma_ex_stall;
    wire ena_pc, ena_if, ena_ex, ena_ma, ena_wb;
    assign ena_pc = core_inst.ena_pc;
    assign ena_if = core_inst.ena_if;
    assign ena_ex = core_inst.ena_ex;
    assign ena_ma = core_inst.ena_ma;
    assign ena_wb = core_inst.ena_wb;
    wire [4:0] test;
    assign test = core_inst.rs1_addr;
endmodule