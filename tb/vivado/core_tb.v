`timescale 1ns/1ns
module core_tb();
    reg clk, rst;
    wire signal;
    top inst(clk, rst, signal);
    integer num;
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
        num = 0;
        while (1) begin
            clk = 0;
            #10;
            clk = 1;
            #10;
            num = num + 1;
        end
    end
`define N 16
    wire [31:0] pc;
    assign pc = inst.core_inst.pc;
    wire [31:0] ir;
    assign ir = inst.core_inst.ir_if;
    reg [31:0] r[1:`N];
    integer i;
    always @(*)
        for (i = 1; i <= `N; i = i + 1)
            r[i] = inst.core_inst.gpr[i];
endmodule