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
endmodule