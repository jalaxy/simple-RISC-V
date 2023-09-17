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
        for (i = 0; i < 100; i = i + 1) begin
            clk = 0;
            #10;
            clk = 1;
            #10;
        end
    end

    wire [31:0] pc;
    assign pc = core_inst.pc_wb;
    wire [31:0] ir;
    assign ir = core_inst.ir;
    reg [31:0] r[0:31];
    integer i;
    always @(*)
        for (i = 0; i < 32; i = i + 1)
            r[i] <= core_inst.gpreg_inst.r[i];
endmodule