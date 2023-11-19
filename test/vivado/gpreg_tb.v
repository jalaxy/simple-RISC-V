`timescale 1ns/1ns
module gpreg_tb();
    reg clk, rd_w;
    reg [4:0] rs1_addr, rs2_addr, rd_addr;
    reg [31:0] rd;
    wire [31:0] rs1, rs2;
    gpreg inst(clk, rd_w, rs1_addr, rs2_addr, rd_addr, rd, rs1, rs2);
    wire clk_rd;
    assign clk_rd = inst.clk_rd;
    integer i;
    initial begin
        rs1_addr = 1;
        rs2_addr = 18;
        rd_w = 0;
        clk = 0; #10;
        clk = 1; #100;
        clk = 0; #10;
        rd_w = 1;
        for (i = 0; i < 32; i = i + 1) begin
            rd_addr = i;
            rd = 32 - i;
            clk = 0;
            #5;
            clk = 1;
            #5;
        end
        rd_w = 0;
        for (i = 0; i < 32; i = i + 1) begin
            rd_addr = i;
            rd = i;
            clk = 0;
            #5;
            clk = 1;
            #5;
        end
        for (i = 0; i < 32; i = i + 1) begin
            rs1_addr = i;
            rs2_addr = 32 - i;
            #10;
        end
    end
endmodule