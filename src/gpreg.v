`timescale 1ns/1ns
module gpreg(
    input clk,
    input rd_w,
    input [4:0] rs1_addr,
    input [4:0] rs2_addr,
    input [4:0] rd_addr,
    input [31:0] rd,
    output [31:0] rs1,
    output [31:0] rs2
);
    reg [31:0] r[0:31];
    wire clk_rd;
    assign clk_rd = clk & rd_w;
    assign rs1 = rs1_addr == 5'd0 ? 32'd0 : r[rs1_addr];
    assign rs2 = rs2_addr == 5'd0 ? 32'd0 : r[rs2_addr];
    always@(posedge clk_rd)
        r[rd_addr] <= rd;
endmodule