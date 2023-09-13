module gpreg(
    input clk,
    input rst,
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
    assign clk_rd = clk & (rd_w | rst);
    assign rs1 = r[rs1_addr];
    assign rs2 = r[rs2_addr];
    always@(posedge clk_rd) // reset is sync mode
        r[rd_addr] <= rst ? 32'd0 : rd;
endmodule