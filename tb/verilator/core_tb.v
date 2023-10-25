module core_tb(
    input clk,
    input rst,
    output [31:0] pc,
    output [31:0] gpr[0:31]
);
    core core_inst();
endmodule

module mul(
    input CLK,
    input [31:0] A,
    input [31:0] B,
    input CE,
    output [63:0] P
);
endmodule

module div(
    input aclk,
    input aclken,
    input [31:0] s_axis_divisor_tdata,
    input s_axis_divisor_tvalid,
    input [31:0] s_axis_dividend_tdata,
    input s_axis_dividend_tvalid,
    output [63:0] m_axis_dout_tdata,
    output m_axis_dout_tvalid
);
endmodule
