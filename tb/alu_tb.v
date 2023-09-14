`timescale 1ns/1ns
module alu_tb();
    reg [31:0] a, b;
    reg [3:0] func3;
    reg [6:0] func7;
    wire [31:0] r;
    alu inst(a, b, func3, func7, r);
    initial begin
        a = 32'hfffffffe;
        b = 4;
        for (func3 = 0; func3 < 8; func3 = func3 + 1) begin
            func7 = 0;
            #10;
            func7 = 7'b0100000;
            #10;
        end
    end
endmodule