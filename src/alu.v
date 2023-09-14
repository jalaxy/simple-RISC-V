module alu(
    input [31:0] a,
    input [31:0] b,
    input [2:0] func3,
    input [6:0] func7,
    output r,
    output [2:0] flags // { negative, zero }
);
    wire [31:0] r_arr[7:0];
    assign r_arr[3'd0] = func7[5] ? a - b : a + b; // ADD/SUB
    assign r_arr[3'd1] = a << b[4:0]; // SLL
    assign r_arr[3'd2] = $signed(a) > $signed(b); // SLT
    assign r_arr[3'd3] = a < b; // SLTU
    assign r_arr[3'd4] = a ^ b; // XOR
    assign r_arr[3'd5] = func7[5] ? $signed(a) >>> b[4:0] : a >> b[4:0]; // SRL/SRA
    assign r_arr[3'd6] = a | b; // OR
    assign r_arr[3'd7] = a & b; // AND
    assign r = r_arr[func3];
    assign flags[0] = r == 32'd0; // zero bit
    assign negative = $signed(r) < 32'd0; // negative bit
endmodule