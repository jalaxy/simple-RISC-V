`timescale 1ns/1ns
module queue_tb();
    reg clk, rst, pop, push;
    reg [31:0] rear;
    wire empty, full;
    wire [31:0] front;
    queue queue_inst(
        .clk(clk),
        .rst(rst),
        .pop(pop),
        .push(push),
        .empty(empty),
        .full(full),
        .rear(rear),
        .front(front));
    integer i;
    initial begin
        rst = 1;
        clk = 0;
        #10;
        clk = 1; #10;
        clk = 0; #10;
        clk = 1; #10;
        clk = 0; #10;
        rst = 0; #10;

        pop = 0; push = 0;
        rear = 32'h12345;
        clk = 0; #10;
        clk = 1; #10;

        pop = 0; push = 1;
        clk = 0; #10;
        clk = 1; #10;

        pop = 0; push = 0;
        clk = 0; #10;
        clk = 1; #10;

        for (rear = 1; rear <= 10; rear = rear + 1) begin
            pop = 0; push = 1;
            clk = 0; #10;
            clk = 1; #10;
        end

        for (i = 0; i < 10; i = i + 1) begin
            pop = 1; push = 0;
            clk = 0; #10;
            clk = 1; #10;
        end

        pop = 1; push = 0;
        clk = 0; #10;
        clk = 1; #10;

        $finish();
    end
endmodule