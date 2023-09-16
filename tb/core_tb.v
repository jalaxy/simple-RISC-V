module core_tb();
    reg clk, rst;
    integer i;
    core core_inst(clk, rst);
    initial begin
        rst = 0;
        #10;
        rst = 1;
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
    assign pc = core.pc_wb;
endmodule