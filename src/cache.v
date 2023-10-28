module icache(
    input clk,
    input ena,
    input [31:0] addr,
    output valid,
    output reg [31:0] data
);
    assign valid = 1'b1;
    integer fd, res;
    initial
        fd = $fopen("/home/ubuntu/Desktop/test.dump", "r");
    always @(posedge clk) begin
        if (ena) begin
            res <= $fseek(fd, ((addr-32'h00400000)>>2)*9, 0);
            res <= $fscanf(fd, "%h", data);
        end
    end
endmodule

module icache_synth(
    input clk,
    input ena,
    input [31:0] addr,
    output valid,
    output reg [31:0] data
);
    assign valid = 1'b1;
    wire [31:0] data_w, addr_relative;
    assign addr_relative = addr - 32'h00400000;
    rom rom_inst(.a(addr_relative[9:0] >> 2), .spo(data_w));
    always @(posedge clk) data <= ena ? data_w : data;
endmodule

module dcache(
    input clk,
    input r_ena,
    input w_ena,
    input [31:0] addr,
    input [1:0] width,
    input ext, // if extend with sign
    input [31:0] data_in,
    output valid,
    output reg [31:0] data_out
);
    assign valid = 1'b1;
    reg [31:0] mem[0:1024];
    always @(posedge clk) begin
        if (r_ena)
            data_out <= mem[(addr-32'h10010000)>>2];
        if (w_ena)
            mem[(addr-32'h10010000)>>2] <= data_in;
    end
endmodule
