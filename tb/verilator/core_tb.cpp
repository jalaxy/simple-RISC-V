#include <cstdio>
#include <verilated.h>
#include "Vcore_tb.h"
extern unsigned int inst[], arr[];
int main()
{
    Vcore_tb *tb = new Vcore_tb;
    tb->rst = 0, tb->eval();
    tb->rst = 1, tb->clk = 0, tb->eval();
    for (int i = 0; i < 8; i++)
        tb->clk = !tb->clk, tb->eval();
    tb->rst = 0, tb->eval();
    for (int i = 0; i < 1000; i++)
    {
        unsigned int icache_data = inst[(tb->icache_addr - 0x400000) >> 2];
        tb->icache_valid = tb->dcache_valid = 1;
        printf("cycle %d:\n    pc: 0x%08x\n", i, tb->pc);
        for (int i = 0; i <32; i++)
            printf("    x%d: 0x%08x\n", i, tb->gpr[i]);
        tb->clk = 0, tb->eval();
        tb->clk = 1, tb->eval();
        if (tb->icache_ena)
            tb->icache_data = icache_data;
        if (tb->dcache_r_ena)
            tb->dcache_data_out = arr[(tb->dcache_addr - 0x10010000) >> 2];
        if (tb->dcache_w_ena)
            arr[(tb->dcache_addr - 0x10010000) >> 2] = tb->dcache_data_in;
    }
    delete tb;
    return 0;
}
unsigned int arr[1024],
    inst[] = {
        0x000000b3,
        0x00000133,
        0x02000193,
        0x00108093,
        0x00210113,
        0xfe30cce3,
        0x0040006f,
        0x00408093,
        0x0040026f,
        0x0000006f,
        0x00000013};