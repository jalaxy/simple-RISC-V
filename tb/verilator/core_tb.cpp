#include <cstdio>
#include <verilated.h>
#include "Vcore_tb.h"
int main()
{
    Vcore_tb *tb = new Vcore_tb;
    tb->rst = 0, tb->eval();
    tb->rst = 1, tb->clk = 0, tb->eval();
    for (int i = 0; i < 8; i++)
        tb->clk = !tb->clk, tb->eval();
    tb->rst = 0, tb->eval();
    for (int i = 0; i < 100; i++)
    {
        printf("cycle %d:\n    pc: 0x%08x\n", i, tb->pc);
        tb->clk = 0, tb->eval();
        tb->clk = 1, tb->eval();
    }
    delete tb;
    return 0;
}