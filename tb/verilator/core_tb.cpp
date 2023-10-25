#include <cstdio>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vcore.h"
int main()
{
    Vcore *dut = new Vcore;
    Verilated::traceEverOn(true);
    VerilatedVcdC *m_trace=new VerilatedVcdC;
    dut->trace(m_trace, 5);
    m_trace->open("waveform.vcd");
    vluint64_t sim_time = 0;
    dut->a = dut->b = 0;
    while (sim_time < 20)
    {
        dut->a++;
        dut->b++;
        dut->eval();
        m_trace->dump(sim_time);
        sim_time++;
    }
    m_trace->close();
    delete dut;
    return 0;
}