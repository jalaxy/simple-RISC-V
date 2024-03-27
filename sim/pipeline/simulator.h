#include <cstdint>
#include <map>

class simulator
{
private:
    std::map<uint64_t, uint8_t> memory;
    uint64_t pc = 0, npc, ir, arregs[64] = {0};
    uint64_t mwaddr, mwdata;
    uint8_t mwwidth;
    char asmcode[32];

public:
    simulator(const uint64_t &initpc, const std::map<uint64_t, uint8_t> &initmem);
    void step();
    int check(uint64_t pc, uint64_t rda, uint64_t rd,
              uint64_t mwaddr, uint64_t mwdata, uint8_t mwwidth);
    const uint64_t *get_arreg();
    uint64_t get_pc();
    uint64_t get_ir();
    uint64_t get_mwaddr();
    uint64_t get_mwdata();
    uint8_t get_mwwidth();
    char *get_asmcode();
    std::map<uint64_t, uint8_t> &get_mem();
};
