#include <cstdint>
#include <map>

typedef struct
{
    const char *name;
    uint64_t val;
} csr_t;

class simulator
{
private:
    std::map<uint64_t, uint8_t> memory, reserved;
    std::map<uint64_t, int> instfreq;
    uint64_t pc = 0, npc, arregs[64] = {0};
    uint64_t mwaddr, mwdata, csraddr, csrdata;
    uint32_t ir;
    uint8_t mwwidth;
    char asmcode[64];

public:
    std::map<uint64_t, csr_t> csr;
    simulator(const uint64_t &initpc, const std::map<uint64_t, uint8_t> &initmem);
    void step(int nojump = 0);
    int check(uint64_t pc, uint64_t rda, uint64_t rd,
              uint64_t mwaddr, uint64_t mwdata, uint8_t mwwidth,
              uint64_t csraddr, uint64_t csrdata);
    const uint64_t *get_arreg();
    uint64_t get_pc();
    uint64_t get_ir();
    uint64_t get_mwaddr();
    uint64_t get_mwdata();
    uint8_t get_mwwidth();
    uint64_t get_csraddr();
    uint64_t get_csrdata();
    const char *get_asmcode();
    const char *get_csrname(uint64_t addr);
    const std::map<uint64_t, int> &get_freq();
    std::map<uint64_t, uint8_t> &get_mem();
};
