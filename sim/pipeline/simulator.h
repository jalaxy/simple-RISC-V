#include <cstdint>
#include <map>

class simulator
{
private:
    std::map<uint64_t, uint8_t> memory;
    uint64_t pc, arregs[64];

public:
    simulator(const uint64_t &initpc, const std::map<uint64_t, uint8_t> &initmem);
    void step();
};
