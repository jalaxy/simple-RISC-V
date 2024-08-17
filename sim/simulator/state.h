#include <cstdint>
#include <vector>
#include <map>
#include <string>

extern const char *gprname[64];
extern std::map<uint16_t, const char *> csrname;

/* class for bit operation */
class bits
{
private:
    uint64_t data;

public:
    bits();
    bits(uint64_t);
    explicit bits(double);
    explicit bits(float);
    operator uint64_t() const;
    explicit operator double() const;
    explicit operator float() const;
    uint8_t operator[](uint8_t) const;
    uint64_t operator++(int);
    bits &operator>>=(int);
    bits &operator|=(uint64_t);
    bits range(uint8_t, uint8_t) const;
    int64_t sext(int) const;
    void write(uint8_t, uint8_t, uint64_t);
    void write(uint8_t, uint64_t);
};

/* class for segmented memory */
class memory
{
private:
    std::vector<uint64_t> base;
    std::vector<uint64_t> size;
    std::vector<uint8_t *> ptr;
    uint8_t segfault;

public:
    memory();
    memory(const memory &);
    ~memory();
    bool add(uint64_t, uint64_t);
    bool copy(uint8_t *, uint64_t, uint64_t);
    bool read(FILE *, uint64_t, uint64_t);
    uint8_t &ui8(uint64_t);
    uint16_t &ui16(uint64_t);
    uint32_t &ui32(uint64_t);
    uint64_t &ui64(uint64_t);
    bool issegfault(const uint8_t &);
    uint8_t &operator[](uint64_t);
    memory &operator=(const memory &);
};

typedef struct
{
    uint64_t pc = 0x80000000ull;
    uint32_t ir;
    memory mem;
    uint8_t level = 3;
    bits gpr[64];
    std::map<std::string, bits> csr = {
        {"mstatus", 0xa00002000ull},
        {"misa", 0x800000000014112d}, // USMIFDCA
        {"mcause", 0},
        {"mcycle", 0},
        {"pmpcfg0", 0},
        {"pmpcfg1", 0},
        {"pmpcfg2", 0}};
    std::map<uint64_t, uint8_t> rsrv;
} state_t;

typedef struct
{
    uint8_t level;      // NEXT level
    uint64_t pc;        // NEXT pc
    uint8_t gprw, memw; // gprw: [01]  memw: [01248] (rsrv: r+0x80/w+0xc0)
    uint64_t gpra, mema;
    uint64_t gprv, memv;
    std::map<std::string, bits> csr;
    uint8_t ldlocal; // possibly load local store in load axiom of RVWMO
    uint64_t ldaddr;
} delta_t;

typedef struct
{
    uint64_t fromhost = 0, tohost = 0, lock = 0;
} htifaddr_t;

uint64_t paddr(memory &mem, bits satp, bits vaddr, bits perm = 0, bool adpf = 1);
std::string disas(uint32_t ir);
delta_t next(state_t &s);
void apply(state_t &s, delta_t delta);
uint64_t htif(memory &mem, htifaddr_t &addr, std::vector<const char *> &pkargs, memory *pmem = 0);
void print(uint64_t cycle, state_t &state, const delta_t &delta);
void print(state_t &s, uint64_t addr = 0, uint64_t size = 0);
void dumpmem(const uint8_t *mem, uint64_t base, uint64_t size);
void disasmem(const uint8_t *mem, uint64_t size);
