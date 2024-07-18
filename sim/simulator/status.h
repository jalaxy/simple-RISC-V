#include <cstdint>
#include <vector>
#include <map>
#include <string>

extern const char *gprname[64];
extern std::map<uint16_t, const char *> csrname;

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
} status_t;

typedef struct
{
    uint8_t level;
    uint64_t pc;
    uint8_t gprw, memw; // gprw: [01]  memw: [01248] (rsrv: r+0x80/w+0xc0)
    uint64_t gpra, mema;
    uint64_t gprv, memv;
    std::map<std::string, bits> csr;
} delta_t;

std::string disas(uint32_t ir);
delta_t next(status_t &status);
void apply(status_t &status, delta_t &delta);
