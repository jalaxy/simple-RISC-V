#include <cstdio>
#include <cstring>
#include <vector>
#include "status.h"

typedef struct
{
    const char *filename = 0, *dtb = 0, *initrd = 0;
    std::vector<const char *> args;
    uint8_t debug = 0, help = 0, filetype = 0;
    int mintime = 0, maxtime = INT32_MAX;
} cmd_t;

typedef struct
{
    uint64_t fromhost = 0, tohost = 0, lock = 0;
} htif_t;

void print(const status_t &status, const delta_t &delta)
{
    fprintf(stderr, "[Info] i@%lx: %s\n", status.pc, disas(status.ir).c_str());
    if (delta.gprw)
        fprintf(stderr, "[Info]     %s: %lx\n", gprname[delta.gpra], delta.gprv);
    if (delta.memw)
        fprintf(stderr, "[Info]     d%d@%lx: %lx\n", delta.memw, delta.mema, delta.memv);
}

void dumpmem(uint8_t *mem, uint64_t addr, uint64_t size)
{
    printf("Memory@%016lx:", addr);
    for (int i = 0; i < size; i++)
    {
        i % 16 ? printf(i % 2 ? "" : " ") : printf("\n%016lx: ", addr + i);
        printf("%02x", mem[addr + i]);
        if ((i + 1) % 16 == 0 || i == size - 1)
        {
            if (i == size - 1)
                for (int j = i + 1; j < i / 16 * 16 + 16; j++)
                    printf(j % 2 ? "  " : "   ");
            printf("  ");
            for (int j = i / 16 * 16; j <= i; j++)
                if (mem[addr + j] >= 0x20 && mem[addr + j] <= 0x7e)
                    printf("%c", mem[addr + j]);
                else
                    printf(" ");
            if (i == size - 1)
                printf("\n");
        }
    }
}

void disasmem(uint8_t *mem, uint64_t size)
{
    printf("Memory@%016lx:\n", (uint64_t)mem);
    uint8_t *p = mem;
    while (p < mem + size)
        if (*p & 3 == 3)
            printf("%08x: %s\n", *(uint32_t *)p, disas(*(uint32_t *)p).c_str()), p += 4;
        else
            printf("    %04hx: %s\n", *(uint16_t *)p, disas(*(uint16_t *)p).c_str()), p += 2;
}

int main(int argc, char *argv[])
{
    /* Read command line */
    cmd_t cmd;
    for (int i = 1; i < argc; i++)
        if (argv[i][0] == '-' && cmd.filename == NULL)
        {
            int j = 1;
            while (argv[i][j] == '-')
                j++;
            if (strcmp(argv[i] + j, "dump") == 0)
                cmd.filetype = 1;
            else if (strcmp(argv[i] + j, "elf") == 0)
                cmd.filetype = 0;
            else if (strcmp(argv[i] + j, "dtb") == 0)
                cmd.dtb = argv[++i];
            else if (strcmp(argv[i] + j, "initrd") == 0)
                cmd.initrd = argv[++i];
            else if (strcmp(argv[i] + j, "t") == 0)
            {
                if (i + 1 < argc)
                    cmd.mintime = atoi(argv[i + 1]);
                if (i + 2 < argc)
                    cmd.maxtime = atoi(argv[i + 2]);
                if (cmd.mintime <= 0)
                    cmd.mintime = 0;
                if (cmd.maxtime <= 0)
                    cmd.maxtime = INT32_MAX;
                i += 2;
            }
            else if (strcmp(argv[i] + j, "d") == 0)
                cmd.help = 1;
            else if (strcmp(argv[i] + j, "h") == 0)
                cmd.help = 1;
        }
        else if (cmd.filename == NULL)
            cmd.args.push_back(cmd.filename = argv[i]);
        else
            cmd.args.push_back(argv[i]);
    if (!cmd.help && cmd.filename == NULL)
        printf("Not enough arguments.\n"), cmd.help = 1;
    if (cmd.help)
    {
        printf("Usage: exec [options] file [arguments]\n");
        printf("Available options:\n");
        printf("    -dump: (force) input file as hex hump\n");
        printf("    -elf: (default) input file as RISC-V ELF executable\n");
        printf("    -dtb `binary`: specify device tree binary.\n");
        printf("    -initrd `binary`: specify initial rootfs.\n");
        printf("    -t `t1` `t2`: simulation time between `t1` and `t2`\n");
        return 0;
    }
    fprintf(stderr, "[Info] Running simulation in %s mode with:\n[Info]     ",
            cmd.filetype == 0 ? "dump" : "elf");
    for (int i = 0; i < cmd.args.size(); i++)
        fprintf(stderr, " %s", cmd.args[i]);
    fprintf(stderr, "\n");

    /* Initialize memory and registers */
    memory mem;
    uint64_t entry;
    if (cmd.filetype == 0) // ELF format
    {
    }
    else if (cmd.filetype == 1) // hex dump code
    {
        entry = 0x80000000;
        FILE *fp = fopen(cmd.filename, "r");
        if (!fp)
            return fprintf(stderr, "Unable to open file '%s'\n", cmd.filename), 1;
        uint64_t size = 0;
        uint32_t inst;
        while (fscanf(fp, "%x", &inst) == 1)
            size++;
        size *= 4;
        uint8_t *buffer;
        if (size > 0x80000000ull || (buffer = new (std::nothrow) uint8_t[size + 4]) == 0)
            return fprintf(stderr, "Require too much memory\n"), 1;
        rewind(fp);
        for (int i = 0; i < size; i += 4)
            if (fscanf(fp, "%x", (uint32_t *)(buffer + i)) != 1)
                return fprintf(stderr, "Read file failed\n"), 1;
        fclose(fp);
        ((uint32_t *)buffer)[size / 4] = 0x6f; // j 0(pc)
        if (!mem.copy(buffer, size + 4, entry))
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
        if (!mem.add(0x1000, 0x10010000)) // data segment
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
    }

    disasmem(&mem[0x80000000], 64);

    /* Simulate */
    status_t s = {.pc = entry, .mem = mem};
    while (true)
    {
        delta_t d = next(s);
        print(s, d);
        s.pc = d.pc;
        s.level = d.level;
        if (d.gprw)
            s.gpr[d.gpra] = d.gprv;
        if (d.memw && s.mem.issegfault(s.mem[d.mema]))
            fprintf(stderr, "[Warning] Attempt to access undefined memory@%lx\n", d.mema);
        else if (d.memw == 1)
            s.mem.ui8(d.mema) = d.memv;
        else if (d.memw == 2)
            s.mem.ui16(d.mema) = d.memv;
        else if (d.memw == 4)
            s.mem.ui32(d.mema) = d.memv;
        else if (d.memw == 8)
            s.mem.ui64(d.mema) = d.memv;
        for (auto i : d.csr)
            s.csr[i.first] = i.second;
        getchar();
    }

    return 0;
}