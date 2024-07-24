#include <cstdio>
#include <cstring>
#include <vector>
#include <elf.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include "status.h"

typedef struct
{
    const char *elf = 0, *dtb = 0, *initrd = 0;
    std::vector<const char *> args;
    uint8_t debug = 0, help = 0, filetype = 0;
    int mintime = 0, maxtime = INT32_MAX;
} cmd_t;

typedef struct
{
    uint64_t fromhost = 0, tohost = 0, lock = 0;
} htifaddr_t;

int interrupt = 0;
void intrhandler(int) { fprintf(stderr, "[Info] Interrupted.\n"), interrupt = 1; }

void dumpmem(const uint8_t *mem, uint64_t base, uint64_t size)
{
    fprintf(stderr, "[Debug] Memory@%016lx:", base);
    for (int i = 0; i < size; i++)
    {
        i % 16 ? fprintf(stderr, i % 2 ? "" : " ") : fprintf(stderr, "\n[Debug]     %08x: ", i);
        fprintf(stderr, "%02x", mem[i]);
        if ((i + 1) % 16 == 0 || i == size - 1)
        {
            if (i == size - 1)
                for (int j = i + 1; j < i / 16 * 16 + 16; j++)
                    fprintf(stderr, j % 2 ? "  " : "   ");
            fprintf(stderr, "  ");
            for (int j = i / 16 * 16; j <= i; j++)
                if (mem[j] >= 0x20 && mem[j] <= 0x7e)
                    fprintf(stderr, "%c", mem[j]);
                else
                    fprintf(stderr, " ");
            if (i == size - 1)
                fprintf(stderr, "\n");
        }
    }
}

void disasmem(const uint8_t *mem, uint64_t size)
{
    fprintf(stderr, "[Debug] Memory@%016lx:\n", (uint64_t)mem);
    const uint8_t *p = mem;
    while (p < mem + size)
        if (*p & 3 == 3)
            fprintf(stderr, "[Debug]     %08x: %08x %s\n",
                    uint32_t(p - mem), *(uint32_t *)p, disas(*(uint32_t *)p).c_str()),
                p += 4;
        else
            fprintf(stderr, "[Debug]     %08x:    %04hx %s\n",
                    uint32_t(p - mem), *(uint16_t *)p, disas(*(uint16_t *)p).c_str()),
                p += 2;
}

void print(uint64_t cycle, status_t &status, const delta_t &delta)
{
    char s[256], lch[4] = {'U', 'S', 'H', 'M'};
    sprintf(s, "[Debug] cycle %ld: %c@%lx: %8x %s", cycle,
            lch[status.level & 3], status.pc, status.ir, disas(status.ir).c_str());
    if (strlen(s) < 63)
    {
        for (int i = strlen(s); i < 63; i++)
            s[i] = ' ';
        s[63] = 0;
    }
    fputs(s, stderr);
    if (delta.gprw)
        fprintf(stderr, " %s: %lx", gprname[delta.gpra], delta.gprv);
    if (delta.memw && delta.memw >> 4 != 0x8)
        fprintf(stderr, " d%d@%lx: %lx", delta.memw & 0xf, delta.mema, delta.memv);
    fprintf(stderr, "\n");
}

void print(status_t &s, uint64_t addr = 0, uint64_t size = 0)
{
    fprintf(stderr, "[Debug] General-purpose registers:\n");
    for (int i = 0; i < 16; i++)
    {
        fprintf(stderr, "[Debug]");
        for (int j = 0; j < 4; j++)
            fprintf(stderr, " %8s: %016lx", gprname[i * 4 + j], (uint64_t)s.gpr[i * 4 + j]);
        fprintf(stderr, "\n");
    }
    if (size)
        dumpmem(&s.mem[addr], addr, size);
}

int main(int argc, char *argv[])
{
    /* Read command line */
    cmd_t cmd;
    for (int i = 1; i < argc; i++)
        if (argv[i][0] == '-' && cmd.elf == NULL)
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
                cmd.debug = 1;
            else if (strcmp(argv[i] + j, "h") == 0)
                cmd.help = 1;
        }
        else if (cmd.elf == NULL)
            cmd.args.push_back(cmd.elf = argv[i]);
        else
            cmd.args.push_back(argv[i]);
    if (!cmd.help && cmd.elf == NULL)
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
    uint64_t entry, dumpsz;
    htifaddr_t htifaddr;
    uint64_t dtbaddr = 0x1020; // -0x80000000 - 0x7fffffff
    uint64_t initrdaddr = 0xfe60fe00;
    if (cmd.filetype == 0)
    {
        /* ELF format */
        /* Read and check ELF header */
        Elf64_Ehdr elf_h; // ELF header
        FILE *fp = fopen(cmd.elf, "r");
        if (!fp)
            return fprintf(stderr, "[Error] Unable to open file %s.\n", cmd.elf), 1;
        if (fread(&elf_h, sizeof(elf_h), 1, fp) < 0)
            return fprintf(stderr, "[Error] Fread failed.\n"), 1;
        if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)) ||
            elf_h.e_ident[EI_CLASS] != ELFCLASS64)
            return fprintf(stderr, "[Error] Not 64-bit ELF format.\n"), 1;
        if (elf_h.e_type != ET_EXEC && elf_h.e_type != ET_DYN)
            return fprintf(stderr, "[Error] Not an executable file.\n"), 1;
        if (elf_h.e_machine != EM_RISCV)
            return fprintf(stderr, "[Error] Not RISC-V architecture.\n"), 1;
        /* sections from ELF file */
        Elf64_Shdr *shdr = new (std::nothrow) Elf64_Shdr[elf_h.e_shnum]; // section headers
        fseek(fp, elf_h.e_shoff, SEEK_SET);
        if (fread(shdr, sizeof(Elf64_Shdr) * elf_h.e_shnum, 1, fp) < 0)
            return fprintf(stderr, "[Error] Fread failed.\n"), 1;
        for (int i = 0; i < elf_h.e_shnum; i++)
            if (shdr[i].sh_flags & SHF_ALLOC)
                if (shdr[i].sh_type == SHT_NOBITS)
                {
                    if (!mem.add(shdr[i].sh_size, shdr[i].sh_addr))
                        return fprintf(stderr, "[Error] Adding memory failed.\n"), 1;
                }
                else
                {
                    fseek(fp, shdr[i].sh_offset, SEEK_SET);
                    if (!mem.read(fp, shdr[i].sh_size, shdr[i].sh_addr))
                        return fprintf(stderr, "[Error] Adding memory from file failed.\n"), 1;
                }
            else if (shdr[i].sh_type == SHT_SYMTAB)
            {
                /* check section name */
                char name[1024];
                fseek(fp, shdr[elf_h.e_shstrndx].sh_offset + shdr[i].sh_name, SEEK_SET);
                if (fscanf(fp, "%1023s", name) <= 0)
                    return fprintf(stderr, "[Error] Fscanf failed.\n"), 1;
                if (strcmp(name, ".symtab") != 0)
                    continue;
                /* read symbol table and search for fromhost and tohost */
                int sym_sz = shdr[i].sh_size / shdr[i].sh_entsize;
                for (int j = 0; j < sym_sz; j++)
                {
                    Elf64_Sym sym;
                    fseek(fp, shdr[i].sh_offset + j * shdr[i].sh_entsize, SEEK_SET);
                    if (fread(&sym, sizeof(sym), 1, fp) < 0)
                        return fprintf(stderr, "[Error] Fread failed.\n"), 1;
                    fseek(fp, shdr[shdr[i].sh_link].sh_offset + sym.st_name, SEEK_SET);
                    if (fscanf(fp, "%1023s", name) <= 0)
                        return fprintf(stderr, "[Error] Fscanf failed.\n"), 1;
                    if (strcmp(name, "fromhost") == 0)
                        htifaddr.fromhost = sym.st_value;
                    else if (strcmp(name, "tohost") == 0)
                        htifaddr.tohost = sym.st_value;
                    else if (strcmp(name, "htif_lock") == 0)
                        htifaddr.lock = sym.st_value;
                }
            }
        if (htifaddr.fromhost == 0 || htifaddr.tohost == 0)
        {
            htifaddr = {0x2000, 0x2008, 0x2010}; // default htif addresses
            mem.add(4096, 0x2000);
            fprintf(stderr, "[Info] HTIF address not specified, set to default.\n");
        }
        delete[] shdr;
        fclose(fp);
        if (cmd.dtb)
        {
            fp = fopen(cmd.dtb, "r");
            if (!fp)
                return fprintf(stderr, "[Error] Unable to open file %s.\n", cmd.dtb), 1;
            fseek(fp, 0, SEEK_END);
            int sz = ftell(fp);
            rewind(fp);
            if (!mem.read(fp, sz, dtbaddr))
                return fprintf(stderr, "[Error] Adding memory from file failed.\n"), 1;
            fclose(fp);
        }
        if (cmd.initrd)
        {
            fp = fopen(cmd.initrd, "r");
            if (!fp)
                return fprintf(stderr, "[Error] Unable to open file %s.\n", cmd.initrd), 1;
            fseek(fp, 0, SEEK_END);
            int sz = ftell(fp);
            rewind(fp);
            if (!mem.read(fp, sz, initrdaddr))
                return fprintf(stderr, "[Error] Adding memory from file failed.\n"), 1;
            fclose(fp);
        }
        /* start section: jump from reset address to ELF entry */
        entry = 0;
        if (!mem.issegfault(mem[entry]))
            entry += 0x1000; // find an unused page
        mem.add(0x1000, entry);
        mem.ui32(entry + 0) = 0x5b7 | dtbaddr & 0xfffff000; // lui a1, `dtbaddr >> 12`
        mem.ui32(entry + 4) = 0x58593 | dtbaddr << 20;      // addi a1, a1, `dtbaddr & 0xfff`
        mem.ui32(entry + 8) = 0x93;                         // addi ra, zero, 0
        for (int i = 0; i < 8; i++)
        {
            mem.ui32(entry + 12 + 8 * i) = 0x8093 | (uint8_t(elf_h.e_entry >> 8 * (7 - i)) << 20);
            mem.ui32(entry + 16 + 8 * i) = (i == 7 ? 0x8067 : 0x809093); // ret : slli ra, ra, 8
        }
    }
    else if (cmd.filetype == 1)
    {
        /* hex dump code */
        entry = 0x80000000;
        dumpsz = 0;
        FILE *fp = fopen(cmd.elf, "r");
        if (!fp)
            return fprintf(stderr, "Unable to open file '%s'\n", cmd.elf), 1;
        uint32_t inst;
        while (fscanf(fp, "%x", &inst) == 1)
            dumpsz++;
        dumpsz *= 4;
        uint8_t *buffer;
        if (dumpsz > 0x80000000ull || (buffer = new (std::nothrow) uint8_t[dumpsz + 4]) == 0)
            return fprintf(stderr, "Require too much memory\n"), 1;
        rewind(fp);
        for (int i = 0; i < dumpsz; i += 4)
            if (fscanf(fp, "%x", (uint32_t *)(buffer + i)) != 1)
                return fprintf(stderr, "Read file failed\n"), 1;
        fclose(fp);
        ((uint32_t *)buffer)[dumpsz / 4] = 0x6f; // j 0(pc)
        if (!mem.copy(buffer, dumpsz += 4, entry))
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
        if (!mem.add(0x1000, 0x10010000)) // data segment
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
        delete[] buffer;
    }

    /* Simulate */
    status_t s = {.pc = entry, .mem = mem};
    uint8_t exitcall = 0, exitcode = 0;
    uint64_t cycle = 0;
    s.mem.add(0xc0000, 0x2000000);                     // CLINT area
    s.mem.ui64(s.csr["mtime"] = 0x200bff8) = 0;        // mtime
    s.mem.ui64(s.csr["mtimecmp"] = 0x2004000) = -1ull; // mtimecmp
    signal(SIGINT, intrhandler);
    while (!interrupt && !exitcall && cycle <= cmd.maxtime)
    {
        /* set interrupts */
        if (cycle % 10 == 0) // increase mtime
            s.mem.ui64(s.csr["mtime"])++;
        s.csr["mip"].write(7, s.mem.ui64(s.csr["mtime"]) >= s.mem.ui64(s.csr["mtimecmp"]));

        /* get next status */
        delta_t d = next(s);
        if (cycle >= cmd.mintime)
            cmd.debug ? print(cycle, s, d), 0 : 0;
        apply(s, d);
        if ((cycle + 1) % 1000000 == 0)
            fprintf(stderr, "[Info] Keep-alive: cycle %d: pc: 0x%lx ir: 0x%x\n",
                    (int)cycle, s.pc, s.ir);

        /* handle HTIF requests */
        uint64_t tohost_dev = s.mem[htifaddr.tohost + 7];
        uint64_t tohost_cmd = s.mem[htifaddr.tohost + 6];
        uint64_t tohost_dat = s.mem.ui64(htifaddr.tohost) & 0xffff'ffff'ffff;
        if (tohost_dev == 0 && tohost_cmd == 0)
        {
            if (tohost_dat & 1) // exit
                exitcall = 1, exitcode = s.mem.ui64(htifaddr.tohost) >> 1;
            else if (tohost_dat != 0) // proxied ststem call
            {
                uint64_t magic_mem = tohost_dat, which = s.mem.ui64(magic_mem);
                uint64_t retval = 0;
                if (which == 0x38) // sysopenat
                {
                    uint64_t arg0, arg1, arg2, arg3, arg4;
                    arg0 = s.mem.ui64(magic_mem + 8);  // directory file descriptor
                    arg1 = s.mem.ui64(magic_mem + 16); // filename
                    arg2 = s.mem.ui64(magic_mem + 24); // filename size
                    arg3 = s.mem.ui64(magic_mem + 32); // flags
                    arg4 = s.mem.ui64(magic_mem + 40); // mode
                    s.mem[arg1 + arg2] = 0;
                    retval = openat(arg0, (char *)&s.mem[arg1], arg3, arg4);
                }
                else if (which == 0x39) // sysclose
                {
                    uint64_t arg0;
                    arg0 = s.mem.ui64(magic_mem + 8); // file descriptor
                    retval = close(arg0);
                }
                else if (which == 0x3e) // syslseek
                {
                    uint64_t arg0, arg1, arg2;
                    arg0 = s.mem.ui64(magic_mem + 8);  // file descriptor
                    arg1 = s.mem.ui64(magic_mem + 16); // pointer
                    arg2 = s.mem.ui64(magic_mem + 24); // directive
                    retval = lseek(arg0, arg1, arg2);
                }
                else if (which == 0x3f) // sysread
                {
                    uint64_t arg0, arg1, arg2;
                    arg0 = s.mem.ui64(magic_mem + 8);  // file descriptor
                    arg1 = s.mem.ui64(magic_mem + 16); // memory address
                    arg2 = s.mem.ui64(magic_mem + 24); // max read size
                    retval = read(arg0, &s.mem[arg1], arg2);
                }
                else if (which == 0x40) // syswrite
                {
                    uint64_t arg0, arg1, arg2;
                    arg0 = s.mem.ui64(magic_mem + 8);  // file descriptor
                    arg1 = s.mem.ui64(magic_mem + 16); // memory address
                    arg2 = s.mem.ui64(magic_mem + 24); // write size
                    fflush(NULL);
                    if (arg0 = 2)
                        arg0 = 1; // redirect stderr of program to stdout for debugging
                    retval = write(arg0, &s.mem[arg1], arg2);
                }
                else if (which == 0x43) // syspread
                {
                    uint64_t arg0, arg1, arg2, arg3;
                    arg0 = s.mem.ui64(magic_mem + 8);  // file descriptor
                    arg1 = s.mem.ui64(magic_mem + 16); // memory address
                    arg2 = s.mem.ui64(magic_mem + 24); // read size
                    arg3 = s.mem.ui64(magic_mem + 32); // read offset
                    retval = pread(arg0, &s.mem[arg1], arg2, arg3);
                }
                else if (which == 0x50) // sysfstat
                {
                    uint64_t arg0, arg1;
                    arg0 = s.mem.ui64(magic_mem + 8);  // file descriptor
                    arg1 = s.mem.ui64(magic_mem + 16); // memory address
                    retval = fstat(arg0, (struct stat *)&s.mem[arg1]);
                }
                else if (which == 0x5d) // exit
                    exitcall = 1, exitcode = s.mem.ui64(magic_mem + 8);
                else if (which == 0x7db) // pk-sysgetmainvars
                {
                    // buffer format: argc(64) argv[0](64) argv[1](64) ...
                    uint64_t arg0, arg1;
                    arg0 = s.mem.ui64(magic_mem + 8);  // argument buffer address
                    arg1 = s.mem.ui64(magic_mem + 16); // argument buffer size
                    s.mem.ui64(arg0) = cmd.args.size();
                    uint64_t addr = arg0 + (cmd.args.size() + 1) * 8;
                    for (int i = 0; i < cmd.args.size(); i++)
                    {
                        s.mem.ui64(arg0 + (i + 1) * 8) = addr;
                        if (addr - arg0 + strlen(cmd.args[i]) + 1 <= arg1)
                            memcpy(&s.mem[addr], cmd.args[i], strlen(cmd.args[i]) + 1);
                        addr += strlen(cmd.args[i]) + 1;
                    }
                    if (addr - arg0 >= arg1)
                        retval = -1;
                }
                else
                    fprintf(stderr, "[Info] Unhandled proxied system call 0x%lx\n", which);
                s.mem.ui64(magic_mem) = retval;
                s.mem.ui64(htifaddr.fromhost) = 1;
            }
        }
        else if (tohost_dev == 1 && tohost_cmd == 1) // console write
            putchar(tohost_dat), fflush(stdout);
        else if (tohost_dev == 1 && tohost_cmd == 0) // console_read
            ;
        else
            fprintf(stderr, "[Info] Unrecognized HTIF command:\n  dev: 0x%lx  cmd: 0x%lx  data: 0x%lx\n",
                    tohost_dev, tohost_cmd, tohost_dat);
        s.mem.ui64(htifaddr.tohost) = 0;
        fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
        char ch; // receive character from stdin
        if (s.mem.ui64(htifaddr.fromhost) == 0 && (ch = getchar()) != EOF)
            s.mem.ui64(htifaddr.fromhost) = (1ull << 56) | ch;
        fcntl(0, F_SETFL, fcntl(0, F_GETFL) & ~O_NONBLOCK);

        /* cycle increment */
        cycle++;
    }
    if (cmd.filetype == 1 && cmd.debug)
        disasmem(&s.mem[entry], dumpsz), print(s, 0x10010000, 256);

    fprintf(stderr, "[Info] Exited with code %hhu at cycle %lu\n", exitcode, (uint64_t)s.csr["mcycle"]);
    return exitcode;
}
