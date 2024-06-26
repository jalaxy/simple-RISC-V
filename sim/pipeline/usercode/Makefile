CC = $(RISCV)/bin/riscv64-unknown-elf-gcc
CFLAGS = #-O2
PROG = $(patsubst %.c, %, $(wildcard *.c))

all : $(PROG)

clean :
	$(RM) $(PROG)
