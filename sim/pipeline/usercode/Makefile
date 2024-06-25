DOCKER = docker
CC = $(DOCKER) run --rm -it -v `pwd`:/work -w /work \
	docker.io/jxy324/riscv-toolchain:v0 /opt/riscv/bin/riscv64-unknown-elf-gcc
CFLAGS = #-O2
PROG = $(patsubst %.c, %, $(wildcard *.c))

all : $(PROG)

clean :
	$(RM) $(PROG)
