.PHONY : all clean

# Execute in docker.io/jxy324/verilator:v0

# Verilator variables
MAKE = make -j8
VERILATOR_V = ../../src/mmu.sv
VERILATOR_H = 
VERILATOR_CPP = 
VERILATOR_SRC = $(VERILATOR_V) $(VERILATOR_CPP)
VERILATOR_TOP = cache
VERILATOR_FLAGS = -cc -exe -assert -top-module $(VERILATOR_TOP)\
	-trace -trace-structs -trace-max-array 256

all: run

# Verilator bin target
obj_dir/V$(VERILATOR_TOP) : obj_dir $(VERILATOR_CPP) $(VERILATOR_H)
	$(MAKE) -C obj_dir -f V$(VERILATOR_TOP).mk

# Verilator obj_dir target
obj_dir : $(VERILATOR_V)
	$(RM) -r obj_dir
	/opt/verilator/bin/verilator $(VERILATOR_FLAGS) $(VERILATOR_SRC)

run : obj_dir/V$(VERILATOR_TOP)
	ln -fs obj_dir/V$(VERILATOR_TOP) run

clean :
	$(RM) -r obj_dir
