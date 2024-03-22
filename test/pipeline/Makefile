.PHONY: all

clean:
	$(RM) -r obj_dir

######################## Verilator #############################

# Execute in docker.io/jxy324/verilator:v0

# Verilator variables
VERILATOR_V=../../src/pipeline.sv
VERILATOR_CPP=test.cpp
VERILATOR_SRC=$(VERILATOR_V) $(VERILATOR_CPP)
VERILATOR_TOP=pipeline
VERILATOR_FLAGS=-cc -exe -assert -top-module $(VERILATOR_TOP)\
	-trace -trace-structs -trace-max-array 256

all: obj_dir/V$(VERILATOR_TOP)

# Verilator bin target
obj_dir/V$(VERILATOR_TOP): obj_dir $(VERILATOR_CPP)
	make -C obj_dir -f V$(VERILATOR_TOP).mk

# Verilator obj_dir target
obj_dir: $(VERILATOR_V)
	$(RM) -r obj_dir
	/opt/verilator/bin/verilator $(VERILATOR_FLAGS) $(VERILATOR_SRC)

################################################################
