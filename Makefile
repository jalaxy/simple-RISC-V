.PHONY: verilator

clean:
	$(RM) -r obj_dir

######################## Verilator #############################

# Execute in docker.io/jxy324/verilator:v0

# Verilator variables
VERILATOR_V=src/core.v tb/verilator/core_tb.v
VERILATOR_CPP=tb/verilator/core_tb.cpp
VERILATOR_SRC=$(VERILATOR_V) $(VERILATOR_CPP)
VERILATOR_TOP=core_tb
VERILATOR_FLAGS=--cc --exe --trace --top-module $(VERILATOR_TOP)

verilator: obj_dir/V$(VERILATOR_TOP)

# Verilator bin target
obj_dir/V$(VERILATOR_TOP): obj_dir $(VERILATOR_CPP)
	make -C obj_dir -f V$(VERILATOR_TOP).mk

# Verilator obj_dir target
obj_dir: $(VERILATOR_V)
	$(RM) -r obj_dir
	/opt/verilator/bin/verilator $(VERILATOR_FLAGS) $(VERILATOR_SRC)

################################################################
