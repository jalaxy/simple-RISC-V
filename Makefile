DOCKER=docker

# Verilator variables
VERILATOR_IMAGE=docker.io/jxy324/verilator:v0
VERILATOR_V=src/core.v tb/verilator/core_tb.v
VERILATOR_CPP=tb/verilator/core_tb.cpp
VERILATOR_SRC=$(VERILATOR_V) $(VERILATOR_CPP)
VERILATOR_TOP=core_tb

VERILATOR_FLAGS=--cc --exe --trace --top-module $(VERILATOR_TOP)
VERILATOR_TOCPP=/opt/verilator/bin/verilator $(VERILATOR_FLAGS) $(VERILATOR_SRC)
VERILATOR_COMPILE=make -C obj_dir -f V$(VERILATOR_TOP).mk

# Verilator bin target
obj_dir/V$(VERILATOR_TOP): obj_dir $(VERILATOR_CPP)
	$(DOCKER) run --rm -w /work -v .:/work $(VERILATOR_IMAGE) $(VERILATOR_COMPILE)

# Verilator obj_dir target
obj_dir: $(VERILATOR_V)
	$(RM) -r obj_dir
	$(DOCKER) run --rm -w /work -v .:/work $(VERILATOR_IMAGE) $(VERILATOR_TOCPP)

clean:
	$(RM) -r obj_dir
