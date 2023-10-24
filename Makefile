DOCKER=docker

# Verilator variables
VERILATOR_IMAGE=jxy324/verilator:v0
VERILATOR_SRC=src/core.v
VERILATOR_TOP=core

VERILATOR_FLAGS=--cc --top-module $(VERILATOR_TOP)
VERILATOR_CMD=/opt/verilator/bin/verilator $(VERILATOR_FLAGS) $(VERILATOR_SRC)

# Verilator obj_dir target
obj_dir: $(VERILATOR_SRC)
	$(RM) -r obj_dir
	$(DOCKER) run --rm -w /work -v .:/work $(VERILATOR_IMAGE) $(VERILATOR_CMD)

clean:
	$(RM) -r obj_dir
