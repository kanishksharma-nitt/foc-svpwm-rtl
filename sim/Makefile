# Targets: run (default), build, lint, golden, clean. Run from sim/.
.DEFAULT_GOAL := run

TOP    = tb_foc
VVP    = sim.vvp
PYTHON ?= python3

build:
	iverilog -g2012 -Wall -s $(TOP) -o $(VVP) -f files.f

run: build
	@out=$$(vvp $(VVP)); \
	echo "$$out"; \
	echo "$$out" | grep -q "TEST PASSED"

lint:
	verilator --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSED \
	  --top-module foc_top \
	  ../rtl/sincos.v ../rtl/clarke.v ../rtl/park.v ../rtl/inv_park.v \
	  ../rtl/pi_ctrl.v ../rtl/svpwm.v ../rtl/deadtime.v ../rtl/foc_top.v

# regenerate rtl/sin_lut.mem and test/foc_vectors.mem (stdlib only)
golden:
	cd .. && $(PYTHON) python/foc_model.py

clean:
	rm -f $(VVP) *.vcd
