# Makefile for Octa-Core RV64IMAFDCV Space Processor V7
# Target: Virtex UltraScale+ VU9P (space-grade: XQRVU9P)
# Clock: 125 MHz

VIVADO = vivado
PART = xcvu9p-flga2104-2-e
TOP = octa_core_rv64_soc_integrated

RTL_DIR = ../rtl
BUILD_DIR = build
REPORT_DIR = reports

# RTL Source Files (same as VU095 project - RTL is FPGA-agnostic)
RTL_FILES = \
    $(RTL_DIR)/core/rv64_core.sv \
    $(RTL_DIR)/fpu/fp64_fpu_integrated.sv \
    $(RTL_DIR)/vector/rvv_coprocessor.sv \
    $(RTL_DIR)/dls/dls_core_pair_rv64.sv \
    $(RTL_DIR)/octa_core/octa_core_rv64_soc.sv \
    $(RTL_DIR)/cache/l1_icache_bram.sv \
    $(RTL_DIR)/cache/l1_dcache_bram.sv \
    $(RTL_DIR)/cache/l2_cache_bram.sv \
    $(RTL_DIR)/cache/moesi_controller.sv \
    $(RTL_DIR)/cache/ecc_secded.sv \
    $(RTL_DIR)/cache/snoop_bus.sv \
    $(RTL_DIR)/cache/amo_unit.sv \
    $(RTL_DIR)/cache/memory_barrier.sv \
    $(RTL_DIR)/accelerators/fp64_cordic_unit.sv \
    $(RTL_DIR)/accelerators/systolic_array_3x3.sv \
    $(RTL_DIR)/accelerators/tinyml_subsystem.sv \
    $(RTL_DIR)/accelerators/per_core_accelerators.sv \
    $(RTL_DIR)/compression/compression_subsystem.sv \
    $(RTL_DIR)/compression/ccsds_compression_core.sv \
    $(RTL_DIR)/spacewire/spacewire_codec.sv \
    $(RTL_DIR)/spacewire/spacewire_router.sv \
    $(RTL_DIR)/tsn_ethernet/tsn_mac.sv \
    $(RTL_DIR)/tsn_ethernet/tsn_switch.sv \
    $(RTL_DIR)/interfaces/spi_master.sv \
    $(RTL_DIR)/interfaces/cxl_memory_interface.sv \
    $(RTL_DIR)/debug/debug_module.sv \
    $(RTL_DIR)/debug/jtag_dtm.sv \
    $(RTL_DIR)/memory/prom_controller.sv \
    $(RTL_DIR)/fault_tolerance/watchdog_timer.sv \
    $(RTL_DIR)/interconnect/crossbar_interconnect.sv

CONSTRAINTS = \
    constraints/timing_vu9p.xdc \
    constraints/pins_vu9p.xdc

.PHONY: all synth impl bit reports clean full

all: bit

# Full flow: synth + impl + bitstream using the combined TCL script
full:
	mkdir -p $(BUILD_DIR) $(REPORT_DIR)/synth $(REPORT_DIR)/impl
	$(VIVADO) -mode batch -source scripts/synth_pcpi_soc.tcl \
	    -log $(BUILD_DIR)/synth_impl_full.log \
	    -journal $(BUILD_DIR)/synth_impl_full.jou

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR) $(REPORT_DIR)/synth $(REPORT_DIR)/impl

# Synthesis
synth: $(BUILD_DIR)
	$(VIVADO) -mode batch -source scripts/run_synth.tcl \
	    -tclargs $(PART) $(TOP) \
	    -log $(BUILD_DIR)/synth.log \
	    -journal $(BUILD_DIR)/synth.jou

# Implementation (Place & Route)
impl: synth
	$(VIVADO) -mode batch -source scripts/run_impl.tcl \
	    -tclargs $(PART) $(TOP) \
	    -log $(BUILD_DIR)/impl.log \
	    -journal $(BUILD_DIR)/impl.jou

# Implementation only (from existing synth checkpoint)
impl_only:
	$(VIVADO) -mode batch -source scripts/impl_only.tcl \
	    -log $(BUILD_DIR)/impl_only.log \
	    -journal $(BUILD_DIR)/impl_only.jou

# Generate Bitstream
bit: impl
	$(VIVADO) -mode batch -source scripts/run_bit.tcl \
	    -tclargs $(PART) $(TOP) \
	    -log $(BUILD_DIR)/bit.log \
	    -journal $(BUILD_DIR)/bit.jou

# Clean
clean:
	rm -rf $(BUILD_DIR) $(REPORT_DIR)
	rm -rf vivado.log vivado.jou .Xil
	rm -rf *.dcp *.bit *.bin
