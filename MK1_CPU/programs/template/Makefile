# MK1 project Makefile.
#
# Drop a .c file in this directory, edit `PROGRAM` below if you renamed
# main.c, then:
#
#   make           build (compile C → asm)
#   make run       upload to MK1 + RUN (single-OI capture, returns first out() value)
#   make runhz     upload + RUNHZ at $(HZ) Hz, capture all OI events with heartbeats
#   make runlog    upload + RUNLOG at $(US)us half-period, capture full OI trace
#   make upload    build + upload only (no run)
#   make sim       compile + simulate (no hardware needed)
#   make reset     pulse RESET on the MK1 CPU
#   make clean     remove build artifacts
#   make help      print this list with current variable values
#
# Override any variable on the command line, e.g.:
#   make run CYCLES=1000000
#   make runhz HZ=200000 MAXOI=100
#   make PROGRAM=demo.c upload
#   make run PORT=/dev/cu.usbmodem...

PROGRAM ?= main.c
TOOLS   ?= ../../code
PORT    ?= /dev/cu.usbmodem3C8427C2E7C82
CYCLES  ?= 5000000
HZ      ?= 145000
US      ?= 2
MAXOI   ?= 0
TIMEOUT ?= 60
PYTHON  ?= python3
OPT     ?= -O
EEPROM  ?=

ASM := $(PROGRAM:.c=.asm)
COMPILER  := $(TOOLS)/mk1cc2.py
UPLOADER  := $(TOOLS)/mk1_upload.py
RUNHZ     := $(TOOLS)/runhz_runner.py
SIM       := $(TOOLS)/verify_via_sim.py
HW_REG    := $(TOOLS)/hw_regression.py

EEPROM_FLAG = $(if $(EEPROM),--eeprom,)

.PHONY: all build upload run runhz runlog sim reset clean help

all: build

build: $(ASM)

$(ASM): $(PROGRAM)
	$(PYTHON) $(COMPILER) $(PROGRAM) $(OPT) $(EEPROM_FLAG) -o $(ASM)

upload: $(ASM)
	$(PYTHON) $(UPLOADER) $(ASM) --port $(PORT) --timeout $(TIMEOUT)

run: $(ASM)
	$(PYTHON) $(UPLOADER) $(ASM) --run $(CYCLES) --port $(PORT) --timeout $(TIMEOUT)

runhz: $(PROGRAM)
	$(PYTHON) $(RUNHZ) $(PROGRAM) $(EEPROM_FLAG) --cycles $(CYCLES) --hz $(HZ) --maxoi $(MAXOI) --timeout $(TIMEOUT)

# RUNLOG via the firmware: captures full OI history (up to 256 events, ring buffer)
# at us-microsecond half-period clocking. No heartbeat handling — the run finishes
# in one shot. Good for short programs that halt cleanly.
runlog: $(ASM)
	@$(PYTHON) -c "import serial, json, sys, time; \
		ser = serial.Serial('$(PORT)', 115200, timeout=5); \
		time.sleep(0.2); ser.reset_input_buffer(); \
		ser.write(b'RESET\n'); ser.readline(); time.sleep(0.2); \
		asm = open('$(ASM)').read().replace('\n', '\\\\n'); \
		ser.write(f'ASM:{asm}\n'.encode()); time.sleep(0.1); \
		r = json.loads(ser.readline().decode().strip()); \
		assert r.get('ok'), f'ASM failed: {r}'; \
		ser.write(b'UPLOAD\n'); ser.readline(); \
		ser.timeout = $(TIMEOUT); \
		ser.write(b'RUNLOG:$(CYCLES),$(US),0\n'); \
		print(json.dumps(json.loads(ser.readline().decode().strip()), indent=2)); \
		ser.close()"

sim: $(PROGRAM)
	$(PYTHON) $(SIM) $(abspath $(PROGRAM)) --cycles $(CYCLES)

reset:
	@$(PYTHON) -c "import serial, time; \
		ser = serial.Serial('$(PORT)', 115200, timeout=2); \
		time.sleep(0.2); ser.reset_input_buffer(); \
		ser.write(b'RESET\n'); print(ser.readline().decode().strip()); ser.close()"

clean:
	rm -f $(ASM)

help:
	@echo "MK1 Makefile targets:"
	@echo "  make            build $(ASM) from $(PROGRAM)"
	@echo "  make upload     build + upload to MK1 (no run)"
	@echo "  make run        upload + RUN ($(CYCLES) cycles, single OI capture)"
	@echo "  make runhz      upload + RUNHZ ($(HZ)Hz, maxoi=$(MAXOI), $(CYCLES) cycles)"
	@echo "  make runlog     upload + RUNLOG (us=$(US), $(CYCLES) cycles, full trace)"
	@echo "  make sim        run $(PROGRAM) in simulator"
	@echo "  make reset      pulse the MK1 reset line"
	@echo "  make clean      remove $(ASM)"
	@echo ""
	@echo "Variables (override with VAR=value):"
	@echo "  PROGRAM = $(PROGRAM)         source file"
	@echo "  TOOLS   = $(TOOLS)        path to MK1 toolchain (mk1cc2.py etc.)"
	@echo "  PORT    = $(PORT)"
	@echo "  CYCLES  = $(CYCLES)"
	@echo "  HZ      = $(HZ)             clock for RUNHZ"
	@echo "  US      = $(US)             half-period (us) for RUN/RUNLOG"
	@echo "  MAXOI   = $(MAXOI)             0 = unlimited (run until halt or cycles)"
	@echo "  TIMEOUT = $(TIMEOUT)            seconds to wait for serial reply"
	@echo "  OPT     = $(OPT)            pass empty (OPT=) to disable optimization"
	@echo "  EEPROM  = $(EEPROM)             set to 1 to enable --eeprom overlay mode"
