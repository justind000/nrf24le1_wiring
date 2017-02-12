CC         := sdcc --verbose
PACKIHX    := packihx
CFLAGS     := -L ./lib/* --std-sdcc99 --model-large
LFLAGS     := --iram-size 256 --code-loc 0x0000 --code-size 0x4000 --xram-loc 0x0000 --xram-size 0x400
MAIN       := main

FLASHER    := nrf24le1_flasher
FLASH_BP    := nrfprog/nrfprog
FLASH_BP_PORT   ?= /dev/ttyUSB0

SDK_DIR            := nRF24LE1_SDK
PINS               := 32
INCLUDE            += -I include -I $(SDK_DIR)/include -I $(SDK_DIR)/_target_sdcc_nrf24le1_$(PINS)/include
LIBS               += -L $(SDK_DIR)/_target_sdcc_nrf24le1_$(PINS)/lib -lnrf24le1
REL_EXTERNAL_DIR   := $(SDK_DIR)/_target_sdcc_nrf24le1_$(PINS)/obj
REL_EXTERNAL_FILES += $(REL_EXTERNAL_DIR)/delay/delay_us.rel
REL_EXTERNAL_FILES += $(REL_EXTERNAL_DIR)/delay/delay_ms.rel
REL_EXTERNAL_FILES += $(REL_EXTERNAL_DIR)/delay/delay_s.rel

REL_SRC := $(MAIN).c
REL_OBJ := $(patsubst %.c,%.rel,$(REL_SRC))

all: rel build
tools: sdk flasher

%.rel: %.c
	$(CC) -c $(INCLUDE) $(CFLAGS) $(LFLAGS) $(LIBS) $^

rel: $(REL_OBJ)

build:
	$(CC) $(CFLAGS) $(LFLAGS) $(LIBS) $(REL_OBJ) $(REL_EXTERNAL_FILES)
	$(PACKIHX) $(MAIN).ihx > $(MAIN).hex
	tail -n5 $(MAIN).mem

clean:
	$(RM) *.asm *.cdb *.hex *.ihx *.lk *.lst *.map *.mem *.omf *.rel *.rst *.sym *.img

test:
	@echo $(REL_EXTERNAL_FILES)

sdk:
	cd $(SDK_DIR) && make

flasher:
	cd $(FLASHER) && make

flash:
	 $(FLASHER)/$(FLASHER) -c -w $(MAIN).ihx

nrfprog_buspirate:
	make -C nrfprog    

flash_buspirate:
	$(FLASH_BP) $(FLASH_BP_PORT) $(MAIN).ihx

reset_ftdi:
	sudo modprobe -r ftdi_sio
	sudo modprobe ftdi_sio
