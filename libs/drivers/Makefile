OUT_DIR := out/
CNXN_DIR := connections/
DRV_NAME := libA2K_Drivers
DRV_OUT := $(OUT_DIR)$(DRV_NAME).a
DRV_DEPLIB := DRV_CNXN

#define these as special targets, meaning run even if a "all" file exists and is update to date
.PHONY: $(DRV_DEPLIB) DRIVERS clean all

all: clean default

default: $(OUT_DIR) DRIVERS

DRIVERS: $(DRV_DEPLIB)
	$(AR) rcs $(DRV_OUT) $(OUT_DIR)*.o
	rm $(OUT_DIR)*.o

DRV_CNXN:
	$(MAKE) -C $(CNXN_DIR) $@

clean:
	$(shell rm $(OUT_DIR)* 2> /dev/null)