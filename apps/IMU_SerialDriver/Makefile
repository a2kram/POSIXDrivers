SRC_DIR := 
INC_DIR := ../../common
CMN_DIR := ../../../common
OBJ_DIR := obj/
BIN_DIR := bin/
DRV_DIR := ../../libs/drivers
LIB_DIR := ../../libs/libraries
OUT_DRV_DIR := $(DRV_DIR)/out/
OUT_LIB_DIR := $(LIB_DIR)/out/
LIBS_LIB := A2K_Libraries
DRV_LIB := A2K_Drivers

APP_OUT := $(BIN_DIR)IMU_SerialDriver
APP_SRCS_C := $(wildcard $(SRC_DIR)*.c)
APP_SRCS_CXX := $(wildcard $(SRC_DIR)*.cpp)
APP_OBJS := $(addprefix $(OBJ_DIR), $(APP_SRCS_C:%.c=%.o) \
			$(APP_SRCS_CXX:%.cpp=%.o))
APP_DRV_DEPLIB := DRIVERS
APP_LIB_DEPLIB := LIBRARIES

LFLAGS := -L $(OUT_DRV_DIR) -L $(OUT_LIB_DIR) -l$(DRV_LIB)
IFLAGS := -I $(INC_DIR) -I $(CMN_DIR) -I $(DRV_DIR) -I $(LIB_DIR)

#define these as special targets, meaning run even if a "all" file exists and is update to date
.PHONY: $(APP_DEPLIB) clean upload

## RULES ##

# rule to make target, depend on objects 
# (make will run rule to make objects since dependency)
default: $(APP_OUT)

$(APP_OUT): $(OBJ_DIR) $(BIN_DIR) $(APP_DRV_DEPLIB) $(APP_OBJS)
	$(CXX) $(CFLAGS) $(IFLAGS) -o $@ $(APP_OBJS) $(LFLAGS) 
	$(shell rm *.o 2> /dev/null)

upload: $(APP_HEX)
	$(UPD_TOOL) -V -F -p m328p -c arduino -b 115200 -Uflash:w:$(APP_HEX) -P$(PORT)

#rule to clean application
clean:
	rm $(OBJ_DIR)*
	rm $(APP_OUT)


## DEPENDENCIES ##

# rule to make objects
$(OBJ_DIR)%.o : %.c
	$(CC) $< $(CFLAGS) $(IFLAGS) -c -o $@

$(OBJ_DIR)%.o : %.cpp
	$(CXX) $< $(CFLAGS) $(IFLAGS) -c -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# rule to make drivers library
$(APP_DRV_DEPLIB):
	$(MAKE) -C $(DRV_DIR) $@

# rule to make libraries
$(APP_LIB_DEPLIB):
	$(MAKE) -C $(LIB_DIR) $@