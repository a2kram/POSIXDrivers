SRC_DIR := 
INC_DIR := ../../../
OBJ_DIR := obj/
DRV_DIR := ../
OUT_DIR := $(DRV_DIR)out/

DRV_SRCS_C := $(wildcard $(SRC_DIR)*.c)
DRV_SRCS_CXX := $(wildcard $(SRC_DIR)*.cpp)
DRV_OBJS := $(addprefix $(OBJ_DIR), $(DRV_SRCS_C:%.c=%.o) \
			$(DRV_SRCS_CXX:%.cpp=%.o))

#define these as special targets, meaning run even if a "all" file exists and is update to date
.PHONY: clean

# rule to make target, depend on objects 
# (make will run rule to make objects since dependency)
default: DRV_CNXN

DRV_CNXN: $(OUT_DIR) $(OBJ_DIR) $(DRV_OBJS)
	cp $(OBJ_DIR)* $(OUT_DIR)

#rule to clean output
clean:
	rm $(OBJ_DIR)*

# rule to make objects
$(OBJ_DIR)%.o : %.c
	$(CC) $< $(CFLAGS) -I $(INC_DIR) -c -o $@

$(OBJ_DIR)%.o : %.cpp
	$(CXX) $< $(CFLAGS) -I $(INC_DIR) -c -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)