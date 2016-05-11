TARGETS = $(BIN_PATH)/realsense-stop

CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_LCMTYPES)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_LCMTYPES) -lrealsense
DEPS := $(DEPS_STD) $(DEPS_VX) $(DEPS_LCMTYPES)

include $(BUILD_COMMON)


all: $(TARGETS)
	@/bin/true

$(BIN_PATH)/realsense-stop: realsense_stop.o $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(OBJS) $(TARGETS)
