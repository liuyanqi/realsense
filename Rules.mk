.PHONY: sensor_realsense sensor_realsense_clean

sensor_realsense: lcmtypes vx

sensor_realsense:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/sensor/realsense -f Build.mk

sensor_realsense_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/sensor/realsense -f Build.mk clean

all: sensor_realsense

clean: sensor_realsense_clean
