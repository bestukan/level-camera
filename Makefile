SDK_SYSROOT ?= ../../tspi_linux_sdk_20230916/buildroot/output/rockchip_rk3566/host/aarch64-buildroot-linux-gnu/sysroot
SDK_HOST_BIN ?= ../../tspi_linux_sdk_20230916/buildroot/output/rockchip_rk3566/host/bin
ifeq ($(origin CC),default)
CC := $(SDK_HOST_BIN)/aarch64-buildroot-linux-gnu-gcc
endif
CFLAGS ?= -Wall -Wextra -std=c11 -O2
SYSROOT_FLAGS ?= --sysroot=$(SDK_SYSROOT)
CPPFLAGS ?=
LIBDRM_CFLAGS ?= -I$(SDK_SYSROOT)/usr/include/libdrm
LIBDRM_LIBS ?= -ldrm
RGA_CFLAGS ?= -I$(SDK_SYSROOT)/usr/include/rga
RGA_LIBS ?= -lrga
EGL_GBM_LIBS ?= -lmali_hook -Wl,--whole-archive -lmali_hook_injector -Wl,--no-whole-archive -lmali -lwayland-client -lwayland-server -ldrm

BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin
SRC_DIR := src
INCLUDE_DIR := include
APP_DIR := apps
TOOL_DIR := tools
TEST_DIR := tests

COMMON_CFLAGS := $(SYSROOT_FLAGS) $(CPPFLAGS) $(CFLAGS) -I$(INCLUDE_DIR)

CAMERA_V4L2_SRC := $(SRC_DIR)/camera_v4l2.c
CAMERA_V4L2_HDR := $(INCLUDE_DIR)/camera_v4l2.h
IMU_ATTITUDE_SRC := $(SRC_DIR)/imu_attitude.c
IMU_ATTITUDE_HDR := $(INCLUDE_DIR)/imu_attitude.h

PROGRAMS := \
	$(BIN_DIR)/camera_dump_test \
	$(BIN_DIR)/camera_drm_preview \
	$(BIN_DIR)/egl_gbm_probe \
	$(BIN_DIR)/imu_attitude_test

.PHONY: all clean camera_dump_test camera_drm_preview egl_gbm_probe imu_attitude_test

all: $(PROGRAMS)

camera_dump_test: $(BIN_DIR)/camera_dump_test
camera_drm_preview: $(BIN_DIR)/camera_drm_preview
egl_gbm_probe: $(BIN_DIR)/egl_gbm_probe
imu_attitude_test: $(BIN_DIR)/imu_attitude_test

$(BIN_DIR):
	mkdir -p $@

$(BIN_DIR)/camera_dump_test: $(TEST_DIR)/camera_dump_test.c $(CAMERA_V4L2_SRC) $(CAMERA_V4L2_HDR) | $(BIN_DIR)
	$(CC) $(COMMON_CFLAGS) $< $(CAMERA_V4L2_SRC) -o $@

$(BIN_DIR)/camera_drm_preview: $(APP_DIR)/camera_drm_preview.c $(CAMERA_V4L2_SRC) $(CAMERA_V4L2_HDR) | $(BIN_DIR)
	$(CC) $(COMMON_CFLAGS) $(LIBDRM_CFLAGS) $(RGA_CFLAGS) $< $(CAMERA_V4L2_SRC) -o $@ $(LIBDRM_LIBS) $(RGA_LIBS)

$(BIN_DIR)/egl_gbm_probe: $(TOOL_DIR)/egl_gbm_probe.c | $(BIN_DIR)
	$(CC) $(COMMON_CFLAGS) $(LIBDRM_CFLAGS) $< -o $@ $(EGL_GBM_LIBS)

$(BIN_DIR)/imu_attitude_test: $(TEST_DIR)/imu_attitude_test.c $(IMU_ATTITUDE_SRC) $(IMU_ATTITUDE_HDR) | $(BIN_DIR)
	$(CC) $(COMMON_CFLAGS) $< $(IMU_ATTITUDE_SRC) -o $@ -lm

clean:
	$(RM) -r $(BUILD_DIR)
