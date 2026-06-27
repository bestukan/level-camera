SDK_SYSROOT ?= ../../tspi_linux_sdk_20230916/buildroot/output/rockchip_rk3566/host/aarch64-buildroot-linux-gnu/sysroot
SDK_HOST_BIN ?= ../../tspi_linux_sdk_20230916/buildroot/output/rockchip_rk3566/host/bin
CC ?= $(SDK_HOST_BIN)/aarch64-buildroot-linux-gnu-gcc
CFLAGS ?= -Wall -Wextra -std=c11 -O2
SYSROOT_FLAGS ?= --sysroot=$(SDK_SYSROOT)
LIBDRM_CFLAGS ?= -I$(SDK_SYSROOT)/usr/include/libdrm
LIBDRM_LIBS ?= -ldrm

.PHONY: all clean

all: camera_dump_test camera_drm_preview

camera_dump_test: camera_dump_test.c camera_v4l2.c camera_v4l2.h
	$(CC) $(SYSROOT_FLAGS) $(CFLAGS) camera_dump_test.c camera_v4l2.c -o $@

camera_drm_preview: camera_drm_preview.c camera_v4l2.c camera_v4l2.h
	$(CC) $(SYSROOT_FLAGS) $(CFLAGS) $(LIBDRM_CFLAGS) camera_drm_preview.c camera_v4l2.c -o $@ $(LIBDRM_LIBS)

clean:
	$(RM) camera_dump_test camera_drm_preview
