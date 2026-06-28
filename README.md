# level_camera

Prototype code for a Linux level-camera pipeline on RK3566.

## Layout

- `include/`: public headers shared by tests, tools, and apps.
- `src/`: reusable camera and IMU implementation files.
- `tests/`: hardware bring-up and focused validation programs.
- `tools/`: environment/probe utilities.
- `apps/`: runnable preview/application prototypes.
- `build/bin/`: generated executables.

## Build

```sh
make
```

Generated binaries are written to `build/bin/`.

Individual targets are also available:

```sh
make camera_dump_test
make imu_attitude_test
make egl_gbm_probe
make camera_drm_preview
```

## Current Programs

- `camera_dump_test`: validates V4L2 capture and frame metadata.
- `imu_attitude_test`: validates sysfs IMU readout and attitude calculation.
- `egl_gbm_probe`: checks GBM/EGL/GLES and dmabuf import support.
- `camera_drm_preview`: displays V4L2 NV12 dmabuf frames through DRM/KMS.

## Next Step

Add the first OpenGL ES preview demo under `apps/`, for example:

```text
apps/camera_gles_preview.c
```

Initial goal:

```text
V4L2 DQBUF -> dmabuf/EGLImage -> GLES shader draw -> eglSwapBuffers -> glFinish -> V4L2 QBUF
```
