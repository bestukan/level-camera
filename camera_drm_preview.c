#include "camera_v4l2.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#ifndef DRM_CLIENT_CAP_UNIVERSAL_PLANES
#define DRM_CLIENT_CAP_UNIVERSAL_PLANES 2
#endif

#ifndef DRM_CLIENT_CAP_ATOMIC
#define DRM_CLIENT_CAP_ATOMIC 3
#endif

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

#define MAX_DRM_PLANES 16

typedef struct {
    uint32_t fb_id;
    uint32_t handles[4];
    unsigned int handle_count;
} drm_fb_t;

typedef struct {
    int fd;
    uint32_t connector_id;
    uint32_t crtc_id;
    uint32_t crtc_index;
    uint32_t plane_id;
    uint32_t plane_ids[MAX_DRM_PLANES];
    unsigned int plane_count;
    drmModeModeInfo mode;
    drmModeCrtc *old_crtc;

    uint32_t black_fb_id;
    uint32_t black_handle;
    uint32_t black_pitch;
    uint64_t black_size;

    drm_fb_t frame_fbs[CAMERA_V4L2_MAX_BUFFERS];
} drm_preview_t;

static void drm_preview_close(drm_preview_t *drm);

static volatile sig_atomic_t g_running = 1;

static void on_signal(int signo)
{
    (void)signo;
    g_running = 0;
}

static unsigned int parse_fourcc(const char *s)
{
    if (!s || strlen(s) != 4)
        return V4L2_PIX_FMT_NV12;

    return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

static int drm_ioctl_retry(int fd, unsigned long request, void *arg)
{
    int ret;

    do {
        ret = ioctl(fd, request, arg);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

static int plane_supports_format(const drmModePlane *plane, uint32_t format)
{
    uint32_t i;

    for (i = 0; i < plane->count_formats; i++) {
        if (plane->formats[i] == format)
            return 1;
    }

    return 0;
}

static const char *drm_fourcc_name(uint32_t format, char out[5])
{
    out[0] = (char)(format & 0xff);
    out[1] = (char)((format >> 8) & 0xff);
    out[2] = (char)((format >> 16) & 0xff);
    out[3] = (char)((format >> 24) & 0xff);
    out[4] = '\0';

    return out;
}

static int choose_connector_crtc(drm_preview_t *drm)
{
    drmModeRes *res;
    drmModeConnector *conn = NULL;
    drmModeEncoder *enc = NULL;
    int i;
    int j;
    int ret = -1;

    res = drmModeGetResources(drm->fd);
    if (!res)
        return -1;

    for (i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(drm->fd, res->connectors[i]);
        if (!conn)
            continue;

        if (conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0)
            break;

        drmModeFreeConnector(conn);
        conn = NULL;
    }

    if (!conn)
        goto out_res;

    drm->connector_id = conn->connector_id;
    drm->mode = conn->modes[0];

    if (conn->encoder_id)
        enc = drmModeGetEncoder(drm->fd, conn->encoder_id);

    if (enc && enc->crtc_id) {
        drm->crtc_id = enc->crtc_id;
        for (i = 0; i < res->count_crtcs; i++) {
            if (res->crtcs[i] == drm->crtc_id) {
                drm->crtc_index = i;
                ret = 0;
                goto out;
            }
        }
    }

    if (enc) {
        for (i = 0; i < res->count_crtcs; i++) {
            if (enc->possible_crtcs & (1 << i)) {
                drm->crtc_id = res->crtcs[i];
                drm->crtc_index = i;
                ret = 0;
                goto out;
            }
        }
    }

    for (i = 0; i < conn->count_encoders; i++) {
        drmModeEncoder *try_enc;

        try_enc = drmModeGetEncoder(drm->fd, conn->encoders[i]);
        if (!try_enc)
            continue;

        for (j = 0; j < res->count_crtcs; j++) {
            if (try_enc->possible_crtcs & (1 << j)) {
                drm->crtc_id = res->crtcs[j];
                drm->crtc_index = j;
                drmModeFreeEncoder(try_enc);
                ret = 0;
                goto out;
            }
        }

        drmModeFreeEncoder(try_enc);
    }

out:
    if (enc)
        drmModeFreeEncoder(enc);
    drmModeFreeConnector(conn);
out_res:
    drmModeFreeResources(res);
    return ret;
}

static int choose_nv12_planes(drm_preview_t *drm)
{
    drmModePlaneRes *planes;
    uint32_t i;
    int ret = -1;

    planes = drmModeGetPlaneResources(drm->fd);
    if (!planes)
        return -1;

    for (i = 0; i < planes->count_planes; i++) {
        drmModePlane *plane = drmModeGetPlane(drm->fd, planes->planes[i]);
        char fmt[5];

        if (!plane)
            continue;

        printf("DRM plane candidate: id=%u crtc_id=%u possible_crtcs=0x%x "
               "supports_nv12=%d first_fmt=%s count_formats=%u\n",
               plane->plane_id,
               plane->crtc_id,
               plane->possible_crtcs,
               plane_supports_format(plane, DRM_FORMAT_NV12),
               plane->count_formats ?
                    drm_fourcc_name(plane->formats[0], fmt) : "none",
               plane->count_formats);

        if ((plane->possible_crtcs & (1 << drm->crtc_index)) &&
            plane_supports_format(plane, DRM_FORMAT_NV12)) {
            if (drm->plane_count < MAX_DRM_PLANES)
                drm->plane_ids[drm->plane_count++] = plane->plane_id;

            if (!drm->plane_id)
                drm->plane_id = plane->plane_id;

            ret = 0;
        }

        drmModeFreePlane(plane);
    }

    drmModeFreePlaneResources(planes);
    if (ret == 0)
        printf("DRM: found %u NV12 plane candidate(s), first=%u\n",
               drm->plane_count,
               drm->plane_id);

    return ret;
}

static int create_black_fb(drm_preview_t *drm)
{
    struct drm_mode_create_dumb create_arg;
    struct drm_mode_destroy_dumb destroy_arg;
    uint32_t depth = 24;
    uint32_t bpp = 32;
    int ret;

    memset(&create_arg, 0, sizeof(create_arg));
    create_arg.width = drm->mode.hdisplay;
    create_arg.height = drm->mode.vdisplay;
    create_arg.bpp = bpp;

    ret = drm_ioctl_retry(drm->fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_arg);
    if (ret < 0)
        return -1;

    drm->black_handle = create_arg.handle;
    drm->black_pitch = create_arg.pitch;
    drm->black_size = create_arg.size;

    ret = drmModeAddFB(drm->fd,
                       drm->mode.hdisplay,
                       drm->mode.vdisplay,
                       depth,
                       bpp,
                       drm->black_pitch,
                       drm->black_handle,
                       &drm->black_fb_id);
    if (ret < 0) {
        memset(&destroy_arg, 0, sizeof(destroy_arg));
        destroy_arg.handle = drm->black_handle;
        drm_ioctl_retry(drm->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy_arg);
        drm->black_handle = 0;
        return -1;
    }

    return 0;
}

static int drm_preview_open(drm_preview_t *drm, const char *dev_name)
{
    int ret;

    memset(drm, 0, sizeof(*drm));
    drm->fd = -1;

    drm->fd = open(dev_name, O_RDWR | O_CLOEXEC);
    if (drm->fd < 0)
        return -1;

    drmSetClientCap(drm->fd, DRM_CLIENT_CAP_ATOMIC, 1);
    drmSetClientCap(drm->fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);

    if (choose_connector_crtc(drm) < 0) {
        fprintf(stderr, "no connected DRM connector/crtc found\n");
        goto fail;
    }

    drm->old_crtc = drmModeGetCrtc(drm->fd, drm->crtc_id);

    if (choose_nv12_planes(drm) < 0) {
        fprintf(stderr, "no DRM plane supports NV12 for selected CRTC\n");
        goto fail;
    }

    if (create_black_fb(drm) < 0) {
        fprintf(stderr, "create black CRTC framebuffer failed: %s\n",
                strerror(errno));
        goto fail;
    }

    ret = drmModeSetCrtc(drm->fd,
                         drm->crtc_id,
                         drm->black_fb_id,
                         0,
                         0,
                         &drm->connector_id,
                         1,
                         &drm->mode);
    if (ret < 0) {
        fprintf(stderr, "drmModeSetCrtc failed: %s\n", strerror(errno));
        goto fail;
    }

    printf("DRM: connector=%u crtc=%u plane=%u mode=%ux%u\n",
           drm->connector_id,
           drm->crtc_id,
           drm->plane_id,
           drm->mode.hdisplay,
           drm->mode.vdisplay);

    return 0;

fail:
    drm_preview_close(drm);
    return -1;
}

static void close_drm_handle(int fd, uint32_t handle)
{
    struct drm_gem_close close_arg;

    if (!handle)
        return;

    memset(&close_arg, 0, sizeof(close_arg));
    close_arg.handle = handle;
    drm_ioctl_retry(fd, DRM_IOCTL_GEM_CLOSE, &close_arg);
}

static void drm_fb_destroy(drm_preview_t *drm, drm_fb_t *fb)
{
    unsigned int i;

    if (!fb || !fb->fb_id)
        return;

    drmModeRmFB(drm->fd, fb->fb_id);
    fb->fb_id = 0;

    for (i = 0; i < fb->handle_count; i++) {
        close_drm_handle(drm->fd, fb->handles[i]);
        fb->handles[i] = 0;
    }
    fb->handle_count = 0;
}

static void drm_preview_close(drm_preview_t *drm)
{
    struct drm_mode_destroy_dumb destroy_arg;
    unsigned int i;

    if (!drm || drm->fd < 0)
        return;

    if (drm->plane_id)
        drmModeSetPlane(drm->fd, drm->plane_id, drm->crtc_id, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0);

    if (drm->old_crtc) {
        drmModeSetCrtc(drm->fd,
                       drm->old_crtc->crtc_id,
                       drm->old_crtc->buffer_id,
                       drm->old_crtc->x,
                       drm->old_crtc->y,
                       &drm->connector_id,
                       1,
                       &drm->old_crtc->mode);
        drmModeFreeCrtc(drm->old_crtc);
        drm->old_crtc = NULL;
    }

    for (i = 0; i < CAMERA_V4L2_MAX_BUFFERS; i++)
        drm_fb_destroy(drm, &drm->frame_fbs[i]);

    if (drm->black_fb_id) {
        drmModeRmFB(drm->fd, drm->black_fb_id);
        drm->black_fb_id = 0;
    }

    if (drm->black_handle) {
        memset(&destroy_arg, 0, sizeof(destroy_arg));
        destroy_arg.handle = drm->black_handle;
        drm_ioctl_retry(drm->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy_arg);
        drm->black_handle = 0;
    }

    close(drm->fd);
    drm->fd = -1;
}

static int import_fd_once(drm_preview_t *drm,
                          drm_fb_t *fb,
                          int dmabuf_fd,
                          uint32_t *handle)
{
    unsigned int i;

    for (i = 0; i < fb->handle_count; i++) {
        if (*handle == fb->handles[i])
            return 0;
    }

    if (drmPrimeFDToHandle(drm->fd, dmabuf_fd, handle) < 0)
        return -1;

    fb->handles[fb->handle_count++] = *handle;
    return 0;
}

static int frame_to_drm_fb(drm_preview_t *drm,
                           const camera_v4l2_frame_t *frame,
                           uint32_t *fb_id)
{
    drm_fb_t *fb;
    uint32_t handles[4] = {0};
    uint32_t pitches[4] = {0};
    uint32_t offsets[4] = {0};
    int ret;

    if (!drm || !frame || !fb_id || frame->index >= CAMERA_V4L2_MAX_BUFFERS) {
        errno = EINVAL;
        return -1;
    }

    if (frame->pixelformat != V4L2_PIX_FMT_NV12) {
        fprintf(stderr, "DRM preview only supports NV12, got fourcc=0x%x\n",
                frame->pixelformat);
        errno = ENOTSUP;
        return -1;
    }

    fb = &drm->frame_fbs[frame->index];
    if (fb->fb_id) {
        *fb_id = fb->fb_id;
        return 0;
    }

    if (frame->plane_count == 1) {
        if (frame->dmabuf_fd[0] < 0) {
            fprintf(stderr,
                    "frame buffer %u has no dmabuf fd; "
                    "VIDIOC_EXPBUF is required for V4L2 -> DRM/KMS zero-copy\n",
                    frame->index);
            errno = EINVAL;
            return -1;
        }

        if (import_fd_once(drm, fb, frame->dmabuf_fd[0], &handles[0]) < 0) {
            fprintf(stderr, "drmPrimeFDToHandle fd=%d failed: %s\n",
                    frame->dmabuf_fd[0],
                    strerror(errno));
            return -1;
        }

        handles[1] = handles[0];
        pitches[0] = frame->bytesperline[0];
        pitches[1] = frame->bytesperline[0];
        offsets[0] = frame->data_offset[0];
        offsets[1] = frame->data_offset[0] +
                     frame->bytesperline[0] * frame->height;
    } else {
        if (frame->plane_count < 2 ||
            frame->dmabuf_fd[0] < 0 ||
            frame->dmabuf_fd[1] < 0) {
            fprintf(stderr,
                    "frame buffer %u has invalid dmabuf fds: plane0=%d plane1=%d\n",
                    frame->index,
                    frame->dmabuf_fd[0],
                    frame->dmabuf_fd[1]);
            errno = EINVAL;
            return -1;
        }

        if (import_fd_once(drm, fb, frame->dmabuf_fd[0], &handles[0]) < 0) {
            fprintf(stderr, "drmPrimeFDToHandle plane0 fd=%d failed: %s\n",
                    frame->dmabuf_fd[0],
                    strerror(errno));
            return -1;
        }

        if (frame->dmabuf_fd[1] == frame->dmabuf_fd[0]) {
            handles[1] = handles[0];
        } else {
            if (import_fd_once(drm, fb, frame->dmabuf_fd[1], &handles[1]) < 0) {
                fprintf(stderr, "drmPrimeFDToHandle plane1 fd=%d failed: %s\n",
                        frame->dmabuf_fd[1],
                        strerror(errno));
                return -1;
            }
        }

        pitches[0] = frame->bytesperline[0];
        pitches[1] = frame->bytesperline[1];
        offsets[0] = frame->data_offset[0];
        offsets[1] = frame->data_offset[1];
    }

    ret = drmModeAddFB2(drm->fd,
                        frame->width,
                        frame->height,
                        DRM_FORMAT_NV12,
                        handles,
                        pitches,
                        offsets,
                        &fb->fb_id,
                        0);
    if (ret < 0) {
        fprintf(stderr,
                "drmModeAddFB2 failed: %s "
                "wxh=%dx%d stride=%u/%u offset=%u/%u handle=%u/%u fd=%d/%d\n",
                strerror(errno),
                frame->width,
                frame->height,
                pitches[0],
                pitches[1],
                offsets[0],
                offsets[1],
                handles[0],
                handles[1],
                frame->dmabuf_fd[0],
                frame->dmabuf_fd[1]);
        drm_fb_destroy(drm, fb);
        return -1;
    }

    *fb_id = fb->fb_id;
    return 0;
}

static uint32_t align_even(uint32_t v)
{
    return v & ~1U;
}

static uint32_t min_u32(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

static int drm_set_plane(drm_preview_t *drm,
                         uint32_t plane_id,
                         uint32_t fb_id,
                         uint32_t dst_x,
                         uint32_t dst_y,
                         uint32_t dst_w,
                         uint32_t dst_h,
                         uint32_t src_x,
                         uint32_t src_y,
                         uint32_t src_w,
                         uint32_t src_h)
{
    return drmModeSetPlane(drm->fd,
                           plane_id,
                           drm->crtc_id,
                           fb_id,
                           0,
                           dst_x,
                           dst_y,
                           dst_w,
                           dst_h,
                           src_x << 16,
                           src_y << 16,
                           src_w << 16,
                           src_h << 16);
}

static int drm_try_show_on_plane(drm_preview_t *drm,
                                 const camera_v4l2_frame_t *frame,
                                 uint32_t plane_id,
                                 uint32_t fb_id)
{
    int ret;

    ret = drm_set_plane(drm,
                        plane_id,
                        fb_id,
                        0,
                        0,
                        drm->mode.hdisplay,
                        drm->mode.vdisplay,
                        0,
                        0,
                        frame->width,
                        frame->height);
    if (ret == 0)
        return 0;

    if (errno == EINVAL) {
        int scale_errno = errno;
        uint32_t dst_w = min_u32(drm->mode.hdisplay, frame->width);
        uint32_t dst_h = min_u32(drm->mode.vdisplay, frame->height);
        uint32_t dst_x = (drm->mode.hdisplay - dst_w) / 2;
        uint32_t dst_y = (drm->mode.vdisplay - dst_h) / 2;
        uint32_t src_x = align_even((frame->width - dst_w) / 2);
        uint32_t src_y = align_even((frame->height - dst_h) / 2);

        ret = drm_set_plane(drm,
                            plane_id,
                            fb_id,
                            dst_x,
                            dst_y,
                            dst_w,
                            dst_h,
                            src_x,
                            src_y,
                            dst_w,
                            dst_h);
        if (ret == 0) {
            static int warned;

            if (!warned) {
                fprintf(stderr,
                        "warning: fullscreen scale unsupported, "
                        "fallback to center crop %ux%u from (%u,%u)\n",
                        dst_w,
                        dst_h,
                        src_x,
                        src_y);
                warned = 1;
            }
            return 0;
        }

        ret = drm_set_plane(drm,
                            plane_id,
                            fb_id,
                            0,
                            0,
                            dst_w,
                            dst_h,
                            0,
                            0,
                            dst_w,
                            dst_h);
        if (ret == 0) {
            static int warned_top_left;

            if (!warned_top_left) {
                fprintf(stderr,
                        "warning: crop offset unsupported, "
                        "fallback to top-left no-scale %ux%u\n",
                        dst_w,
                        dst_h);
                warned_top_left = 1;
            }
            return 0;
        }

        fprintf(stderr,
                "drmModeSetPlane failed: fullscreen errno=%s, "
                "center/top-left errno=%s, fb=%u plane=%u crtc=%u "
                "dst=%ux%u src=%ux%u crop=(%u,%u %ux%u)\n",
                strerror(scale_errno),
                strerror(errno),
                fb_id,
                plane_id,
                drm->crtc_id,
                drm->mode.hdisplay,
                drm->mode.vdisplay,
                frame->width,
                frame->height,
                src_x,
                src_y,
                dst_w,
                dst_h);
    } else {
        fprintf(stderr,
                "drmModeSetPlane fullscreen failed: %s, fb=%u plane=%u crtc=%u\n",
                strerror(errno),
                fb_id,
                plane_id,
                drm->crtc_id);
    }

    return ret;
}

static int drm_preview_show(drm_preview_t *drm,
                            const camera_v4l2_frame_t *frame)
{
    uint32_t fb_id;
    unsigned int i;
    int ret = -1;

    if (frame_to_drm_fb(drm, frame, &fb_id) < 0)
        return -1;

    for (i = 0; i < drm->plane_count; i++) {
        drm->plane_id = drm->plane_ids[i];
        ret = drm_try_show_on_plane(drm, frame, drm->plane_id, fb_id);
        if (ret == 0)
            return 0;
    }

    return ret;
}

static void usage(const char *prog)
{
    printf("Usage: %s [video] [width] [height] [fourcc] [fps] [drm]\n", prog);
    printf("Default: %s /dev/video0 1920 1080 NV12 30 /dev/dri/card0\n", prog);
}

int main(int argc, char **argv)
{
    const char *video_dev = "/dev/video0";
    const char *drm_dev = "/dev/dri/card0";
    int width = 1920;
    int height = 1080;
    int fps = 30;
    unsigned int pixfmt = V4L2_PIX_FMT_NV12;
    camera_v4l2_t cam;
    drm_preview_t drm;
    camera_v4l2_frame_t cur;
    camera_v4l2_frame_t prev;
    int has_prev = 0;
    unsigned int frame_count = 0;
    char fourcc[5];

    if (argc > 1 && strcmp(argv[1], "-h") == 0) {
        usage(argv[0]);
        return 0;
    }

    if (argc > 1)
        video_dev = argv[1];
    if (argc > 2)
        width = atoi(argv[2]);
    if (argc > 3)
        height = atoi(argv[3]);
    if (argc > 4)
        pixfmt = parse_fourcc(argv[4]);
    if (argc > 5)
        fps = atoi(argv[5]);
    if (argc > 6)
        drm_dev = argv[6];

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("video=%s request=%dx%d %s fps=%d drm=%s\n",
           video_dev,
           width,
           height,
           camera_v4l2_fourcc_string(pixfmt, fourcc),
           fps,
           drm_dev);

    if (camera_v4l2_open(&cam, video_dev, width, height, pixfmt, fps, 4) < 0) {
        fprintf(stderr, "camera_v4l2_open failed: %s\n", strerror(errno));
        return 1;
    }

    printf("camera actual=%dx%d %s planes=%u buffers=%u stride0=%u\n",
           cam.width,
           cam.height,
           camera_v4l2_fourcc_string(cam.pixelformat, fourcc),
           cam.plane_count,
           cam.buffer_count,
           cam.bytesperline[0]);

    if (drm_preview_open(&drm, drm_dev) < 0) {
        fprintf(stderr, "drm_preview_open failed: %s\n", strerror(errno));
        camera_v4l2_close(&cam);
        return 1;
    }

    if (camera_v4l2_start(&cam) < 0) {
        fprintf(stderr, "camera_v4l2_start failed: %s\n", strerror(errno));
        drm_preview_close(&drm);
        camera_v4l2_close(&cam);
        return 1;
    }

    memset(&prev, 0, sizeof(prev));

    while (g_running) {
        if (camera_v4l2_dequeue(&cam, &cur, 2000) < 0) {
            fprintf(stderr, "dequeue failed: %s\n", strerror(errno));
            break;
        }

        if (frame_count == 0) {
            unsigned int p;

            for (p = 0; p < cur.plane_count; p++) {
                printf("first frame plane[%u]: dmabuf_fd=%d bytesused=%u "
                       "length=%zu stride=%u data_offset=%u\n",
                       p,
                       cur.dmabuf_fd[p],
                       cur.bytesused[p],
                       cur.lengths[p],
                       cur.bytesperline[p],
                       cur.data_offset[p]);
            }
        }

        if (drm_preview_show(&drm, &cur) < 0) {
            fprintf(stderr, "drm display failed: %s\n", strerror(errno));
            camera_v4l2_queue(&cam, &cur);
            break;
        }

        if (has_prev)
            camera_v4l2_queue(&cam, &prev);

        prev = cur;
        has_prev = 1;
        frame_count++;

        if ((frame_count % 120) == 0) {
            printf("displayed %u frames, last sequence=%u ts=%lld\n",
                   frame_count,
                   cur.sequence,
                   (long long)cur.ts_ns);
            fflush(stdout);
        }
    }

    if (has_prev)
        camera_v4l2_queue(&cam, &prev);

    camera_v4l2_stop(&cam);
    drm_preview_close(&drm);
    camera_v4l2_close(&cam);

    return 0;
}
