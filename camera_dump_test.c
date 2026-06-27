#include "camera_v4l2.h"

#include <errno.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int parse_fourcc(const char *s)
{
    if (!s || strlen(s) != 4)
        return V4L2_PIX_FMT_NV12;

    return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

static void print_frame_info(const camera_v4l2_frame_t *frame)
{
    char fourcc[5];
    unsigned int p;

    printf("frame: index=%u sequence=%u ts_ns=%lld\n",
           frame->index,
           frame->sequence,
           (long long)frame->ts_ns);
    printf("format: %dx%d %s planes=%u timestamp_flags=0x%x\n",
           frame->width,
           frame->height,
           camera_v4l2_fourcc_string(frame->pixelformat, fourcc),
           frame->plane_count,
           frame->timestamp_flags);

    for (p = 0; p < frame->plane_count; p++) {
        printf("plane[%u]: ptr=%p length=%zu bytesused=%u stride=%u "
               "sizeimage=%u data_offset=%u dmabuf_fd=%d\n",
               p,
               frame->planes[p],
               frame->lengths[p],
               frame->bytesused[p],
               frame->bytesperline[p],
               frame->sizeimage[p],
               frame->data_offset[p],
               frame->dmabuf_fd[p]);
    }
}

static int write_rows(FILE *fp,
                      const uint8_t *base,
                      int width_bytes,
                      int rows,
                      int stride)
{
    int y;

    if (!fp || !base || width_bytes <= 0 || rows <= 0 || stride <= 0)
        return -1;

    for (y = 0; y < rows; y++) {
        if (fwrite(base + (size_t)y * stride, 1, width_bytes, fp) !=
            (size_t)width_bytes)
            return -1;
    }

    return 0;
}

static int dump_y_pgm(const camera_v4l2_frame_t *frame, const char *path)
{
    FILE *fp;
    const uint8_t *y;
    int stride;

    if (!frame || frame->plane_count < 1 || !frame->planes[0])
        return -1;

    stride = frame->bytesperline[0] ? (int)frame->bytesperline[0] :
                                      frame->width;
    y = (const uint8_t *)frame->planes[0] + frame->data_offset[0];

    fp = fopen(path, "wb");
    if (!fp)
        return -1;

    fprintf(fp, "P5\n%d %d\n255\n", frame->width, frame->height);
    if (write_rows(fp, y, frame->width, frame->height, stride) < 0) {
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return 0;
}

static int dump_compact_nv12(const camera_v4l2_frame_t *frame, const char *path)
{
    FILE *fp;
    const uint8_t *y;
    const uint8_t *uv;
    int y_stride;
    int uv_stride;

    if (!frame || frame->plane_count < 1 || !frame->planes[0])
        return -1;

    y_stride = frame->bytesperline[0] ? (int)frame->bytesperline[0] :
                                        frame->width;

    y = (const uint8_t *)frame->planes[0] + frame->data_offset[0];

    if (frame->plane_count >= 2 && frame->planes[1]) {
        uv_stride = frame->bytesperline[1] ? (int)frame->bytesperline[1] :
                                             frame->width;
        uv = (const uint8_t *)frame->planes[1] + frame->data_offset[1];
    } else {
        uv_stride = y_stride;
        uv = y + (size_t)y_stride * frame->height;
    }

    fp = fopen(path, "wb");
    if (!fp)
        return -1;

    if (write_rows(fp, y, frame->width, frame->height, y_stride) < 0 ||
        write_rows(fp, uv, frame->width, frame->height / 2, uv_stride) < 0) {
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return 0;
}

static int write_compact_nv12(FILE *fp, const camera_v4l2_frame_t *frame)
{
    const uint8_t *y;
    const uint8_t *uv;
    int y_stride;
    int uv_stride;

    if (!fp || !frame || frame->plane_count < 1 || !frame->planes[0])
        return -1;

    y_stride = frame->bytesperline[0] ? (int)frame->bytesperline[0] :
                                        frame->width;
    y = (const uint8_t *)frame->planes[0] + frame->data_offset[0];

    if (frame->plane_count >= 2 && frame->planes[1]) {
        uv_stride = frame->bytesperline[1] ? (int)frame->bytesperline[1] :
                                             frame->width;
        uv = (const uint8_t *)frame->planes[1] + frame->data_offset[1];
    } else {
        uv_stride = y_stride;
        uv = y + (size_t)y_stride * frame->height;
    }

    return write_rows(fp, y, frame->width, frame->height, y_stride) < 0 ||
           write_rows(fp, uv, frame->width, frame->height / 2, uv_stride) < 0
               ? -1
               : 0;
}

static void usage(const char *prog)
{
    printf("Usage: %s [device] [width] [height] [fourcc] [fps] [skip]\n", prog);
    printf("Default: %s /dev/video0 1920 1080 NV12 30 10\n", prog);
}

int main(int argc, char **argv)
{
    const char *dev_name = "/dev/video0";
    int width = 1920;
    int height = 1080;
    unsigned int pixfmt = V4L2_PIX_FMT_NV12;
    int fps = 30;
    int skip = 10;
    int i;
    camera_v4l2_t cam;
    camera_v4l2_frame_t frame;
    char fourcc[5];

    if (argc > 1 && strcmp(argv[1], "-h") == 0) {
        usage(argv[0]);
        return 0;
    }

    if (argc > 1)
        dev_name = argv[1];
    if (argc > 2)
        width = atoi(argv[2]);
    if (argc > 3)
        height = atoi(argv[3]);
    if (argc > 4)
        pixfmt = parse_fourcc(argv[4]);
    if (argc > 5)
        fps = atoi(argv[5]);
    if (argc > 6)
        skip = atoi(argv[6]);

    fprintf(stderr,"open %s request %dx%d %s fps=%d skip=%d\n",
           dev_name,
           width,
           height,
           camera_v4l2_fourcc_string(pixfmt, fourcc),
           fps,
           skip);

    if (camera_v4l2_open(&cam, dev_name, width, height, pixfmt, fps, 4) < 0) {
        fprintf(stderr, "camera_v4l2_open failed: %s\n", strerror(errno));
        return 1;
    }

    fprintf(stderr,"actual %dx%d %s buf_type=%u planes=%u buffers=%u\n",
           cam.width,
           cam.height,
           camera_v4l2_fourcc_string(cam.pixelformat, fourcc),
           cam.buf_type,
           cam.plane_count,
           cam.buffer_count);

    if (camera_v4l2_start(&cam) < 0) {
        fprintf(stderr, "camera_v4l2_start failed: %s\n", strerror(errno));
        camera_v4l2_close(&cam);
        return 1;
    }

    while (1) {
        if (camera_v4l2_dequeue(&cam, &frame, 2000) < 0) {
            fprintf(stderr, "dequeue failed: %s\n", strerror(errno));
            break;
        }

        if (frame.pixelformat != V4L2_PIX_FMT_NV12) {
            fprintf(stderr, "not NV12\n");
            camera_v4l2_queue(&cam, &frame);
            break;
        }

        if (write_compact_nv12(stdout, &frame) < 0) {
            camera_v4l2_queue(&cam, &frame);
            break;
        }

        camera_v4l2_queue(&cam, &frame);
    }

    camera_v4l2_close(&cam);

    // for (i = 0; i < skip; i++) {
    //     if (camera_v4l2_dequeue(&cam, &frame, 2000) < 0) {
    //         fprintf(stderr, "discard dequeue failed: %s\n", strerror(errno));
    //         camera_v4l2_close(&cam);
    //         return 1;
    //     }
    //     camera_v4l2_queue(&cam, &frame);
    // }

    // if (camera_v4l2_dequeue(&cam, &frame, 2000) < 0) {
    //     fprintf(stderr, "capture dequeue failed: %s\n", strerror(errno));
    //     camera_v4l2_close(&cam);
    //     return 1;
    // }

    // print_frame_info(&frame);

    // if (frame.pixelformat == V4L2_PIX_FMT_NV12) {
    //     if (dump_compact_nv12(&frame, "/tmp/level_camera_frame.nv12") < 0)
    //         fprintf(stderr, "dump nv12 failed: %s\n", strerror(errno));
    //     else
    //         printf("wrote /tmp/level_camera_frame.nv12\n");
    // } else {
    //     fprintf(stderr, "not NV12, skip compact NV12 dump\n");
    // }

    // if (dump_y_pgm(&frame, "/tmp/level_camera_y.pgm") < 0)
    //     fprintf(stderr, "dump pgm failed: %s\n", strerror(errno));
    // else
    //     printf("wrote /tmp/level_camera_y.pgm\n");

    // camera_v4l2_queue(&cam, &frame);
    // camera_v4l2_close(&cam);

    return 0;
}
