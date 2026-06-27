#ifndef CAMERA_V4L2_H
#define CAMERA_V4L2_H

#include <stddef.h>
#include <stdint.h>
#include <time.h>
#include <linux/videodev2.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_V4L2_MAX_BUFFERS 8
#define CAMERA_V4L2_MAX_PLANES 3

typedef struct {
    void *start[CAMERA_V4L2_MAX_PLANES];
    size_t length[CAMERA_V4L2_MAX_PLANES];
    int dmabuf_fd[CAMERA_V4L2_MAX_PLANES];
} camera_v4l2_buffer_t;

typedef struct {
    int fd;
    char dev_name[256];
    int width;
    int height;
    unsigned int pixelformat;
    enum v4l2_buf_type buf_type;
    unsigned int plane_count;
    unsigned int bytesperline[CAMERA_V4L2_MAX_PLANES];
    unsigned int sizeimage[CAMERA_V4L2_MAX_PLANES];
    unsigned int buffer_count;
    int streaming;
    camera_v4l2_buffer_t buffers[CAMERA_V4L2_MAX_BUFFERS];
} camera_v4l2_t;

typedef struct {
    unsigned int index;                                     // 缓冲区索引号
    unsigned int sequence;                                  // 帧序号     
    int64_t ts_ns;                                          // 时间戳 
    unsigned int timestamp_flags;                           // 时间戳标志
    int width;                                              // 宽
    int height;                                             // 高
    unsigned int pixelformat;                               // 像素格式 e.g. V4L2_PIX_FMT_NV12
    unsigned int plane_count;                               // 平面数
    void *planes[CAMERA_V4L2_MAX_PLANES];                   // 每个 plane 在用户态 mmap 后的地址
    size_t lengths[CAMERA_V4L2_MAX_PLANES];                 // 每个 plane 的 mmap buffer 长度
    unsigned int bytesused[CAMERA_V4L2_MAX_PLANES];         // 每个 plane 实际使用长度
    unsigned int bytesperline[CAMERA_V4L2_MAX_PLANES];      // 每个 plane 的 stride，也就是一行实际占多少字节
    unsigned int sizeimage[CAMERA_V4L2_MAX_PLANES];         // 每个 plane 图像 buffer 大小
    unsigned int data_offset[CAMERA_V4L2_MAX_PLANES];       // 每个 plane 数据偏移
    int dmabuf_fd[CAMERA_V4L2_MAX_PLANES];                  // 每个 plane 对应的 dmabuf fd
} camera_v4l2_frame_t;

int camera_v4l2_open(camera_v4l2_t *cam,
                     const char *dev_name,
                     int width,
                     int height,
                     unsigned int pixelformat,
                     int fps,
                     unsigned int buffer_count);
int camera_v4l2_start(camera_v4l2_t *cam);
int camera_v4l2_dequeue(camera_v4l2_t *cam,
                        camera_v4l2_frame_t *frame,
                        int timeout_ms);
int camera_v4l2_queue(camera_v4l2_t *cam,
                      const camera_v4l2_frame_t *frame);
int camera_v4l2_stop(camera_v4l2_t *cam);
void camera_v4l2_close(camera_v4l2_t *cam);

char *camera_v4l2_fourcc_string(unsigned int fourcc, char out[5]);

#ifdef __cplusplus
}
#endif

#endif
