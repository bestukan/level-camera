#include "camera_v4l2.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef V4L2_CAP_DEVICE_CAPS
#define V4L2_CAP_DEVICE_CAPS 0x80000000
#endif

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

/**
 * @brief 健壮的 ioctl 封装，自动重试被信号中断的调用
 * @param fd       设备文件描述符
 * @param request  ioctl 请求码
 * @param arg      ioctl 参数指针
 * @return ioctl 返回值，失败返回 -1 并设置 errno
 *
 * 在循环中调用 ioctl()，当返回错误且 errno == EINTR 时自动重试。
 * 避免因信号到达而导致 ioctl 莫名失败。
 */
static int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;

    do {
        ret = ioctl(fd, request, arg);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

/**
 * @brief 将 struct timeval 时间转换为纳秒值
 * @param tv  指向 timeval 结构体的指针（秒 + 微秒）
 * @return 纳秒级时间戳（int64_t）
 *
 * 统一的纳秒时间戳表示，用于给采集到的帧打上精确时间标记。
 */
static int64_t timeval_to_ns(const struct timeval *tv)
{
    return (int64_t)tv->tv_sec * 1000000000LL + (int64_t)tv->tv_usec * 1000LL;
}

/**
 * @brief 重置单个缓冲区的所有 plane 为初始状态
 * @param buffer  指向待重置的缓冲区结构体
 *
 * 将所有 plane 的映射起始地址置 NULL、长度置 0、
 * DMA-buf 文件描述符置 -1（无效值）。
 */
static void clear_buffer(camera_v4l2_buffer_t *buffer)
{
    unsigned int i;

    for (i = 0; i < CAMERA_V4L2_MAX_PLANES; i++) {
        buffer->start[i] = NULL;
        buffer->length[i] = 0;
        buffer->dmabuf_fd[i] = -1;
    }
}

/**
 * @brief 批量重置摄像头实例的所有缓冲区
 * @param cam  指向摄像头实例结构体
 *
 * 遍历整个缓冲区数组，逐一调用 clear_buffer()。
 */
static void clear_buffers(camera_v4l2_t *cam)
{
    unsigned int i;

    for (i = 0; i < CAMERA_V4L2_MAX_BUFFERS; i++)
        clear_buffer(&cam->buffers[i]);
}

/**
 * @brief 从 V4L2 capability 结构体中提取有效的设备能力位
 * @param cap  指向 v4l2_capability 结构体
 * @return 设备能力掩码
 *
 * V4L2 规范中，若驱动支持 V4L2_CAP_DEVICE_CAPS，则能力值存放在
 * device_caps 字段中；否则回退到 capabilities 字段。
 */
static unsigned int effective_caps(const struct v4l2_capability *cap)
{
    if (cap->capabilities & V4L2_CAP_DEVICE_CAPS)
        return cap->device_caps;

    return cap->capabilities;
}

/**
 * @brief 设置摄像头的帧率
 * @param cam  指向摄像头实例结构体
 * @param fps  目标帧率（每秒帧数）
 * @return 成功返回 0，失败返回 -1
 *
 * 通过 VIDIOC_S_PARM ioctl 设置 timeperframe = {numerator=1, denominator=fps}。
 * 若 fps <= 0 则跳过设置，直接返回成功。
 */
static int set_fps(camera_v4l2_t *cam, int fps)
{
    struct v4l2_streamparm parm;

    if (fps <= 0)
        return 0;

    memset(&parm, 0, sizeof(parm));
    parm.type = cam->buf_type;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;

    if (xioctl(cam->fd, VIDIOC_S_PARM, &parm) < 0) {
        if (errno == ENOTTY || errno == EINVAL) {
            fprintf(stderr,
                    "warning: VIDIOC_S_PARM fps=%d unsupported, continue\n",
                    fps);
            return 0;
        }
        return -1;
    }

    return 0;
}

/**
 * @brief 设置视频格式（分辨率、像素格式）
 * @param cam         指向摄像头实例结构体
 * @param width       目标宽度
 * @param height      目标高度
 * @param pixelformat 目标像素格式（V4L2 FOURCC 码）
 * @return 成功返回 0，失败返回 -1
 *
 * 通过 VIDIOC_S_FMT ioctl 向驱动请求指定格式，然后从驱动返回的实际格式中
 * 读取并保存：宽度、高度、pixelformat、plane 数量、每行字节数(bytesperline)、
 * 每帧图像大小(sizeimage)。自动区分单平面和多平面模式。
 */
static int set_format(camera_v4l2_t *cam,
                      int width,
                      int height,
                      unsigned int pixelformat)
{
    struct v4l2_format fmt;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = cam->buf_type;

    if (cam->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = pixelformat;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    } else {
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = pixelformat;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }

    if (xioctl(cam->fd, VIDIOC_S_FMT, &fmt) < 0)
        return -1;

    if (cam->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        unsigned int p;

        cam->width = fmt.fmt.pix_mp.width;
        cam->height = fmt.fmt.pix_mp.height;
        cam->pixelformat = fmt.fmt.pix_mp.pixelformat;
        cam->plane_count = fmt.fmt.pix_mp.num_planes;

        for (p = 0; p < cam->plane_count &&
                    p < CAMERA_V4L2_MAX_PLANES; p++) {
            cam->bytesperline[p] =
                fmt.fmt.pix_mp.plane_fmt[p].bytesperline;
            cam->sizeimage[p] = fmt.fmt.pix_mp.plane_fmt[p].sizeimage;
        }
    } else {
        cam->width = fmt.fmt.pix.width;
        cam->height = fmt.fmt.pix.height;
        cam->pixelformat = fmt.fmt.pix.pixelformat;
        cam->plane_count = 1;
        cam->bytesperline[0] = fmt.fmt.pix.bytesperline;
        cam->sizeimage[0] = fmt.fmt.pix.sizeimage;
    }

    if (cam->plane_count == 0 ||
        cam->plane_count > CAMERA_V4L2_MAX_PLANES) {
        errno = EINVAL;
        return -1;
    }

    return 0;
}

/**
 * @brief 将指定缓冲区 plane 导出为 DMA-buf 文件描述符
 * @param cam   指向摄像头实例结构体
 * @param index 缓冲区索引号
 * @param plane plane 索引号
 * @return 成功返回 dmabuf fd，失败返回 -1
 *
 * 通过 VIDIOC_EXPBUF ioctl 导出指定缓冲区 plane 的 DMA-buf fd，
 * 供其他设备（如 DRM/GPU、ISP、编码器）零拷贝共享该内存。
 */
static int export_plane_fd(camera_v4l2_t *cam,
                           unsigned int index,
                           unsigned int plane)
{
    struct v4l2_exportbuffer expbuf;

    memset(&expbuf, 0, sizeof(expbuf));
    expbuf.type = cam->buf_type;
    expbuf.index = index;
    expbuf.plane = plane;
    expbuf.flags = O_CLOEXEC;

    if (xioctl(cam->fd, VIDIOC_EXPBUF, &expbuf) < 0)
        return -1;

    return expbuf.fd;
}

/**
 * @brief 向驱动申请 MMAP 缓冲区并映射到用户空间
 * @param cam   指向摄像头实例结构体
 * @param count 期望的缓冲区数量（0 则使用默认值 4）
 * @return 成功返回 0，失败返回 -1
 *
 * 完整的 MMAP 缓冲区初始化流程：
 * 1. VIDIOC_REQBUFS   —— 向驱动申请 count 个 MMAP 缓冲区
 * 2. VIDIOC_QUERYBUF  —— 对每个 buffer 查询偏移量和长度
 * 3. mmap()           —— 将内核缓冲区映射到用户空间
 * 4. export_plane_fd  —— 同时导出每个 plane 的 DMA-buf fd
 *
 * 驱动至少需返回 2 个 buffer，最终数量保存到 cam->buffer_count。
 */
static int request_mmap_buffers(camera_v4l2_t *cam, unsigned int count)
{
    struct v4l2_requestbuffers req;
    unsigned int i;

    if (count == 0)
        count = 4;
    if (count > CAMERA_V4L2_MAX_BUFFERS)
        count = CAMERA_V4L2_MAX_BUFFERS;

    memset(&req, 0, sizeof(req));
    req.count = count;
    req.type = cam->buf_type;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(cam->fd, VIDIOC_REQBUFS, &req) < 0)
        return -1;

    if (req.count < 2) {
        errno = ENOMEM;
        return -1;
    }

    if (req.count > CAMERA_V4L2_MAX_BUFFERS)
        req.count = CAMERA_V4L2_MAX_BUFFERS;

    cam->buffer_count = req.count;

    for (i = 0; i < cam->buffer_count; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[CAMERA_V4L2_MAX_PLANES];
        unsigned int p;

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = cam->buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (cam->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            buf.m.planes = planes;
            buf.length = cam->plane_count;
        }

        if (xioctl(cam->fd, VIDIOC_QUERYBUF, &buf) < 0)
            return -1;

        for (p = 0; p < cam->plane_count; p++) {
            size_t length;
            unsigned int offset;
            void *start;

            if (cam->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
                length = planes[p].length;
                offset = planes[p].m.mem_offset;
            } else {
                length = buf.length;
                offset = buf.m.offset;
            }

            start = mmap(NULL, length, PROT_READ | PROT_WRITE,
                         MAP_SHARED, cam->fd, offset);
            if (start == MAP_FAILED)
                return -1;

            cam->buffers[i].start[p] = start;
            cam->buffers[i].length[p] = length;
            cam->buffers[i].dmabuf_fd[p] = export_plane_fd(cam, i, p);
            if (cam->buffers[i].dmabuf_fd[p] < 0) {
                fprintf(stderr,
                        "warning: VIDIOC_EXPBUF failed for buffer %u plane %u: %s\n",
                        i,
                        p,
                        strerror(errno));
            }
        }
    }

    return 0;
}

/**
 * @brief 将指定索引的缓冲区入队到驱动的空闲队列
 * @param cam   指向摄像头实例结构体
 * @param index 缓冲区索引号
 * @return ioctl 返回值
 *
 * 通过 VIDIOC_QBUF ioctl 通知驱动"该 buffer 空闲可用，可以装新数据了"。
 * 自动区分多平面/单平面模式以正确设置 m.planes 和 length。
 */
static int queue_index(camera_v4l2_t *cam, unsigned int index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[CAMERA_V4L2_MAX_PLANES];

    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type = cam->buf_type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;

    if (cam->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = cam->plane_count;
    }

    return xioctl(cam->fd, VIDIOC_QBUF, &buf);
}

/**
 * @brief 打开并初始化 V4L2 摄像头设备
 * @param cam          指向摄像头实例结构体（由调用者分配）
 * @param dev_name     设备节点路径（如 "/dev/video0"）
 * @param width        期望的采集宽度
 * @param height       期望的采集高度
 * @param pixelformat  期望的像素格式（V4L2 FOURCC 码）
 * @param fps          期望的帧率（<=0 则不设置）
 * @param buffer_count 期望的缓冲区数量（0 则使用默认值 4）
 * @return 成功返回 0，失败返回 -1（并设置 errno）
 *
 * 完整的初始化流程：
 * 1. 参数校验 + 清零 cam 结构体
 * 2. open() 打开设备节点
 * 3. VIDIOC_QUERYCAP 查询设备能力
 * 4. 根据能力自动选择多平面(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
 *    或单平面(V4L2_BUF_TYPE_VIDEO_CAPTURE)缓冲区模式
 * 5. set_format() → set_fps() → request_mmap_buffers()
 * 6. 失败时通过 goto fail 统一清理，保留 errno
 */
int camera_v4l2_open(camera_v4l2_t *cam,
                     const char *dev_name,
                     int width,
                     int height,
                     unsigned int pixelformat,
                     int fps,
                     unsigned int buffer_count)
{
    struct v4l2_capability cap;
    unsigned int caps;

    if (!cam || !dev_name || width <= 0 || height <= 0) {
        errno = EINVAL;
        return -1;
    }

    memset(cam, 0, sizeof(*cam));
    cam->fd = -1;
    clear_buffers(cam);

    /* 保存设备节点名称 */
    snprintf(cam->dev_name, sizeof(cam->dev_name), "%s", dev_name);

    cam->fd = open(dev_name, O_RDWR | O_CLOEXEC, 0);
    if (cam->fd < 0)
        return -1;

    memset(&cap, 0, sizeof(cap));
    if (xioctl(cam->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        goto fail;
    }

    caps = effective_caps(&cap);

    if (!(caps & V4L2_CAP_STREAMING)) {
        errno = ENOTSUP;
        goto fail;
    }

    if (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        cam->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    else if (caps & V4L2_CAP_VIDEO_CAPTURE)
        cam->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    else {
        errno = ENOTSUP;
        goto fail;
    }

    if (set_format(cam, width, height, pixelformat) < 0) {
        perror("VIDIOC_S_FMT");
        goto fail;
    }

    if (set_fps(cam, fps) < 0) {
        perror("VIDIOC_S_PARM");
        goto fail;
    }

    if (request_mmap_buffers(cam, buffer_count) < 0) {
        perror("request_mmap_buffers");
        goto fail;
    }

    return 0;

fail:
    {
        int saved_errno = errno;

        camera_v4l2_close(cam);
        errno = saved_errno;
    }
    return -1;
}

/**
 * @brief 启动视频流采集
 * @param cam  指向已初始化的摄像头实例结构体
 * @return 成功返回 0，失败返回 -1
 *
 * 1. 将 camera_v4l2_open 中已申请的所有 buffer 逐一 VIDIOC_QBUF 入队
 * 2. 调用 VIDIOC_STREAMON 启动硬件采集
 * 3. 设置 cam->streaming = 1
 * 若已经在 streaming 状态则幂等返回 0。
 */
int camera_v4l2_start(camera_v4l2_t *cam)
{
    enum v4l2_buf_type type;
    unsigned int i;

    if (!cam || cam->fd < 0 || cam->buffer_count == 0) {
        errno = EINVAL;
        return -1;
    }

    if (cam->streaming)
        return 0;

    for (i = 0; i < cam->buffer_count; i++) {
        if (queue_index(cam, i) < 0)
            return -1;
    }

    type = cam->buf_type;
    if (xioctl(cam->fd, VIDIOC_STREAMON, &type) < 0)
        return -1;

    cam->streaming = 1;
    return 0;
}

/**
 * @brief 出队一帧已采集完成的图像数据（阻塞等待）
 * @param cam        指向已启动流采集的摄像头实例
 * @param frame      指向调用者分配的帧结构体（输出参数）
 * @param timeout_ms poll 超时等待时间（毫秒）
 * @return 成功返回 0，超时返回 -1（errno=ETIMEDOUT），其他错误返回 -1
 *
 * 这是数据采集的核心函数：
 * 1. poll() 等待设备 fd 有 POLLIN 事件（或超时）
 * 2. VIDIOC_DQBUF 取出驱动已完成填充的缓冲区
 * 3. 填充 frame 结构体，包含：帧序号(sequence)、纳秒时间戳(ts_ns)、
 *    宽高、像素格式、plane 数据指针/长度/dmabuf_fd、有效字节数(bytesused)等
 * 4. 自动区分多平面/单平面模式读取各自的数据偏移和有效字节数
 *
 * 调用者在处理完帧数据后应调用 camera_v4l2_queue() 将 buffer 归还驱动。
 */
int camera_v4l2_dequeue(camera_v4l2_t *cam,
                        camera_v4l2_frame_t *frame,
                        int timeout_ms)
{
    struct pollfd pfd;
    struct v4l2_buffer buf;
    struct v4l2_plane planes[CAMERA_V4L2_MAX_PLANES];
    unsigned int p;
    int ret;

    if (!cam || !frame || cam->fd < 0 || !cam->streaming) {
        errno = EINVAL;
        return -1;
    }

    memset(frame, 0, sizeof(*frame));
    for (p = 0; p < CAMERA_V4L2_MAX_PLANES; p++)
        frame->dmabuf_fd[p] = -1;

    memset(&pfd, 0, sizeof(pfd));
    pfd.fd = cam->fd;
    pfd.events = POLLIN | POLLERR;

    do {
        ret = poll(&pfd, 1, timeout_ms);
    } while (ret < 0 && errno == EINTR);

    if (ret <= 0) {
        if (ret == 0)
            errno = ETIMEDOUT;
        return -1;
    }

    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = cam->buf_type;
    buf.memory = V4L2_MEMORY_MMAP;

    if (cam->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = cam->plane_count;
    }

    if (xioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0)
        return -1;

    if (buf.index >= cam->buffer_count) {
        errno = EIO;
        return -1;
    }

    frame->index = buf.index;
    frame->sequence = buf.sequence;
    frame->ts_ns = timeval_to_ns(&buf.timestamp);
    frame->timestamp_flags = buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
    frame->width = cam->width;
    frame->height = cam->height;
    frame->pixelformat = cam->pixelformat;
    frame->plane_count = cam->plane_count;

    for (p = 0; p < cam->plane_count; p++) {
        frame->planes[p] = cam->buffers[buf.index].start[p];
        frame->lengths[p] = cam->buffers[buf.index].length[p];
        frame->dmabuf_fd[p] = cam->buffers[buf.index].dmabuf_fd[p];
        frame->bytesperline[p] = cam->bytesperline[p];
        frame->sizeimage[p] = cam->sizeimage[p];

        if (cam->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            frame->bytesused[p] = planes[p].bytesused;
            frame->data_offset[p] = planes[p].data_offset;
        } else {
            frame->bytesused[p] = buf.bytesused;
            frame->data_offset[p] = 0;
        }
    }

    return 0;
}

/**
 * @brief 将已处理的帧缓冲区重新入队，归还驱动
 * @param cam   指向摄像头实例结构体
 * @param frame 指向之前 dequeue 出的帧（仅需 frame->index 有效）
 * @return 成功返回 0，失败返回 -1
 *
 * 用户处理完 camera_v4l2_dequeue 取出的帧后调用此函数，
 * 通过 VIDIOC_QBUF 将 buffer 还回驱动的空闲队列，
 * 形成 "dequeue → 处理 → queue" 的循环采集管线。
 */
int camera_v4l2_queue(camera_v4l2_t *cam,
                      const camera_v4l2_frame_t *frame)
{
    if (!cam || !frame || cam->fd < 0 || frame->index >= cam->buffer_count) {
        errno = EINVAL;
        return -1;
    }

    return queue_index(cam, frame->index);
}

/**
 * @brief 停止视频流采集
 * @param cam  指向摄像头实例结构体
 * @return 成功返回 0，失败返回 -1
 *
 * 通过 VIDIOC_STREAMOFF ioctl 停止硬件采集，
 * 设置 cam->streaming = 0。若已停止则幂等返回 0。
 */
int camera_v4l2_stop(camera_v4l2_t *cam)
{
    enum v4l2_buf_type type;

    if (!cam || cam->fd < 0) {
        errno = EINVAL;
        return -1;
    }

    if (!cam->streaming)
        return 0;

    type = cam->buf_type;
    if (xioctl(cam->fd, VIDIOC_STREAMOFF, &type) < 0)
        return -1;

    cam->streaming = 0;
    return 0;
}

/**
 * @brief 关闭摄像头设备并释放所有资源
 * @param cam  指向摄像头实例结构体
 *
 * 逆序清理所有已分配的资源：
 * 1. 若还在 streaming 则先 camera_v4l2_stop() 停止采集
 * 2. munmap() 所有 mmap 映射的用户空间缓冲区
 * 3. close()  所有导出的 DMA-buf 文件描述符
 * 4. VIDIOC_REQBUFS(count=0) 通知驱动释放内核缓冲区
 * 5. close() 设备 fd，清零整个 cam 结构体
 *
 * 该函数是幂等的，可安全多次调用或对未完全初始化的实例调用。
 */
void camera_v4l2_close(camera_v4l2_t *cam)
{
    unsigned int i;
    unsigned int p;

    if (!cam)
        return;

    if (cam->fd >= 0 && cam->streaming)
        camera_v4l2_stop(cam);

    for (i = 0; i < cam->buffer_count; i++) {
        for (p = 0; p < cam->plane_count; p++) {
            if (cam->buffers[i].start[p]) {
                munmap(cam->buffers[i].start[p],
                       cam->buffers[i].length[p]);
                cam->buffers[i].start[p] = NULL;
            }

            if (cam->buffers[i].dmabuf_fd[p] >= 0) {
                close(cam->buffers[i].dmabuf_fd[p]);
                cam->buffers[i].dmabuf_fd[p] = -1;
            }
        }
    }

    if (cam->fd >= 0) {
        struct v4l2_requestbuffers req;

        memset(&req, 0, sizeof(req));
        req.count = 0;
        req.type = cam->buf_type;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(cam->fd, VIDIOC_REQBUFS, &req);

        close(cam->fd);
    }

    memset(cam, 0, sizeof(*cam));
    cam->fd = -1;
    clear_buffers(cam);
}

/**
 * @brief 将 V4L2 四字符码（FOURCC）转为可读字符串
 * @param fourcc  像素格式 FOURCC 码（如 NV12 = 0x3231564e）
 * @param out     调用者提供的至少 5 字节的输出缓冲区
 * @return 返回 out 指针，方便在 printf 中直接使用
 *
 * V4L2 像素格式采用 little-endian 四字符编码，此函数逐字节提取
 * 并生成 C 字符串。例如 0x3231564e → "NV12"。
 */
char *camera_v4l2_fourcc_string(unsigned int fourcc, char out[5])
{
    out[0] = (char)(fourcc & 0xff);
    out[1] = (char)((fourcc >> 8) & 0xff);
    out[2] = (char)((fourcc >> 16) & 0xff);
    out[3] = (char)((fourcc >> 24) & 0xff);
    out[4] = '\0';

    return out;
}
