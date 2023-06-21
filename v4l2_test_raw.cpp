/*
 * Copyright (c) 2018 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <pthread.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>
#include <semaphore.h>

#include "logs.h"

#include "mediactl.h"
#include "v4l2subdev.h"
#include "v4l2videodev.h"
#include "mediaApi.h"

#include "staticPipe.h"
#include "ispMgr.h"
#include "lens_config.h"

//#define WDR_ENABLE
#define DUAL_CAMERA
#define dump_forever

#define NB_BUFFER                4
#define NB_BUFFER_PARAM          1

#define V4L2_META_AML_ISP_CONFIG    v4l2_fourcc('A', 'C', 'F', 'G') /* Aml isp config */
#define V4L2_META_AML_ISP_STATS     v4l2_fourcc('A', 'S', 'T', 'S') /* Aml isp statistics */

uint32_t image_width = 1920;
uint32_t image_height = 1080;

#ifndef ANDROID
#define RTSP 0
#endif

enum {
    ARM_V4L2_TEST_STREAM_OUTPUT0,
    ARM_V4L2_TEST_STREAM_RAW,
    ARM_V4L2_TEST_STREAM_STATS,
    ARM_V4L2_TEST_STREAM_MAX
};

struct thread_info {
    pthread_t p_id;
    uint32_t status;//1: run 0:stop
    sem_t p_sem;

    aisp_calib_info_t calib;
    AML_ALG_CTX_S pstAlgCtx;
};

pthread_t tid[ARM_V4L2_TEST_STREAM_MAX];

struct ispIF ispIf;

/* config parameters */
struct thread_param {
    /* v4l2 variables */
    struct media_stream         v4l2_media_stream;
    void                        *v4l2_mem_param[NB_BUFFER_PARAM];
    void                        *v4l2_mem[NB_BUFFER];
    int                          param_buf_length;
    int                          stats_buf_length;
    struct sensorConfig          *sensorCfg;
    struct lensConfig            *lensCfg;

    char                        *mediadevname;
    /* video device info */
    char                        *devname;

    /* display device info */
    char                        *fbp;
    struct fb_var_screeninfo    vinfo;
    struct fb_fix_screeninfo    finfo;

    /* format info */
    uint32_t                    width;
    uint32_t                    height;
    uint32_t                    pixformat;
    uint32_t                    fmt_code;
    uint32_t                    wdr_mode;
    uint32_t                    exposure;

    /* for snapshot stream (non-zsl implementation) */
    int32_t                     capture_count;
    int32_t                     gdc_ctrl;
    int                         videofd;
    uint32_t                    c_width;
    uint32_t                    c_height;
    uint32_t                    a_ctrl;
    int                         fps;
    int                         pipe_idx;

    struct thread_info          info;
};

static int32_t manual_sensor_integration_time = -1;
static int32_t manual_sensor_analog_gain = -1;
static int32_t manual_sensor_digital_gain = -1;
static int32_t manual_isp_digital_gain = -1;

int media_stream_start(media_stream_t * stream, int type)
{
    return v4l2_video_stream_on(stream->video_ent0, type);
}

int media_stream_stop(media_stream_t * stream, int type)
{
    return v4l2_video_stream_off(stream->video_ent0, type);
}

static int getInterface() {
    auto lib = ::dlopen("libispaml.so", RTLD_NOW);
    if (!lib) {
        char const* err_str = ::dlerror();
        ERR("dlopen: error:%s", (err_str ? err_str : "unknown"));
        dlclose(lib);
        return -1;
    }
    ispIf.alg2User = (isp_alg2user)::dlsym(lib, "aisp_alg2user");
    if (!ispIf.alg2User) {
        char const* err_str = ::dlerror();
        ERR("dlsym: error:%s", (err_str ? err_str : "unknown"));
        return -1;
    }
    ispIf.alg2Kernel = (isp_alg2kernel)::dlsym(lib, "aisp_alg2kernel");
    if (!ispIf.alg2Kernel) {
        char const* err_str = ::dlerror();
        ERR("dlsym: error:%s", (err_str ? err_str : "unknown"));
        return -1;
    }
    ispIf.algEnable = (isp_enable)::dlsym(lib, "aisp_enable");
    if (!ispIf.algEnable) {
        char const* err_str = ::dlerror();
        ERR("dlsym: error:%s", (err_str ? err_str : "unknown"));
        return -1;
    }
    ispIf.algDisable = (isp_disable)::dlsym(lib, "aisp_disable");
    if (!ispIf.algDisable) {
        char const* err_str = ::dlerror();
        ERR("dlsym: error:%s", (err_str ? err_str : "unknown"));
        return -1;
    }
    ispIf.algFwInterface = (isp_fw_interface)::dlsym(lib, "aisp_fw_interface");
    if (!ispIf.algFwInterface) {
        char const* err_str = ::dlerror();
        ERR("dlsym: error:%s", (err_str ? err_str : "unknown"));
        return -1;
    }
    INFO("%s success", __FUNCTION__);
    return 0;
}

/**********
 * helper functions
 */
uint64_t getTimestamp() {
    struct timespec ts;
    int rc;

    rc = clock_gettime(0x00, &ts);
    if (rc == 0) {
        return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    } else {
        return 0;
    }
}

int64_t GetTimeMsec() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static int do_get_dma_buf_fd(int videofd, uint32_t index, uint32_t plane)
{
    struct v4l2_exportbuffer ex_buf;

    ex_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ex_buf.index = index;
    ex_buf.plane = plane;
    ex_buf.flags = 0;
    ex_buf.fd = -1;

    if (ioctl(videofd, VIDIOC_EXPBUF, &ex_buf))  {
        ERR("LIKE-0:Failed get dma buf fd\n");
    }

    return ex_buf.fd;
}

#ifdef dump_forever
void save_img(const char* prefix, void* buf, int length){
    FILE* fp = NULL;
    char name[60] = {'\0'};
#ifdef ANDROID
    sprintf(name, "/sdcard/DCIM/dst_%s.yuv", prefix);
#else
    sprintf(name, "/tmp/dst_%s.yuv", prefix);
#endif
    fp = fopen(name,"ab+");
    if (!fp) {
        printf("%s:Error open file\n", __func__);
        return;
    }
    fwrite(buf,1,length,fp);
    fclose(fp);
    fp = NULL;
    return;
}
#else
void save_img(const char* prefix, void *buff, unsigned int size, int flag, int num, int width, int height)
{
    char name[60] = {'\0'};
    int fd = -1;

    if (buff == NULL || size == 0) {
        ERR("%s:Error input param\n", __func__);
        return;
    }

    if (num > 1000)
        return;

    if (num % 10 != 0)
        return;

    #ifdef ANDROID
    sprintf(name, "/sdcard/DCIM/ca_%s-%d_dump-%d-%dx%d.yuv", prefix, flag, num, width, height);
    #else
    sprintf(name, "/tmp/ca_%s-%d_dump-%d-%dx%d.yuv", prefix, flag, num, width, height);
    #endif

    fd = open(name, O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        ERR("%s:Error open file\n", __func__);
        return;
    }
    write(fd, buff, size);
    close(fd);
}
#endif

int get_file_size(char *f_name)
{
    int f_size = -1;
    FILE *fp = NULL;

    if (f_name == NULL) {
        ERR("Error file name\n");
        return f_size;
    }

    fp = fopen(f_name, "rb");
    if (fp == NULL) {
        ERR("Error open file %s\n", f_name);
        return f_size;
    }

    fseek(fp, 0, SEEK_END);

    f_size = ftell(fp);

    fclose(fp);

    INFO("%s: size %d\n", f_name, f_size);

    return f_size;
}

void isp_param_init(struct media_stream v4l2_media_stream, struct thread_param *tparm)
{
    struct v4l2_requestbuffers  v4l2_rb;
    int rc, i;
    int total_mapped_mem=0;
    struct v4l2_buffer v4l2_buf;
    char alg_init[256*1024];

    stream_configuration_t     stream_config;
    stream_config.format.width = 1024;
    stream_config.format.height = 256;
    stream_config.format.nplanes   = 1;

    rc = setDataFormat(&v4l2_media_stream, &stream_config);
    if (rc < 0) {
        ERR("Failed to set stats format");
        return;
    }

    rc = setConfigFormat(&v4l2_media_stream, &stream_config);
    if (rc < 0) {
        ERR("Failed to set param format");
        return;
    }

    /* request buffers */
    memset (&v4l2_rb, 0, sizeof (struct v4l2_requestbuffers));
    v4l2_rb.count  = NB_BUFFER;
    v4l2_rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_rb.memory = V4L2_MEMORY_MMAP;
    rc = v4l2_video_req_bufs(tparm->v4l2_media_stream.video_stats, &v4l2_rb);
    if (rc < 0) {
        ERR("Error: request buffer.\n");
        return;
    }

    memset (&v4l2_rb, 0, sizeof (struct v4l2_requestbuffers));
    v4l2_rb.count  = NB_BUFFER_PARAM;
    v4l2_rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_rb.memory = V4L2_MEMORY_MMAP;
    rc = v4l2_video_req_bufs(v4l2_media_stream.video_param, &v4l2_rb);
    if (rc < 0) {
        ERR("Failed to req_bufs");
        return;
    }

    /* map stats buffers */
    for (i = 0; i < NB_BUFFER; i++) {
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        v4l2_buf.index   = i;
        v4l2_buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory  = V4L2_MEMORY_MMAP;
        rc = v4l2_video_query_buf(tparm->v4l2_media_stream.video_stats, &v4l2_buf);
        if (rc < 0) {
            ERR("Error: query buffer %d.\n", rc);
            return;
        }

        if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            tparm->stats_buf_length = v4l2_buf.length;
            INFO("video capture. length: %u offset: %u\n", v4l2_buf.length, v4l2_buf.m.offset);
            tparm->v4l2_mem[i] = mmap (0, v4l2_buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                tparm->v4l2_media_stream.video_stats->fd, v4l2_buf.m.offset);
            ++total_mapped_mem;
            INFO("Buffer[%d] mapped at address 0x%p total_mapped_mem:%d.\n", i, tparm->v4l2_mem[i], total_mapped_mem);
        }
        if (tparm->v4l2_mem[i] == MAP_FAILED) {
            ERR("Error: mmap buffers.\n");
            return;
        }
    }

    /* map buffers */
    for (i = 0; i < NB_BUFFER_PARAM; i++) {
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        v4l2_buf.index   = i;
        v4l2_buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory  = V4L2_MEMORY_MMAP;
        rc = v4l2_video_query_buf(v4l2_media_stream.video_param, &v4l2_buf);
        if (rc < 0) {
            ERR("Failed to query bufs");
            return;
        }

        if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            tparm->param_buf_length = v4l2_buf.length;
            INFO("[T#2] type video capture. length: %u offset: %u\n", v4l2_buf.length, v4l2_buf.m.offset);
            tparm->v4l2_mem_param[i] = mmap (0, v4l2_buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                v4l2_media_stream.video_param->fd, v4l2_buf.m.offset);
            INFO("[T#2] Buffer[0] mapped at address 0x%p total_mapped_mem:%d.\n", tparm->v4l2_mem_param[i], total_mapped_mem);
        }
        if (tparm->v4l2_mem_param[i] == MAP_FAILED) {
            ERR("[T#2] Error: mmap buffers.\n");
            return;
        }
    }

    static int ret = getInterface();
    if (ret == -1) {
        ERR("Failed to getInterface");
        return ;
    }

    tparm->sensorCfg = matchSensorConfig(&v4l2_media_stream);
    if (tparm->sensorCfg == nullptr) {
        ERR("Failed to matchSensorConfig");
        return ;
    }

    tparm->lensCfg = matchLensConfig(&v4l2_media_stream);
    if (tparm->lensCfg != nullptr) {
        lens_set_entity(tparm->lensCfg, v4l2_media_stream.lens_ent);
        lens_control_cb(tparm->lensCfg, &tparm->info.pstAlgCtx.stLensFunc);
    }

#ifdef WDR_ENABLE
    cmos_set_sensor_entity(tparm->sensorCfg, v4l2_media_stream.sensor_ent, 1);
#else
    cmos_set_sensor_entity(tparm->sensorCfg, v4l2_media_stream.sensor_ent, 0);
#endif
    cmos_sensor_control_cb(tparm->sensorCfg, &tparm->info.pstAlgCtx.stSnsExp);
    cmos_get_sensor_calibration(tparm->sensorCfg, v4l2_media_stream.sensor_ent, &tparm->info.calib);

    (ispIf.algEnable)(0, &tparm->info.pstAlgCtx, &tparm->info.calib);
    memset(alg_init, 0, sizeof(alg_init));

    (ispIf.alg2User)(0, alg_init);
    (ispIf.alg2Kernel)(0, tparm->v4l2_mem_param[0]);

    /* queue buffers */
    DBG("[T#0] begin to Queue buf.\n");
    for (i = 0; i < NB_BUFFER; ++i) {
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        v4l2_buf.index   = i;
        v4l2_buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory  = V4L2_MEMORY_MMAP;
        rc = v4l2_video_q_buf( tparm->v4l2_media_stream.video_stats, &v4l2_buf );
        if (rc < 0) {
            ERR("Error: queue buffers, rc:%d i:%d\n",rc, i);
            return;
        }
    }

    for (i = 0; i < NB_BUFFER_PARAM; ++i) {
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        v4l2_buf.index   = i;
        v4l2_buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory  = V4L2_MEMORY_MMAP;
        rc = v4l2_video_q_buf( v4l2_media_stream.video_param, &v4l2_buf );
        if (rc < 0) {
            ERR("Error: queue buffers, rc:%d\n", rc);
        }
    }

    rc = v4l2_video_stream_on(tparm->v4l2_media_stream.video_stats, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (rc < 0) {
        ERR("[T#0] Error: streamon.\n");
        return;
    }

    rc = v4l2_video_stream_on(v4l2_media_stream.video_param, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (rc < 0) {
        ERR("[T#0] Error: streamon.\n");
        return;
    }

    DBG("[T#0] Finished alg_param_init.\n");
}

void * video_thread(void *arg)
{
    struct v4l2_capability      v4l2_cap;
    struct v4l2_format          v4l2_fmt;
    struct v4l2_requestbuffers  v4l2_rb;

    int                         v4l2_buf_length = 0;

    int                         dma_fd = -1;

    void                        *v4l2_mem[NB_BUFFER*VIDEO_MAX_PLANES];
    int                         v4l2_dma_fd[NB_BUFFER * VIDEO_MAX_PLANES] = {0};
    int                         total_mapped_mem=0;

    /* thread parameters */
    struct thread_param         *tparm = (struct thread_param *)arg;
    pthread_t                   cur_pthread = pthread_self();
    int                         stream_type = -1;

    /* condition & loop flags */
    int                         rc = 0;
    int                         i,j;

    uint32_t          v4l2_enum_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    uint64_t display_count = 0;
    int64_t start, end;

    /**************************************************
     * find thread id
     *************************************************/
    for (i = 0; i < ARM_V4L2_TEST_STREAM_MAX; i++) {
        if (cur_pthread == tid[i]) {
            stream_type = i;
            break;
        }
    }

    for (i = 0; i < NB_BUFFER * VIDEO_MAX_PLANES; i++) {
        v4l2_dma_fd[i] = -1;
    }
    tparm->v4l2_media_stream.media_dev = media_device_new(tparm->mediadevname);

    rc = mediaStreamInit(&tparm->v4l2_media_stream, tparm->v4l2_media_stream.media_dev);
    if (0 != rc) {
        ERR("[T#%d] The %s device init fail.\n", stream_type, tparm->mediadevname);
        return NULL;
    }
    INFO("[T#%d] The %s device was opened successfully. stream init ok\n", stream_type, tparm->mediadevname);
    android::staticPipe::fetchPipeMaxResolution(&tparm->v4l2_media_stream, tparm->width, tparm->height);
    /* check capability */
    memset (&v4l2_cap, 0, sizeof (struct v4l2_capability));
    rc = v4l2_video_get_capability(tparm->v4l2_media_stream.video_ent0, &v4l2_cap);
    if (rc < 0) {
        ERR ("[T#%d] Error: get capability.\n", stream_type);
        return NULL;
    }
    INFO("[T#%d] VIDIOC_QUERYCAP: cap.driver = %s, capabilities=0x%x, device_caps:0x%x\n",
        stream_type, v4l2_cap.driver, v4l2_cap.capabilities, v4l2_cap.device_caps);

    media_set_wdrMode(&tparm->v4l2_media_stream, 0);
    media_set_wdrMode(&tparm->v4l2_media_stream, tparm->wdr_mode);
    /* config & set format */
    stream_configuration     stream_config ;
    memset(&stream_config, 0, sizeof(stream_configuration));
    stream_config.format.width =  tparm->width;
    stream_config.format.height = tparm->height;
    stream_config.format.fourcc = tparm->pixformat;
    stream_config.format.code   = tparm->fmt_code;
    stream_config.format.nplanes   = 1;

    stream_config.vformat[0].width  = tparm->width;
    stream_config.vformat[0].height = tparm->height;
    stream_config.vformat[0].fourcc = tparm->pixformat;

    rc = mediaStreamConfig(&tparm->v4l2_media_stream, &stream_config);
    if (rc < 0) {
        ERR("[T#%d] Error: config stream %d.\n", stream_type, rc);
        return NULL;
    }

    memset (&v4l2_fmt, 0, sizeof (struct v4l2_format));
    v4l2_fmt.type                    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_fmt.fmt.pix_mp.width        = stream_config.format.width;
    v4l2_fmt.fmt.pix_mp.height       = stream_config.format.height;
    v4l2_fmt.fmt.pix_mp.pixelformat  = stream_config.format.fourcc;
    v4l2_fmt.fmt.pix_mp.field        = V4L2_FIELD_ANY;

    rc = v4l2_video_get_format(tparm->v4l2_media_stream.video_ent0, &v4l2_fmt);
    if (rc < 0) {
        ERR("[T#%d] Error: get video format %d.\n", stream_type, rc);
        return NULL;
    }

    /* Selection */
    //v4l2_video_crop(tparm->v4l2_media_stream.video_ent0, 1280, 720);

    /* request buffers */
    memset (&v4l2_rb, 0, sizeof (struct v4l2_requestbuffers));
    v4l2_rb.count  = NB_BUFFER;
    v4l2_rb.type   = v4l2_enum_type;
    v4l2_rb.memory = V4L2_MEMORY_MMAP;
    rc = v4l2_video_req_bufs(tparm->v4l2_media_stream.video_ent0, &v4l2_rb);
    if (rc < 0) {
        ERR("[T#%d] Error: request buffer.\n", stream_type);
        return NULL;
    }

    INFO("[T#%d] request buf ok\n", stream_type);

    /* map buffers */
    for (i = 0; i < NB_BUFFER; i++) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane buf_planes[v4l2_fmt.fmt.pix_mp.num_planes];
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        v4l2_buf.index   = i;
        v4l2_buf.type    = v4l2_enum_type;
        v4l2_buf.memory  = V4L2_MEMORY_MMAP;
        if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            v4l2_buf.m.planes=buf_planes;
            v4l2_buf.length = v4l2_fmt.fmt.pix_mp.num_planes;
        }
        rc = v4l2_video_query_buf(tparm->v4l2_media_stream.video_ent0, &v4l2_buf);
        if (rc < 0) {
            ERR("[T#%d] Error: query buffer %d.\n", stream_type, rc);
            return NULL;
        }

        if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            v4l2_buf_length = v4l2_buf.length;
            INFO("[T#%d] type video capture. length: %u offset: %u\n", stream_type, v4l2_buf.length, v4l2_buf.m.offset);
            v4l2_mem[i] = mmap (0, v4l2_buf.length, PROT_READ, MAP_SHARED,
                tparm->v4l2_media_stream.video_ent0->fd, v4l2_buf.m.offset);
            ++total_mapped_mem;
            INFO("[T#%d] Buffer[%d] mapped at address 0x%p total_mapped_mem:%d.\n", stream_type, i, v4l2_mem[i], total_mapped_mem);
        }
        else if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            for (j=0;j<v4l2_fmt.fmt.pix_mp.num_planes;j++) {
                v4l2_buf_length = v4l2_buf.m.planes[j].length;
                dma_fd = do_get_dma_buf_fd(tparm->v4l2_media_stream.video_ent0->fd, i, j);
                INFO("[T#%d] plane:%d multiplanar length: %u offset: %u, dma_fd:%d\n",
                    stream_type, j, v4l2_buf.m.planes[j].length, v4l2_buf.m.planes[j].m.mem_offset, dma_fd);
                v4l2_mem[i*v4l2_fmt.fmt.pix_mp.num_planes + j] = mmap (0, v4l2_buf.m.planes[j].length, PROT_READ, MAP_SHARED,
                    tparm->v4l2_media_stream.video_ent0->fd, v4l2_buf.m.planes[j].m.mem_offset);
                v4l2_dma_fd[i*v4l2_fmt.fmt.pix_mp.num_planes + j] = dma_fd;
                ++total_mapped_mem;
                INFO("[T#%d] Buffer[%d] mapped at address %p total_mapped_mem:%d.\n", stream_type,i*v4l2_fmt.fmt.pix_mp.num_planes + j, v4l2_mem[i*v4l2_fmt.fmt.pix_mp.num_planes + j],total_mapped_mem);
            }
        }
        if (v4l2_mem[i] == MAP_FAILED) {
            ERR("[T#%d] Error: mmap buffers.\n", stream_type);
            return NULL;
        }
        MSG("[T#%d] map  %d ok, 0x%p\n", stream_type, i, v4l2_mem[i]);
    }

    /* queue buffers */
    DBG("[T#%d] begin to Queue buf.\n", stream_type);

    for (i = 0; i < NB_BUFFER; ++i) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane buf_planes[v4l2_fmt.fmt.pix_mp.num_planes];
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        v4l2_buf.index   = i;
        v4l2_buf.type    = v4l2_enum_type;
        v4l2_buf.memory  = V4L2_MEMORY_MMAP;
        if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            v4l2_buf.m.planes=buf_planes;
            v4l2_buf.length = v4l2_fmt.fmt.pix_mp.num_planes;
        }
        rc = v4l2_video_q_buf( tparm->v4l2_media_stream.video_ent0, &v4l2_buf );
        if (rc < 0) {
            ERR("[T#%d] Error: queue buffers, rc:%d i:%d\n",stream_type, rc, i);
            return NULL;
        }
    }
    DBG("[T#%d] Queue buf done.\n", stream_type);

    isp_param_init(tparm->v4l2_media_stream, tparm);

    /**************************************************
     * V4L2 stream on, get buffers
     *************************************************/
    /* stream on */
    //int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int type = v4l2_enum_type;
    rc =  media_stream_start(&tparm->v4l2_media_stream, type); // v4l2_video_stream_on(v4l2_media_stream->video_ent0, type);
    if (rc < 0) {
        ERR("[T#%d] Error: streamon.\n", stream_type);
        return NULL;
    }

    INFO("[T#%d] Video stream is on.\n", stream_type);

    sem_post(&tparm->info.p_sem);

    start = GetTimeMsec();

    /* dequeue and display */
    do {
        struct v4l2_buffer v4l2_buf;

        int idx = -1;
        if (idx == 0)
            ;
        // dqbuf from video node
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        //v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.type = v4l2_enum_type;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            //v4l2_buf.m.planes=buf_planes;
            v4l2_buf.m.planes = (struct v4l2_plane *)malloc(v4l2_fmt.fmt.pix_mp.num_planes*sizeof(struct v4l2_plane));
            v4l2_buf.length = v4l2_fmt.fmt.pix_mp.num_planes;
        }
        //INFO("beg to  dq buffer.\n");
        rc = v4l2_video_dq_buf(tparm->v4l2_media_stream.video_ent0, &v4l2_buf);
        if (rc < 0) {
            ERR ("Error: dequeue buffer.\n");
            if (v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                free(v4l2_buf.m.planes);
            break;
        }
        idx = v4l2_buf.index;
#ifdef dump_forever
        if (strstr(tparm->mediadevname, "/dev/media0")) {
            save_img("mif_0", v4l2_mem[idx], tparm->width * tparm->height*3/2);
        }
        if (strstr(tparm->mediadevname, "/dev/media1")) {
            save_img("mif_1", v4l2_mem[idx], tparm->width * tparm->height*3/2);
        }
#else
        if (strstr(tparm->mediadevname, "/dev/media0")) {
            save_img("mif_0",v4l2_mem[idx], tparm->width * tparm->height*3/2, stream_type, display_count, tparm->width, tparm->height);
        }
        if (strstr(tparm->mediadevname, "/dev/media1")) {
            save_img("mif_1",v4l2_mem[idx], tparm->width * tparm->height*3/2, stream_type, display_count, tparm->width, tparm->height);
        }
#endif

#if RTSP
        lib_put_frame(v4l2_mem[idx], tparm->width * tparm->height * 3 / 2);
#endif
        //INFO("[T#%d] todo: do something with buf. capture count %d \n ", stream_type, tparm->capture_count);
        usleep(1000*10);

        rc = v4l2_video_q_buf(tparm->v4l2_media_stream.video_ent0, &v4l2_buf);
        if (rc < 0) {
            ERR ("[T#%d] Error: queue buffer.\n", stream_type);
            break;
        }
        //INFO("[T#%d] q buf back idx %d \n", stream_type, v4l2_buf.index);

        display_count++;
        if ((display_count % 100 == 0)) {
            end = GetTimeMsec();
            end = end - start;
            #ifdef ANDROID
            INFO("[T#%d] stream port %s fps is : %ld\n",stream_type, "raw", (100 * 1000) /end);
            #else
            INFO("[T#%d] stream port %s fps is : %lld\n",stream_type, "raw", (100 * 1000) /end);
            #endif
            start = GetTimeMsec();
        }

        if (tparm->capture_count > 0)
            tparm->capture_count--;

    } while (tparm->capture_count > 0);


    INFO("[T#%d] media stream stop \n",stream_type);

    /* stream off */
    rc = media_stream_stop(&tparm->v4l2_media_stream, type);

    /* unmap buffers */
    for (i = 0; i < NB_BUFFER; i++) {
        munmap (v4l2_mem[i], v4l2_buf_length);
        if (v4l2_dma_fd[i] >= 0)
            close(v4l2_dma_fd[i]);
    }


    MSG("[T#%d]  close success. thread exit ...\n", stream_type);
    return 0;
}

void * stats_thread(void *arg)
{
    /* thread parameters */
    struct thread_param         *tparm = (struct thread_param *)arg;
    pthread_t                   cur_pthread = pthread_self();
    int                         stream_type = -1;

    /* condition & loop flags */
    int                         rc = 0;
    int                         i;

    uint32_t          v4l2_enum_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    uint64_t display_count = 0;
    int64_t start, end;

    /**************************************************
     * find thread id
     *************************************************/
    for (i = 0; i < ARM_V4L2_TEST_STREAM_MAX; i++) {
        if (cur_pthread == tid[i]) {
            stream_type = i;
            break;
        }
    }

    sem_wait(&tparm->info.p_sem);

    INFO("[T#%d] Video stream is on.\n", stream_type);

    start = GetTimeMsec();

    if (manual_sensor_integration_time > 0 || manual_sensor_analog_gain > 0
            || manual_sensor_digital_gain > 0 || manual_isp_digital_gain > 0) {
        aisp_api_type_t param;
        isp_exposure_attr_s data;
        aisp_api_type_t *api_type = &param;
        isp_exposure_attr_s *attr = &data;
        api_type->u8Direction = AML_CMD_SET;
        api_type->u8CmdType = 0;// not used
        api_type->u8CmdId = AML_MBI_ISP_ExposureAttr;
        api_type->u32Value = 0;// not used
        api_type->pData = (uint32_t *)&data;
        INFO("manual mode: integration_time %d, analog_gain %d, digital_gain %d, isp_digital_gain %d\n",
            manual_sensor_integration_time, manual_sensor_analog_gain,
            manual_sensor_digital_gain, manual_isp_digital_gain);

        attr->bByPass = mbp_true;
        if (manual_sensor_integration_time > 0) {
            attr->stManual.enExpTimeOpType = OP_TYPE_MANUAL;
            attr->stManual.u32ExpTime = manual_sensor_integration_time;
        } else {
                attr->stManual.enExpTimeOpType = OP_TYPE_AUTO;
        }
        if (manual_sensor_analog_gain > 0) {
            attr->stManual.enAGainOpType  = OP_TYPE_MANUAL;
            attr->stManual.u32AGain = manual_sensor_analog_gain;
        } else {
            attr->stManual.enAGainOpType = OP_TYPE_AUTO;
        }
        if (manual_sensor_digital_gain > 0) {
            attr->stManual.enDGainOpType  = OP_TYPE_MANUAL;
            attr->stManual.u32DGain = manual_sensor_digital_gain;
        } else {
            attr->stManual.enDGainOpType = OP_TYPE_AUTO;
        }
        if (manual_isp_digital_gain > 0) {
            attr->stManual.enISPDGainOpType  = OP_TYPE_MANUAL;
            attr->stManual.u32ISPDGain  = manual_isp_digital_gain;
        } else {
            attr->stManual.enISPDGainOpType  = OP_TYPE_AUTO;
        }

        ispIf.algFwInterface(0, api_type);
    }

    /* dequeue and display */
    do {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_buffer v4l2_buf_param;

        int idx = -1;
        int idx1 = -1;

        // dqbuf from video node
        memset (&v4l2_buf, 0, sizeof (struct v4l2_buffer));
        v4l2_buf.type = v4l2_enum_type;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        rc = v4l2_video_dq_buf(tparm->v4l2_media_stream.video_stats, &v4l2_buf);
        if (rc < 0) {
            ERR ("Error: dequeue buffer.\n");
            usleep(10000);
            continue;
        }

        memset (&v4l2_buf_param, 0, sizeof (struct v4l2_buffer));
        v4l2_buf_param.type = v4l2_enum_type;
        v4l2_buf_param.memory = V4L2_MEMORY_MMAP;
        rc = v4l2_video_dq_buf(tparm->v4l2_media_stream.video_param, &v4l2_buf_param);
        if (rc < 0) {
            ERR ("Error: video param dequeue buffer.\n");
            rc = v4l2_video_q_buf(tparm->v4l2_media_stream.video_stats, &v4l2_buf);
            usleep(10000);
            continue;
        }

        idx = v4l2_buf.index;
        idx1 = v4l2_buf_param.index;

        (ispIf.alg2User)(0, tparm->v4l2_mem[idx]);
        (ispIf.alg2Kernel)(0, tparm->v4l2_mem_param[idx1]);

        usleep(1000*5);

        rc = v4l2_video_q_buf(tparm->v4l2_media_stream.video_stats,  &v4l2_buf);
        if (rc < 0) {
            ERR ("[T#%d] Error: queue buffer.\n", stream_type);
            break;
        }
        rc = v4l2_video_q_buf(tparm->v4l2_media_stream.video_param,  &v4l2_buf_param);

        display_count++;
        if ((display_count % 100 == 0)) {
            end = GetTimeMsec();
            end = end - start;
            #ifdef ANDROID
            INFO("[T#%d] stream port %s fps is : %ld\n",stream_type, "stats", (100 * 1000) /end);
            #else
            INFO("[T#%d] stream port %s fps is : %lld\n",stream_type, "stats", (100 * 1000) /end);
            #endif
            start = GetTimeMsec();
        }

        /*if (tparm->capture_count > 0)
            tparm->capture_count--;*/

    } while (tparm->capture_count > 0);


    INFO("[T#%d] media stream stop \n",stream_type);

    /* stream off */
    rc = v4l2_video_stream_off(tparm->v4l2_media_stream.video_stats, v4l2_enum_type);

    /* unmap buffers */
    for (i = 0; i < NB_BUFFER; i++) {
        munmap (tparm->v4l2_mem[i], tparm->stats_buf_length);
    }

    /* stream off */
    rc = v4l2_video_stream_off(tparm->v4l2_media_stream.video_param, v4l2_enum_type);

    /* unmap buffers */
    for (i = 0; i < NB_BUFFER_PARAM; i++) {
        munmap (tparm->v4l2_mem_param[i], tparm->param_buf_length);
    }

    MSG("[T#%d]  close success. thread exit ...\n", stream_type);

    return 0;
}


/**********
 * raw capture functions
 */
 int prepareOutput0Capture(struct thread_param * tparam) {

    return pthread_create(&tid[ARM_V4L2_TEST_STREAM_OUTPUT0], NULL, &video_thread, tparam);
}
int prepareRawCapture(struct thread_param * tparam) {

    return pthread_create(&tid[ARM_V4L2_TEST_STREAM_RAW], NULL, &video_thread, tparam);
}

int prepareStatsCapture(struct thread_param * tparam) {

    return pthread_create(&tid[ARM_V4L2_TEST_STREAM_STATS], NULL, &stats_thread, tparam);
}

void finishOutput0Capture(struct thread_param * tparam) {
    INFO("join and wait for output subthread to exit...... \n");

    pthread_join(tid[ARM_V4L2_TEST_STREAM_OUTPUT0], NULL);
    tparam->capture_count = 0;
}


void finishRawCapture(struct thread_param * tparam) {
    INFO("join and wait for raw subthread to exit...... \n");

    pthread_join(tid[ARM_V4L2_TEST_STREAM_RAW], NULL);
    tparam->capture_count = 0;
}

void finishStatsCapture(struct thread_param * tparam) {
    INFO("join and wait for stats subthread to exit...... \n");

    pthread_join(tid[ARM_V4L2_TEST_STREAM_STATS], NULL);
    tparam->capture_count = 0;
}

void usage(char * prog){
    INFO("%s\n", prog);
    INFO("usage:\n");
    INFO(" example   : ./v4l2_test_raw  -n 100 -m /dev/media0 -p 0 \n");
    //INFO("    f : fmt       : 0: rgb24  1:nv12 \n");
    INFO("    m : media dev name: /dev/media0 or /dev/media1 \n");
    INFO("    p : pipe selection  : 0 1 default 0\n");
    //INFO("    e : exposure value	  : min 1, max 4, default is 1\n");
    //INFO("    b : fbdev			 : default: /dev/fb0\n");
    INFO("    v : videodev		 : default: /dev/video0\n");
    INFO("    n : frame count \n");
}

/**********
 * main function
*/

void parse_fmt_res(uint8_t fmt, int res, uint32_t wdr_mode_prm, uint32_t exposure_prm, void *param)
{
    struct thread_param *t_param = NULL;

    uint32_t pixel_format = 0;
    uint32_t width        = 0;
    uint32_t height       = 0;
    uint32_t wdr_mode     = 0;
    uint32_t exposure     = 0;

    if (param == NULL) {
        ERR("Error input param\n");
        return;
    }

    t_param = (struct thread_param *)param;

    switch (fmt) {
    case 0:
        pixel_format = V4L2_PIX_FMT_RGB24;
        break;
    case 1:
        pixel_format = V4L2_PIX_FMT_NV12;
        break;
    case 2:
        pixel_format = V4L2_PIX_FMT_SBGGR16;
        break;
    default:
        ERR("Invalid FR_OUT fmt %d !\n", fmt);
        break;
    }

    switch (res) {
    case 0:
        width = 3840;
        height = 2160;
        break;
    case 1:
        width = 1920;
        height = 1080;
        break;
    case 2:
        width = 1280;
        height = 720;
        break;
    case 3:
        width = 640;
        height = 480;
        break;
    case 4:
        width = 1280;
        height = 960;
        break;
    case 5:
        width = 320;
        height = 240;
        break;
    case 6:
        width = 228;
        height = 228;
        break;
    default:
        ERR("Invalid resolution %d !\n", res);
        break;
    }

    switch (wdr_mode_prm) {
    case 0:
        wdr_mode = 0;
        break;
    case 1:
        wdr_mode = 1;
        break;
    case 2:
        wdr_mode = 2;
        break;
    default:
        ERR("Invalid wdr mode %d !\n", wdr_mode_prm);
        break;
    }

    switch (exposure_prm) {
    case 1:
        exposure = 1;
        break;
    case 2:
        exposure = 2;
        break;
    case 3:
        exposure = 3;
        break;
    case 4:
        exposure = 4;
        break;
    default:
        ERR("Invalid exposure %d !\n", exposure_prm);
        break;
    }

    t_param->pixformat = pixel_format;
    t_param->width     = width;
    t_param->height    = height;
    t_param->wdr_mode  = wdr_mode;
    t_param->exposure  = exposure;

    ERR("\n parse_fmt_res: pixel fmt 0x%x, width %d, height %d, wdr_mode %d, exposure %d\n",
        pixel_format, width, height, wdr_mode, exposure);
}

int main(int argc, char *argv[])
{

    //char *fbdevname = "/dev/fb0";
    //char *v4ldevname = "/dev/video60";
    char v4ldevname[128] = "/dev/video3";
    char v4l2mediadevname[128] = "/dev/media0";

    int count = -100;
    int pipe_idx = 0;

    if (argc < 3) {
        usage(argv[0]);
        return -1;
    }

    //int optind = 0;
    int c;

    while (optind < argc) {
        if ((c = getopt (argc, argv, "c:p:F:f:D:R:r:d:N:n:w:e:b:v:t:x:g:I:W:H:Y:Z:a:M:L:A:G:S:K:m:s:")) != -1) {
            switch (c) {
            case 'n':
                count = atoi(optarg);
                break;
            case 'b':
                //fbdevname = optarg;
                break;
            case 'v':
                strcpy(v4ldevname, optarg);
                break;
            case 'm':
                strcpy(v4l2mediadevname, optarg);
                break;
            case 'p':
                pipe_idx = atoi(optarg);
                break;
            case 'L':
                manual_sensor_integration_time = atoi(optarg);
                break;
            case 'A':
                manual_sensor_analog_gain = atoi(optarg);
                break;
            case 'G':
                manual_sensor_digital_gain = atoi(optarg);
                break;
            case 'S':
                manual_isp_digital_gain = atoi(optarg);
                break;
            case '?':
                usage(argv[0]);
                exit(1);
            }
        }else{
            MSG("Invalid argument %s\n",argv[optind]);
            usage(argv[0]);
            exit(1);
        }
    }

#if RTSP
    lib_initialization(image_width, image_height);

    if (fork() == 0)
        lib_run();
#endif
    struct thread_param tparam_raw = {
        .mediadevname = v4l2mediadevname,
        .devname    = v4ldevname,
        .fbp        = 0,

        .width      = 1920,
        .height     = 1080,
        .pixformat  = V4L2_PIX_FMT_NV12,//V4L2_PIX_FMT_SBGGR10, //V4L2_PIX_FMT_Y12,//V4L2_PIX_FMT_NV12,//V4L2_PIX_FMT_SRGGB12,//

#if defined (DUAL_CAMERA)
        .fmt_code   = MEDIA_BUS_FMT_SRGGB12_1X12,
        .wdr_mode   = ISP_SDR_DCAM_MODE,
#elif defined (WDR_ENABLE)
        .fmt_code   = MEDIA_BUS_FMT_SBGGR10_1X10,//MEDIA_BUS_FMT_SRGGB12_1X12,//
        .wdr_mode   = WDR_MODE_2To1_LINE,//WDR_MODE_2To1_LINE,
#else
        .fmt_code   = MEDIA_BUS_FMT_SRGGB12_1X12,//
        .wdr_mode   = WDR_MODE_NONE,
#endif
        .exposure   = 0,
        .capture_count = count,
        .pipe_idx = pipe_idx
    };

    sem_init(&tparam_raw.info.p_sem, 0, 0);

    /* turn on stats capture stream */
    if (prepareStatsCapture(&tparam_raw) != 0) {
        ERR("Error: Can't start raw stream, cancelling capture.\n");
    }

    /* turn on output0 capture stream */
    if (prepareOutput0Capture(&tparam_raw) != 0) {
        ERR("Error: Can't start raw stream, cancelling capture.\n");
    }

    /* turn on raw capture stream */
    //if (prepareRawCapture(&tparam_raw) != 0) {
    //    ERR("Error: Can't start raw stream, cancelling capture.\n");
    //}
    sleep(1);

    /* wait raw capture stream to be turned off */
    finishOutput0Capture(&tparam_raw);
    //finishRawCapture(&tparam_raw);
    finishStatsCapture(&tparam_raw);

    /**************************************************
     * Terminating threads and process
     *************************************************/
    MSG("terminating all threads ...\n");


    MSG("terminating v4l2 test app, thank you ...\n");

    return 0;
}


