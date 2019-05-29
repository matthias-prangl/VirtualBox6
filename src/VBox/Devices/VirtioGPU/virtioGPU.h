#ifndef VIRTIOGPU
#define VIRTIOGPU

#include "../VirtIOModern/rtsgseg_helpers.h"
#include "../VirtIOModern/virtio.h"
#include "../VirtIOModern/virtioPCI.h"
#include "virtio_gpu.h"
#include <VBox/vmm/pdmdev.h>
#include <pixman.h>
#include <vector>

#define VIRTIOGPU_NAME_FMT "virtioGPU%d"
#define VIRTIOGPU_PCI_CLASS 0x0300
#define VIRTIOGPU_ID 16
#define VIRTIOGPU_N_QUEUES 2

#define VIRTIO_GPU_FILL_CMD(out)                                               \
  do {                                                                         \
    size_t s;                                                                  \
    s = RTSGSEG_to_buf(cmd->elem.out_sg, cmd->elem.out_num, 0, &out,           \
                       sizeof(out));                                           \
    if (s != sizeof(out)) {                                                    \
      return;                                                                  \
    }                                                                          \
  } while (0)

struct virtio_gpu_simple_resource {
  uint32_t resource_id;
  uint32_t width;
  uint32_t height;
  uint32_t format;
  uint64_t *addrs;
  struct RTSGSEG *iov;
  unsigned int iov_cnt;
  uint32_t scanout_bitmask;
  pixman_image_t *image;
  uint64_t hostmem;
  PGMPAGEMAPLOCK *locks;
};

typedef struct DisplaySurface {
  pixman_format_code_t format;
  pixman_image_t *image;
  uint8_t flags;
} DisplaySurface;

struct virtio_gpu_scanout {
  // QemuConsole *con;
  DisplaySurface *ds;
  uint32_t width, height;
  int x, y;
  int invalidate;
  uint32_t resource_id;
  struct virtio_gpu_update_cursor cursor;
  // QEMUCursor *current_cursor;
};

struct virtio_gpu_requested_state {
  uint32_t width, height;
  int x, y;
};

struct virtio_gpu_conf {
  uint64_t max_hostmem;
  uint32_t max_outputs;
  uint32_t flags;
  uint32_t xres;
  uint32_t yres;
};

struct virtio_gpu_ctrl_command {
  VirtQueueElement elem;
  VirtQueue *vq;
  struct virtio_gpu_ctrl_hdr cmd_hdr;
  uint32_t error;
  bool finished;
};

typedef struct VirtioGPU {
  VirtioPCIDevice pciDev;
  VirtioDevice vdev;

  //   QEMUBH *ctrl_bh;
  //   QEMUBH *cursor_bh;
  VirtQueue *ctrl_vq;
  VirtQueue *cursor_vq;

  int enable;
  std::vector<struct virtio_gpu_simple_resource *> reslist;
  std::vector<struct virtio_gpu_ctrl_command *> cmdq;
  std::vector<struct virtio_gpu_ctrl_command *> fenceq;

  struct virtio_gpu_scanout scanout[VIRTIO_GPU_MAX_SCANOUTS];
  struct virtio_gpu_requested_state req_state[VIRTIO_GPU_MAX_SCANOUTS];

  struct virtio_gpu_conf conf;
  uint64_t hostmem;
  int enabled_output_bitmask;
  struct virtio_gpu_config virtio_config;

  bool use_virgl_renderer;
  bool renderer_inited;
  int renderer_blocked;
  bool renderer_reset;
  //   QEMUTimer *fence_poll;
  //   QEMUTimer *print_stats;

  uint32_t inflight;
  struct {
    uint32_t max_inflight;
    uint32_t requests;
    uint32_t req_3d;
    uint32_t bytes_3d;
  } stats;

  //   Error *migration_blocker;
} VirtioGPU;

DECLCALLBACK(int)
virtioGPUConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg);

const PDMDEVREG g_virtioGPU = {
    PDM_DEVREG_VERSION,
    "virtioGPU",     // device name
    "VBoxDDRC.rc",   // don't care; only if PDM_DEVREG_FLAGS_RC set
    "VBoxDDR0.r0",   // don't care; only if PDM_DEVREG_FLAGS_RC set
    "virtio GPU.\n", // device description
    PDM_DEVREG_FLAGS_DEFAULT_BITS,
    PDM_DEVREG_CLASS_GRAPHICS, // device class
    1,                         // max num. instances
    sizeof(VirtioGPU),
    virtioGPUConstruct, // pfnConstruct
    NULL,               // pfnDestruct
    NULL,               // pfnRelocate
    NULL,               // pfnMemSetup.
    NULL,               // pfnPowerOn
    NULL,               // pfnReset
    NULL,               // pfnSuspend
    NULL,               // pfnResume
    NULL,               // pfnAttach
    NULL,               // pfnDetach
    NULL,               // pfnQueryInterface
    NULL,               // pfnInitComplete
    NULL,               // pfnPowerOff
    NULL,               // pfnSoftReset
    PDM_DEVREG_VERSION  // u32VersionEnd
};

#ifdef RT_BIG_ENDIAN
#define PIXMAN_BE_r8g8b8 PIXMAN_r8g8b8
#define PIXMAN_BE_x8r8g8b8 PIXMAN_x8r8g8b8
#define PIXMAN_BE_a8r8g8b8 PIXMAN_a8r8g8b8
#define PIXMAN_BE_b8g8r8x8 PIXMAN_b8g8r8x8
#define PIXMAN_BE_b8g8r8a8 PIXMAN_b8g8r8a8
#define PIXMAN_BE_r8g8b8x8 PIXMAN_r8g8b8x8
#define PIXMAN_BE_r8g8b8a8 PIXMAN_r8g8b8a8
#define PIXMAN_BE_x8b8g8r8 PIXMAN_x8b8g8r8
#define PIXMAN_BE_a8b8g8r8 PIXMAN_a8b8g8r8
#define PIXMAN_LE_r8g8b8 PIXMAN_b8g8r8
#define PIXMAN_LE_a8r8g8b8 PIXMAN_b8g8r8a8
#define PIXMAN_LE_x8r8g8b8 PIXMAN_b8g8r8x8
#else
#define PIXMAN_BE_r8g8b8 PIXMAN_b8g8r8
#define PIXMAN_BE_x8r8g8b8 PIXMAN_b8g8r8x8
#define PIXMAN_BE_a8r8g8b8 PIXMAN_b8g8r8a8
#define PIXMAN_BE_b8g8r8x8 PIXMAN_x8r8g8b8
#define PIXMAN_BE_b8g8r8a8 PIXMAN_a8r8g8b8
#define PIXMAN_BE_r8g8b8x8 PIXMAN_x8b8g8r8
#define PIXMAN_BE_r8g8b8a8 PIXMAN_a8b8g8r8
#define PIXMAN_BE_x8b8g8r8 PIXMAN_r8g8b8x8
#define PIXMAN_BE_a8b8g8r8 PIXMAN_r8g8b8a8
#define PIXMAN_LE_r8g8b8 PIXMAN_r8g8b8
#define PIXMAN_LE_a8r8g8b8 PIXMAN_a8r8g8b8
#define PIXMAN_LE_x8r8g8b8 PIXMAN_x8r8g8b8
#endif

#endif // VIRTIOGPU