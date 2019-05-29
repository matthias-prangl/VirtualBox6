#include "virtioGPU.h"
#include <iprt/mem.h>

void virtioGPU_get_config(VirtioDevice *vdev, uint8_t *config) {
  VirtioGPU *vgpu = reinterpret_cast<VirtioGPU *>(vdev->pciDev);
  memcpy(config, &vgpu->virtio_config, sizeof(vgpu->virtio_config));
}

void virtioGPU_set_config(VirtioDevice *vdev, uint8_t *config) {
  VirtioGPU *vgpu = reinterpret_cast<VirtioGPU *>(vdev->pciDev);
  struct virtio_gpu_config vgpuconfig;

  memcpy(&vgpuconfig, config, sizeof(vgpu->virtio_config));

  if (vgpuconfig.events_clear) {
    vgpu->virtio_config.events_read &= ~vgpuconfig.events_clear;
  }
}

void virtioGPU_set_features(VirtioDevice *vdev, uint64_t features) {
  static const uint32_t virgl = (1 << VIRTIO_GPU_F_VIRGL);
  VirtioGPU *vgpu = reinterpret_cast<VirtioGPU *>(vdev->pciDev);
  vgpu->use_virgl_renderer = ((features & virgl) == virgl);
}

uint64_t virtioGPU_get_features(VirtioDevice *vdev, uint64_t features) {
  RT_NOREF(vdev, features);
  return features;
}

void virtioGPU_reset(VirtioDevice *vdev) {
  VirtioGPU *vgpu = reinterpret_cast<VirtioGPU *>(vdev->pciDev);
  RT_NOREF(vgpu);
}

static void
virtio_gpu_fill_display_info(VirtioGPU *vgpu,
                             struct virtio_gpu_resp_display_info *dpy_info) {
  for (unsigned int i = 0; i < vgpu->conf.max_outputs; i++) {
    if (vgpu->enabled_output_bitmask & (1 << i)) {
      dpy_info->pmodes[i].enabled = 1;
      dpy_info->pmodes[i].r.width = RT_H2LE_U32(vgpu->req_state[i].width);
      dpy_info->pmodes[i].r.height = RT_H2LE_U32(vgpu->req_state[i].height);
    }
  }
}

void virtio_gpu_ctrl_response(VirtioGPU *vgpu,
                              struct virtio_gpu_ctrl_command *cmd,
                              struct virtio_gpu_ctrl_hdr *resp,
                              size_t resp_len) {
  if (cmd->cmd_hdr.flags & VIRTIO_GPU_FLAG_FENCE) {
    resp->flags |= VIRTIO_GPU_FLAG_FENCE;
    resp->fence_id = cmd->cmd_hdr.fence_id;
    resp->ctx_id = cmd->cmd_hdr.ctx_id;
  }

  unsigned int s = static_cast<unsigned int>(
      RTSGSEG_from_buf(cmd->elem.in_sg, cmd->elem.in_num, 0, resp, resp_len));
  if (s != resp_len) {
    // log useful stuff
  }
  virtqueue_push(cmd->vq, &cmd->elem, s);
  virtio_notify(&vgpu->vdev, cmd->vq);
  cmd->finished = true;
}

void virtio_gpu_ctrl_response_nodata(VirtioGPU *vgpu,
                                     struct virtio_gpu_ctrl_command *cmd,
                                     enum virtio_gpu_ctrl_type type)
{
    struct virtio_gpu_ctrl_hdr resp;

    memset(&resp, 0, sizeof(resp));
    resp.type = type;
    virtio_gpu_ctrl_response(vgpu, cmd, &resp, sizeof(resp));
}

void virtio_gpu_get_display_info(VirtioGPU *vgpu,
                                 struct virtio_gpu_ctrl_command *cmd) {
  struct virtio_gpu_resp_display_info display_info;

  memset(&display_info, 0, sizeof(display_info));
  display_info.hdr.type = VIRTIO_GPU_RESP_OK_DISPLAY_INFO;
  virtio_gpu_fill_display_info(vgpu, &display_info);
  virtio_gpu_ctrl_response(vgpu, cmd, &display_info.hdr, sizeof(display_info));
}

static struct virtio_gpu_simple_resource *
virtio_gpu_find_resource(VirtioGPU *vgpu, uint32_t resource_id) {
  for (auto it = vgpu->reslist.begin(); it != vgpu->reslist.end(); it++) {
    if ((*it)->resource_id == resource_id) {
      return *it;
    }
  }
  return NULL;
}

static pixman_format_code_t get_pixman_format(uint32_t virtio_gpu_format) {
  switch (virtio_gpu_format) {
  case VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM:
    return PIXMAN_BE_b8g8r8x8;
  case VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM:
    return PIXMAN_BE_b8g8r8a8;
  case VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM:
    return PIXMAN_BE_x8r8g8b8;
  case VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM:
    return PIXMAN_BE_a8r8g8b8;
  case VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM:
    return PIXMAN_BE_r8g8b8x8;
  case VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM:
    return PIXMAN_BE_r8g8b8a8;
  case VIRTIO_GPU_FORMAT_X8B8G8R8_UNORM:
    return PIXMAN_BE_x8b8g8r8;
  case VIRTIO_GPU_FORMAT_A8B8G8R8_UNORM:
    return PIXMAN_BE_a8b8g8r8;
  default:
    return static_cast<pixman_format_code_t>(0);
  }
}

static uint32_t calc_image_hostmem(pixman_format_code_t pformat, uint32_t width,
                                   uint32_t height) {
  /* Copied from pixman/pixman-bits-image.c, skip integer overflow check.
   * pixman_image_create_bits will fail in case it overflow.
   */

  int bpp = PIXMAN_FORMAT_BPP(pformat);
  int stride = ((width * bpp + 0x1f) >> 5) * sizeof(uint32_t);
  return height * stride;
}

void virtio_gpu_resource_create_2d(VirtioGPU *vgpu,
                                   struct virtio_gpu_ctrl_command *cmd) {
  struct virtio_gpu_resource_create_2d c2d;
  struct virtio_gpu_simple_resource *res;
  VIRTIO_GPU_FILL_CMD(c2d);

  if (c2d.resource_id == 0) {
    cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
    // id 0 not allowed
    return;
  }
  res = virtio_gpu_find_resource(vgpu, c2d.resource_id);
  if (res) {
    cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
    // already exists
    return;
  }
  res = static_cast<struct virtio_gpu_simple_resource *>(
      calloc(1, sizeof(struct virtio_gpu_simple_resource)));
  res->width = c2d.width;
  res->height = c2d.height;
  res->format = c2d.format;
  res->resource_id = c2d.resource_id;

  pixman_format_code_t pformat = get_pixman_format(c2d.format);
  if (!pformat) {
    // couldn't handle guest format
    free(res);
    cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
    return;
  }

  res->hostmem = calc_image_hostmem(pformat, c2d.width, c2d.height);
  if (res->hostmem + vgpu->hostmem < vgpu->conf.max_hostmem) {
    res->image =
        pixman_image_create_bits(pformat, c2d.width, c2d.height, NULL, 0);
  }
  if (!res->image) {
    free(res);
    cmd->error = VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY;
    return;
  }
  vgpu->reslist.push_back(res);
  vgpu->hostmem += res->hostmem;
}

static void virtio_gpu_unref_resource(pixman_image_t *image, void *data) {
  pixman_image_unref((pixman_image_t *)data);
}

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(n, d) (((n) + (d)-1) / (d))
#endif

static void virtio_gpu_set_scanout(VirtioGPU *vgpu,
                                   struct virtio_gpu_ctrl_command *cmd) {
  struct virtio_gpu_simple_resource *res, *ores;
  struct virtio_gpu_scanout *scanout;
  pixman_format_code_t format;
  uint32_t offset;
  int bpp;
  struct virtio_gpu_set_scanout ss;

  VIRTIO_GPU_FILL_CMD(ss);

  if (ss.scanout_id >= vgpu->conf.max_outputs) {
    // illegal scanout id specified
    cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID;
    return;
  }

  vgpu->enable = 1;
  if (ss.resource_id == 0) {
    // virtio_gpu_disable_scanout(g, ss.scanout_id);
    return;
  }

  /* create a surface for this scanout */
  res = virtio_gpu_find_resource(vgpu, ss.resource_id);
  if (!res) {
    // illegal resource specified
    cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
    return;
  }

  if (ss.r.x > res->width || ss.r.y > res->height || ss.r.width > res->width ||
      ss.r.height > res->height || ss.r.x + ss.r.width > res->width ||
      ss.r.y + ss.r.height > res->height) {
    //"%s: illegal scanout %d bounds for"

    cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
    return;
  }

  scanout = &vgpu->scanout[ss.scanout_id];

  format = pixman_image_get_format(res->image);
  bpp = DIV_ROUND_UP(PIXMAN_FORMAT_BPP(format), 8);
  offset = (ss.r.x * bpp) + ss.r.y * pixman_image_get_stride(res->image);
  if ( //! scanout->ds ||
      /*surface_data(scanout->ds) != ((uint8_t
       *)pixman_image_get_data(res->image) + offset)*/
      scanout->width != ss.r.width || scanout->height != ss.r.height) {
    pixman_image_t *rect;
    uint32_t *ptr =
        (uint32_t *)(((uint8_t *)pixman_image_get_data(res->image)) + offset);
    rect = pixman_image_create_bits(format, ss.r.width, ss.r.height, ptr,
                                    pixman_image_get_stride(res->image));
    pixman_image_ref(res->image);
    pixman_image_set_destroy_function(rect, virtio_gpu_unref_resource,
                                      res->image);
    scanout->ds =
        (DisplaySurface *)calloc(1, sizeof(struct virtio_gpu_scanout));
    scanout->ds->format = pixman_image_get_format(rect);
    scanout->ds->image = pixman_image_ref(rect);
    /* realloc the surface ptr */
    // scanout->ds = qemu_create_displaysurface_pixman(rect);
    // if (!scanout->ds) {
    //   cmd->error = VIRTIO_GPU_RESP_ERR_UNSPEC;
    //   return;
    // }
    pixman_image_unref(rect);
    // dpy_gfx_replace_surface(g->scanout[ss.scanout_id].con, scanout->ds);
  }

  ores = virtio_gpu_find_resource(vgpu, scanout->resource_id);
  if (ores) {
    ores->scanout_bitmask &= ~(1 << ss.scanout_id);
  }

  res->scanout_bitmask |= (1 << ss.scanout_id);
  scanout->resource_id = ss.resource_id;
  scanout->x = ss.r.x;
  scanout->y = ss.r.y;
  scanout->width = ss.r.width;
  scanout->height = ss.r.height;
}

void virtio_gpu_cleanup_mapping_iov(VirtioGPU *vgpu, struct RTSGSEG *iov,
                                    uint32_t count, PGMPAGEMAPLOCK *locks) {
  for (uint32_t i = 0; i < count; i++) {
    PDMDevHlpPhysReleasePageMappingLock(vgpu->vdev.pciDev->pDevInsR3,
                                        &locks[i]);
  }
  free(iov);
}

int virtio_gpu_create_mapping_iov(VirtioGPU *vgpu,
                                  struct virtio_gpu_resource_attach_backing *ab,
                                  struct virtio_gpu_ctrl_command *cmd,
                                  uint64_t **addr, struct RTSGSEG **iov,
                                  PGMPAGEMAPLOCK **locks) {
  struct virtio_gpu_mem_entry *ents;
  size_t esize, s;

  if (ab->nr_entries > 16384) {
    // nr_entries is too big
    return -1;
  }

  esize = sizeof(*ents) * ab->nr_entries;
  ents = reinterpret_cast<virtio_gpu_mem_entry *>(calloc(1, esize));
  s = RTSGSEG_to_buf(cmd->elem.out_sg, cmd->elem.out_num, sizeof(*ab), ents,
                     esize);
  if (s != esize) {
    // command data size incorrect
    free(ents);
    return -1;
  }

  *iov = (RTSGSEG *)calloc(ab->nr_entries, sizeof(struct RTSGSEG));
  if (addr) {
    *addr = (uint64_t *)calloc(ab->nr_entries, sizeof(uint64_t));
  }

  *locks = (PGMPAGEMAPLOCK *)calloc(ab->nr_entries, sizeof(PGMPAGEMAPLOCK));

  for (uint32_t i = 0; i < ab->nr_entries; i++) {
    uint64_t a = RT_LE2H_U64(ents[i].addr);
    uint32_t l = RT_LE2H_U32(ents[i].length);
    uint64_t len = l;
    (*iov)[i].cbSeg = l;

    PDMDevHlpPhysGCPhys2CCPtr(vgpu->vdev.pciDev->pDevInsR3, a, 0,
                                        &((*iov)[i].pvSeg), &(*locks)[i]);
    if (addr) {
      (*addr)[i] = a;
    }
    
    if (!(*iov)[i].pvSeg || len != l) {
      // failed to map MMIO memory for element
      virtio_gpu_cleanup_mapping_iov(vgpu, *iov, i, *locks);
      free(ents);
      *iov = NULL;
      if (addr) {
        free(*addr);
        *addr = NULL;
      }
      return -1;
    }
  }
  free(ents);
  return 0;
}

static void
virtio_gpu_resource_attach_backing(VirtioGPU *vgpu,
                                   struct virtio_gpu_ctrl_command *cmd) {
  struct virtio_gpu_simple_resource *res;
  struct virtio_gpu_resource_attach_backing ab;
  int ret;

  VIRTIO_GPU_FILL_CMD(ab);

  res = virtio_gpu_find_resource(vgpu, ab.resource_id);
  if (!res) {
    // illegal resource specified
    cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
    return;
  }

  if (res->iov) {
    cmd->error = VIRTIO_GPU_RESP_ERR_UNSPEC;
    return;
  }

  ret = virtio_gpu_create_mapping_iov(vgpu, &ab, cmd, &res->addrs, &res->iov);
  if (ret != 0) {
    cmd->error = VIRTIO_GPU_RESP_ERR_UNSPEC;
    return;
  }

  res->iov_cnt = ab.nr_entries;
}

static void virtio_gpu_resource_destroy(VirtioGPU *vgpu,
                                        struct virtio_gpu_simple_resource *res)
{
    if (res->scanout_bitmask) {
        for (uint32_t i = 0; i < vgpu->conf.max_outputs; i++) {
            if (res->scanout_bitmask & (1 << i)) {
                // virtio_gpu_disable_scanout(g, i);
            }
        }
    }

    pixman_image_unref(res->image);
    // virtio_gpu_cleanup_mapping(g, res);
    // QTAILQ_REMOVE(&g->reslist, res, next);
    vgpu->hostmem -= res->hostmem;
    free(res);
}

static void virtio_gpu_resource_unref(VirtioGPU *vgpu,
                                      struct virtio_gpu_ctrl_command *cmd)
{
    struct virtio_gpu_simple_resource *res;
    struct virtio_gpu_resource_unref unref;

    VIRTIO_GPU_FILL_CMD(unref);

    res = virtio_gpu_find_resource(vgpu, unref.resource_id);
    if (!res) {
        cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
        return;
    }
    virtio_gpu_resource_destroy(vgpu, res);
}

static void virtio_gpu_cleanup_mapping(VirtioGPU *vgpu,
                                       struct virtio_gpu_simple_resource *res)
{
    virtio_gpu_cleanup_mapping_iov(vgpu, res->iov, res->iov_cnt, res->locks);
    res->iov = NULL;
    res->iov_cnt = 0;
    free(res->addrs);
    free(res->locks);
    res->locks = NULL;
    res->addrs = NULL;
}


static void
virtio_gpu_resource_detach_backing(VirtioGPU *vgpu,
                                   struct virtio_gpu_ctrl_command *cmd)
{
    struct virtio_gpu_simple_resource *res;
    struct virtio_gpu_resource_detach_backing detach;

    VIRTIO_GPU_FILL_CMD(detach);

    res = virtio_gpu_find_resource(vgpu, detach.resource_id);
    if (!res || !res->iov) {
        cmd->error = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
        return;
    }
    virtio_gpu_cleanup_mapping(vgpu, res);
}

void virtioGPU_handle_ctrl(VirtioDevice *vdev, VirtQueue *vq) {
  VirtioGPU *vgpu = reinterpret_cast<VirtioGPU *>(vdev->pciDev);

  if (vq->vring.avail == 0) {
    return;
  }

  struct virtio_gpu_ctrl_command *cmd =
      reinterpret_cast<struct virtio_gpu_ctrl_command *>(
          virtqueue_pop(vq, sizeof(struct virtio_gpu_ctrl_command)));
  cmd->vq = vq;
  cmd->error = 0;
  cmd->finished = false;
  VIRTIO_GPU_FILL_CMD(cmd->cmd_hdr);

  switch (cmd->cmd_hdr.type) {
  case VIRTIO_GPU_CMD_GET_DISPLAY_INFO:
    virtio_gpu_get_display_info(vgpu, cmd);
    break;
  case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D:
    virtio_gpu_resource_create_2d(vgpu, cmd);
    break;
  case VIRTIO_GPU_CMD_RESOURCE_UNREF:
    virtio_gpu_resource_unref(vgpu, cmd);
    break;
  case VIRTIO_GPU_CMD_RESOURCE_FLUSH:
    // virtio_gpu_resource_flush(vgpu, cmd);
    break;
  case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D:
    // virtio_gpu_transfer_to_host_2d(vgpu, cmd);
    break;
  case VIRTIO_GPU_CMD_SET_SCANOUT:
    virtio_gpu_set_scanout(vgpu, cmd);
    break;
  case VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING:
    virtio_gpu_resource_attach_backing(vgpu, cmd);
    break;
  case VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING:
    virtio_gpu_resource_detach_backing(vgpu, cmd);
    break;
  default:
    cmd->error = VIRTIO_GPU_RESP_ERR_UNSPEC;
    break;
  }
  if(!cmd->finished) {
    enum virtio_gpu_ctrl_type type = VIRTIO_GPU_RESP_OK_NODATA;
    if(cmd->error) {
      type = (enum virtio_gpu_ctrl_type) cmd->error;
    }
      
    virtio_gpu_ctrl_response_nodata(vgpu, cmd, type);
  }
  free(cmd);
}

void virtioGPU_handle_cursor(VirtioDevice *vdev, VirtQueue *vq) {
  VirtQueueElement *vqe = reinterpret_cast<VirtQueueElement *>(
      virtqueue_pop(vq, sizeof(VirtQueueElement)));
  virtqueue_push(vq, vqe, 0);
  virtio_notify(vdev, vq);
  free(vqe);
}

void virtioGPU_default_config(struct virtio_gpu_conf *conf) {
  conf->flags = 0;
  conf->max_hostmem = 256 * 1024 * 1024;
  conf->max_outputs = 1;
  conf->xres = 1024;
  conf->yres = 768;
}

int virtioGPUConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg) {
  PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);
  VirtioGPU *vgpu = PDMINS_2_DATA(pDevIns, VirtioGPU *);
  VirtioPCIDevice *pciDev = &vgpu->pciDev;
  VirtioDevice *vdev = &vgpu->vdev;
  vdev->pciDev = pciDev;
  pciDev->vdev = vdev;

  virtioGPU_default_config(&vgpu->conf);

  int rc =
      PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
  AssertRCReturn(rc, rc);
  virtioPCIConstruct(pDevIns, pciDev, iInstance, VIRTIOGPU_NAME_FMT,
                     VIRTIOGPU_ID, VIRTIOGPU_PCI_CLASS, VIRTIOGPU_N_QUEUES);

  vdev->virtio_notify_bus = virtioPCINotify;
  vdev->set_config = virtioGPU_set_config;
  vdev->get_config = virtioGPU_get_config;
  vdev->get_features = virtioGPU_get_features;
  vdev->set_features = virtioGPU_set_features;
  vdev->reset = virtioGPU_reset;
  vdev->config_len = sizeof(struct virtio_gpu_config);
  vdev->config = calloc(1, sizeof(struct virtio_gpu_config));
  if (!vdev->config) {
    rc = VERR_NO_MEMORY;
  }

  vgpu->virtio_config.num_scanouts = RT_H2LE_U32(vgpu->conf.max_outputs);
  vgpu->virtio_config.num_capsets = 0;
  vgpu->req_state[0].width = vgpu->conf.xres;
  vgpu->req_state[0].height = vgpu->conf.yres;
  for(int i = 0; i < VIRTIO_QUEUE_MAX; i++) {
    vdev->vq[i].vdev = vdev;
    vdev->vq[i].queue_idx = i;
  }

  vgpu->ctrl_vq = virtio_add_queue(vdev, 64, virtioGPU_handle_ctrl);
  vgpu->cursor_vq = virtio_add_queue(vdev, 16, virtioGPU_handle_cursor);

  vgpu->enabled_output_bitmask = 1;

  RT_NOREF(pCfg);
  return rc;
}

extern "C" DECLEXPORT(int)
    VBoxDevicesRegister(PPDMDEVREGCB pCallbacks, uint32_t u32Version) {
  RT_NOREF(u32Version);
  return pCallbacks->pfnRegister(pCallbacks, &g_virtioGPU);
}