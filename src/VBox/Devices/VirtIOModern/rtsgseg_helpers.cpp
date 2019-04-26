#include "rtsgseg_helpers.h"
#include <string.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

size_t RTSGSEG_to_buf(const RTSGSEG *seg, const unsigned int seg_count,
                      size_t offset, void *buf, size_t bytes) {
  size_t done;
  unsigned int i;
  uint8_t *buf_ptr = static_cast<uint8_t *>(buf);
  for (i = 0, done = 0; (offset || done < bytes) && i < seg_count; i++) {
    if (offset < seg[i].cbSeg) {
      const uint8_t *seg_ptr = static_cast<const uint8_t *>(seg[i].pvSeg);
      size_t len = MIN(seg[i].cbSeg - offset, bytes - done);
      memcpy(buf_ptr + done, seg_ptr + offset, len);
      done += len;
      offset = 0;
    } else {
      offset -= seg[i].cbSeg;
    }
  }
  return done;
}

size_t RTSGSEG_from_buf(const RTSGSEG *seg, unsigned int seg_count,
                        size_t offset, const void *buf, size_t bytes) {
  size_t done;
  unsigned int i;
  const uint8_t *buf_ptr = static_cast<const uint8_t *>(buf);
  for (i = 0, done = 0; (offset || done < bytes) && i < seg_count; i++) {
    if (offset < seg[i].cbSeg) {
      uint8_t *seg_ptr = static_cast<uint8_t *>(seg[i].pvSeg);
      size_t len = MIN(seg[i].cbSeg - offset, bytes - done);
      memcpy(seg_ptr + offset, buf_ptr + done, len);
      done += len;
      offset = 0;
    } else {
      offset -= seg[i].cbSeg;
    }
  }
  return done;
}