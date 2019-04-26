#include <iprt/sg.h>

size_t RTSGSEG_to_buf(const RTSGSEG *seg, const unsigned int seg_count,
                      size_t offset, void *buf, size_t bytes);

size_t RTSGSEG_from_buf(const RTSGSEG *seg, unsigned int seg_count,
                        size_t offset, const void *buf, size_t bytes);
