
#include <limits.h>
#include "lwesp/lwesp_private.h"
#include "lwesp/lwesp_mem.h"

#if LWESP_CFG_MEM_CUSTOM || __DOXYGEN__
void *lwesp_mem_malloc(size_t size) {
    return malloc(size);
}

void *lwesp_mem_realloc(void *ptr, size_t size) {
    return realloc(ptr, size);
}

void *lwesp_mem_calloc(size_t num, size_t size) {
    return calloc(num, size);
}

void lwesp_mem_free(void *ptr) {
    free(ptr);
}
#endif
