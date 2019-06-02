#ifndef _LJ_MCODE_SPOOL_H
#define _LJ_MCODE_SPOOL_H

#include "lua.h"
#include "lj_def.h"
#include "lj_arch.h"

#if LJ_HASJIT && defined(__ANDROID__) && LJ_ANDROID_MCODE_STATIC_POOL_KB > 0

/* NOTE: logging to android logcat can be useful for understanding possible mcode allocation memory issues */
/* Allocations, and thus, log writes are rare*/
#ifndef LJ_ANDROID_LOG_LEVEL
#define LJ_ANDROID_LOG_LEVEL 1 /* 2 warns, 1 errors, 0 no logging */
#endif

LJ_FUNC void* lj_alloc_from_static_pool(unsigned int size, int prot);
LJ_FUNC int lj_release_to_static_pool(void* p, unsigned int size);


#endif

#endif
