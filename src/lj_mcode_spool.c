/*
Static Memory Pool in .bss section.
This solves very dense memory mapping around loaded luajit library on Android, and the resulting severe performance degradation
due to constant lj_mcode.c allocation retries.
This is more pronounced on ARM due to shorter +/- 32mb near jump range (luajit futher limits it due to various constraints
to +/- 14mb around lj_vm_exit_handler symbol.

.bss section is allocated by kernel when our ELF is loaded, thus, we will get as close (to luajit code) address for that pool as possible.
The statically allocated pool can be quite large, system doesn't commit physical pages until we touch it.
We also release luajit released blocks, via madvise().
All this results in guaranteed memory area, close to luajit code as required by ARM near jumps, and we still don't trash memory and release it gracefully.

USAGE:
- use LJ_ANDROID_MCODE_STATIC_POOL_KB as makefile argument, e.g. (to reserve 10mb):
  -DLJ_ANDROID_MCODE_STATIC_POOL_KB=10240

CAVEATS:
- the .bss static pool area is shared accross all lua states. Make LJ_ANDROID_MCODE_STATIC_POOL_KB rather large, to accommodate
possible multiple lua state instances running. Note that this memory is not wasted, just address space reserved
- future android versions can break this by allocating bss somewhere far from loaded .so

POSSIBLE ALTERNATIVE SOLUTIONS:
- use long jumps for genetated code to call into luajit code and back,
  and reserve one larger memory pool for all generated code (up to 32mb, if relative jumps still used in genated code)
- use long jumps and near jumps as needed in generated code to avoid any dependency on allocated mcode memory addresses

TODO:
- check if this is needed/required on x86/mips/AARCH64 Android Targets
*/

#include "lj_mcode_spool.h"

#if LJ_HASJIT && defined(__ANDROID__) && LJ_ANDROID_MCODE_STATIC_POOL_KB > 0

#include <sys/mman.h>
#include <errno.h>

#if !defined(__GNUC__)
#error GCC/Clang required!
#endif

#if LJ_ANDROID_LOG_LEVEL > 0
#include <android/log.h>
#define LOG_TAG "lj_mcode_spool.c"
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#else
#define LOGE(...)
#endif
#if LJ_ANDROID_LOG_LEVEL >= 2
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#else
#define LOGW(...)
#endif
#define TOSTRING(s) #s
#define STRINGIFY(s) TOSTRING(s)
#define assert0(cond) do { \
		if(!(cond)) { \
			__android_log_assert(STRINGIFY(cond), LOG_TAG, "ASSERT: %s in %s() at %d\n", STRINGIFY(cond), __func__, __LINE__); \
		} \
} while (0) \

#include <pthread.h>

#define STATIC_POOL_SIZE                  (LJ_ANDROID_MCODE_STATIC_POOL_KB * 1024)
/* As 64kb alignment is required, we have 64k blocks. If <64kb is requested, this means some pool space is wasted, also remainder wasted for non-64kb divisible block sizes*/
#define STATIC_POOL_MIN_BLOCK_SIZE_KB     64
#define STATIC_POOL_MIN_BLOCK_SIZE        (STATIC_POOL_MIN_BLOCK_SIZE_KB * 1024)
#define STATIC_POOL_BLOCK_SIZE_MASK       (STATIC_POOL_MIN_BLOCK_SIZE - 1)

_Static_assert(LJ_ANDROID_MCODE_STATIC_POOL_KB >= STATIC_POOL_MIN_BLOCK_SIZE_KB, "");

/* This is global per-process pool and it requires locking. Luckily, calls to lj_alloc_from_static_pool are rare*/
static pthread_mutex_t _static_alloc_mutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER;

/* Extra STATIC_POOL_MIN_BLOCK_SIZE for 64kb alignment, which is required for some reason by luajit*/
static char _static_alloc_pool[STATIC_POOL_SIZE + STATIC_POOL_MIN_BLOCK_SIZE]
  __attribute__ ((aligned (4096))) __attribute__ ((section (".bss"))) __attribute__ ((used));

#define STATIC_POOL_MAX_ENTRIES (LJ_ANDROID_MCODE_STATIC_POOL_KB / STATIC_POOL_MIN_BLOCK_SIZE_KB)

static int _static_alloc_map[STATIC_POOL_MAX_ENTRIES]; /* NOTE: this can be a bitset, or flags, buf for now just booleans 1/0 */

void* lj_alloc_from_static_pool(unsigned int size, int prot) {
	int size_blocks = size / STATIC_POOL_MIN_BLOCK_SIZE;

	/* Ensure size is rounded up to next x*64kb */
	if((size & STATIC_POOL_BLOCK_SIZE_MASK) != 0) {
		LOGE("%s requested bad size=%d", __func__, size);
		size_blocks++;
	}
	if(size_blocks == 0) return NULL;
	void* p = NULL;
	size_t used = 0;

	pthread_mutex_lock(&_static_alloc_mutex);
	{
		int start, end, mark;
		for(start = 0; start <= STATIC_POOL_MAX_ENTRIES - size_blocks; start++) { // size_blocks always > 0, so we never hit bad index
			if(_static_alloc_map[start] == 0) {

				int all_ok = 1;

				for(end = start; end < start + size_blocks; end++) {
					if(_static_alloc_map[end] != 0) {
						all_ok = 0;
						break;
					}
				}
				if(all_ok) {
					void* aligned_pool_start = (void*)(((uintptr_t)_static_alloc_pool + STATIC_POOL_MIN_BLOCK_SIZE - 1) & ~(STATIC_POOL_MIN_BLOCK_SIZE - 1));
					assert0((uintptr_t)aligned_pool_start >= (uintptr_t)_static_alloc_pool && (uintptr_t)_static_alloc_pool <= (uintptr_t)_static_alloc_pool + sizeof(_static_alloc_pool) / sizeof(_static_alloc_pool[0]));

					/* We're OK, get block */
					p = aligned_pool_start + start * STATIC_POOL_MIN_BLOCK_SIZE;

					/* Try to mprotect or bail on failure */
					if(mprotect(p, size_blocks * STATIC_POOL_MIN_BLOCK_SIZE, prot) == 0) {
						/* Mark blocks as used */
						for(mark = start; mark < start + size_blocks; mark++) {
							_static_alloc_map[mark] = 1;
						}
					} else {
						LOGE("%s failed to mprotect=0x%p prot=0x%x start=%d size=%d size_blocks=%d errno=%d", __func__, p, prot, start, size, size_blocks, errno);
						p = NULL;
					}
					break;
				}
			}

		}
		int i;
		for(i = 0; i < STATIC_POOL_MAX_ENTRIES; i++) {
			if(_static_alloc_map[i]) used += STATIC_POOL_MIN_BLOCK_SIZE;
		}
	}
	pthread_mutex_unlock(&_static_alloc_mutex);


	if(p) {
		LOGW("%s OK p=0x%p size=%d used=%d avail=%d total=%d", __func__, p, size, used, STATIC_POOL_SIZE - used, STATIC_POOL_SIZE);
	}else {
		LOGE("%s no block for size=%d size_blocks=%d used=%d avail=%d total=%d", __func__, size, size_blocks, used, STATIC_POOL_SIZE - used, STATIC_POOL_SIZE);
	}

	return p;
}


int lj_release_to_static_pool(void* p, unsigned int size) {
	int size_blocks = size / STATIC_POOL_MIN_BLOCK_SIZE;

	/* Ensure size is rounded up to next x*64kb */
	if((size & STATIC_POOL_BLOCK_SIZE_MASK) != 0) {
		size_blocks++;
		LOGE("%s requested bad size=%d", __func__, size);
	}
	if(size_blocks == 0) {
		LOGE("%s bad block size=%d size_blocks=%d", __func__, size, size_blocks);
		return 1;
	}

	void* aligned_pool_start = (void*)(((uintptr_t)_static_alloc_pool + STATIC_POOL_MIN_BLOCK_SIZE - 1) & ~(STATIC_POOL_MIN_BLOCK_SIZE - 1));
	assert0((uintptr_t)aligned_pool_start >= (uintptr_t)_static_alloc_pool && (uintptr_t)_static_alloc_pool <= (uintptr_t)_static_alloc_pool + sizeof(_static_alloc_pool) / sizeof(_static_alloc_pool[0]));

	if(p < aligned_pool_start || p + size_blocks * STATIC_POOL_MIN_BLOCK_SIZE > aligned_pool_start + STATIC_POOL_SIZE) {
		LOGE("%s bad p=0x%p aligned_pool_start=0x%p size=0x%x", __func__, p, aligned_pool_start, STATIC_POOL_SIZE);
		return 1;
	}

	assert0(((uintptr_t)p & STATIC_POOL_BLOCK_SIZE_MASK) == 0); /* This should be 64k aligned address. We can't return failure as this address is certainly from pool */

	unsigned int index = (p - aligned_pool_start) / STATIC_POOL_MIN_BLOCK_SIZE;
	assert0(index <  STATIC_POOL_MAX_ENTRIES);
	assert0(index + size_blocks <= STATIC_POOL_MAX_ENTRIES);
	size_t used = 0;

	pthread_mutex_lock(&_static_alloc_mutex);
	{
		/* Mark blocks as free */
		int mark;
		for(mark = index; mark < index + size_blocks; mark++) {
			if(_static_alloc_map[mark] == 0 && index == mark) {
				/* Still, continue with the release - we can double release w/o problems, check just for first block */
				LOGE("%s already has block released ix=%d p=0x%p size=0x%x", __func__, index, p, size_blocks * STATIC_POOL_MIN_BLOCK_SIZE);
			}
			_static_alloc_map[mark] = 0;
		}

		if(madvise(p, size_blocks * STATIC_POOL_MIN_BLOCK_SIZE, MADV_DONTNEED) != 0) {
			LOGE("%s failed madvise for p=0x%p size=0x%x", __func__, p, size_blocks * STATIC_POOL_MIN_BLOCK_SIZE);
		}
		int i;
		for(i = 0; i < STATIC_POOL_MAX_ENTRIES; i++) {
			if(_static_alloc_map[i]) used += STATIC_POOL_MIN_BLOCK_SIZE;
		}
	}
	pthread_mutex_unlock(&_static_alloc_mutex);

	LOGW("%s OK size=%d used=%d avail=%d total=%d", __func__, size, used, STATIC_POOL_SIZE - used, STATIC_POOL_SIZE);

	return 0; /* OK */
}

#if 0
__attribute__((constructor))
static void static_alloc_pool_tests() {
	int total_blocks_avail = LJ_ANDROID_MCODE_STATIC_POOL_KB / STATIC_POOL_MIN_BLOCK_SIZE_KB;
	assert0(sizeof(_static_alloc_map) / sizeof(_static_alloc_map[0]) == total_blocks_avail);
	assert0(total_blocks_avail >= 4);

	LOGE("%s TESTS total_blocks_avail=%d", __func__, total_blocks_avail);


	void* block1 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC);
	assert0(block1);
	*(int*)block1 = 1; /* Touch */

	void* block2 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE * 2, PROT_READ|PROT_WRITE|PROT_EXEC);
	assert0(block2);
	*(int*)block2 = 2;

	void* blocks[total_blocks_avail];
	memset(blocks, 0, sizeof(void*) * total_blocks_avail);
	int i;
	for(i = 3; i < total_blocks_avail; i++) {
		blocks[i] = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC);
		assert0(blocks[i]);
		*(int*)blocks[i] = i;
	}

	void* block3 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC); /* Should fail */
	assert0(!block3);

	block3 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE * 2, PROT_READ|PROT_WRITE|PROT_EXEC); /* Should fail */
	assert0(!block3);

	assert0(lj_release_to_static_pool(block2, STATIC_POOL_MIN_BLOCK_SIZE * 2) == 0);

	block3 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC);
	assert0(block3);
	void* block4 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC);
	assert0(block4);

	/* Free */
	assert0(lj_release_to_static_pool(block1, STATIC_POOL_MIN_BLOCK_SIZE) == 0);
	assert0(lj_release_to_static_pool(block4, STATIC_POOL_MIN_BLOCK_SIZE) == 0);
	assert0(lj_release_to_static_pool(block3, STATIC_POOL_MIN_BLOCK_SIZE) == 0);

	/* Free others from end */
	for(i = total_blocks_avail - 1; i >= 3; i--) {
		assert0(blocks[i]);
		assert0(*(int*)blocks[i] == i);
		assert0(lj_release_to_static_pool(blocks[i], STATIC_POOL_MIN_BLOCK_SIZE) == 0);
	}

	/* Alloc all again */
	for(i = 0; i < total_blocks_avail; i++) {
		blocks[i] = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC);
		assert0(blocks[i]);
		*(int*)blocks[i] = i * 10;
	}
	assert0(!lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC)); // Should fail

	/* Release all */
	for(i = 0; i < total_blocks_avail; i++) {
		assert0(blocks[i]);
		assert0(*(int*)blocks[i] == i * 10);
		assert0(lj_release_to_static_pool(blocks[i], STATIC_POOL_MIN_BLOCK_SIZE) == 0);
	}

	/* Check we have mapping OK */
	for(i = 0; i < total_blocks_avail; i++) {
		assert0(_static_alloc_map[i] == 0);
	}

	assert0(lj_release_to_static_pool(block1, STATIC_POOL_MIN_BLOCK_SIZE) == 0); /* Won't fail, double release is OK */
	assert0(lj_release_to_static_pool((void*)123, STATIC_POOL_MIN_BLOCK_SIZE) != 0); /* Fail, bad addr */
	assert0(lj_release_to_static_pool(_static_alloc_pool + STATIC_POOL_SIZE + 1, STATIC_POOL_MIN_BLOCK_SIZE) != 0); /* Fail, bad addr */


	block1 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE * total_blocks_avail, PROT_READ|PROT_WRITE|PROT_EXEC);
	assert0(block1);
	block2 = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC);
	assert0(!block2);
	assert0(lj_release_to_static_pool(block1, STATIC_POOL_MIN_BLOCK_SIZE * total_blocks_avail) == 0);
	assert0(lj_release_to_static_pool(block1, STATIC_POOL_MIN_BLOCK_SIZE * total_blocks_avail) == 0); /* Won't fail, double release is OK */

	/* Check we have mapping OK */
	for(i = 0; i < total_blocks_avail; i++) {
		assert0(_static_alloc_map[i] == 0);
	}

	/* Alloc all again, block / 2 */
	for(i = 0; i < total_blocks_avail; i++) {
		blocks[i] = lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE / 2, PROT_READ|PROT_WRITE|PROT_EXEC); /* Size should be expanded to block size */
		assert0(blocks[i]);
		*(int*)blocks[i] = i * 10;
	}
	assert0(!lj_alloc_from_static_pool(STATIC_POOL_MIN_BLOCK_SIZE, PROT_READ|PROT_WRITE|PROT_EXEC)); /* Should fail */

	/* Release all */
	for(i = 0; i < total_blocks_avail; i++) {
		assert0(blocks[i]);
		assert0(*(int*)blocks[i] == i * 10);
		assert0(lj_release_to_static_pool(blocks[i], STATIC_POOL_MIN_BLOCK_SIZE / 2) == 0);
	}

	/* Check we have mapping OK */
	for(i = 0; i < total_blocks_avail; i++) {
		assert0(_static_alloc_map[i] == 0);
	}

	LOGE("%s TESTS DONE", __func__);
}
#endif

#endif
