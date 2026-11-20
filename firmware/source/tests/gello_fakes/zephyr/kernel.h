#pragma once
/* Host-only seams for exercising the actual standalone adapter. */
#include <stdint.h>
#include <stdio.h>
typedef int32_t atomic_t;
struct k_spinlock { int unused; };
typedef unsigned k_spinlock_key_t;
static inline int32_t atomic_get(const atomic_t *p) { return *p; }
static inline void atomic_set(atomic_t *p,int32_t v) { *p=v; }
static inline unsigned k_spin_lock(struct k_spinlock *p) { (void)p;return 0; }
static inline void k_spin_unlock(struct k_spinlock *p,unsigned key) { (void)p;(void)key; }
#define IS_ENABLED(x) (x)
#define BUILD_ASSERT(x) _Static_assert(x,#x)
#define ARG_UNUSED(x) (void)(x)
#define K_THREAD_DEFINE(...)
#define K_MSEC(x) (x)
#define printk printf
static inline void k_sleep(int value) { (void)value; }
