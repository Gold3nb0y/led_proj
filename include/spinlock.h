#ifndef _SPINLOCK_H
#define _SPINLOCK_H

#include <stdatomic.h>
#include <stdbool.h>

typedef struct _spinlock{
    atomic_bool in_use;
} spinlock_t;

bool lock(spinlock_t *l);
void unlock(spinlock_t *l);

#endif
