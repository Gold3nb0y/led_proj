#include "spinlock.h"

bool __inline lock(spinlock_t *l){
    if(l->in_use)
        return false;
    l->in_use = true;
    return true;
}

void __inline unlock(spinlock_t *l){
    l->in_use = false;
}
