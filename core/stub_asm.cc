#include <stdlib.h>

extern "C" {

void
SwitchContext(void *old_ctx, void *new_ctx)
{
    abort();
}

};
