#ifndef __ASM_ARM_TYPES_H
#define __ASM_ARM_TYPES_H

#if defined(CONFIG_SYNO_ARMADA_ARCH)
#include <asm-generic/types.h>
#else
#include <asm-generic/int-ll64.h>

#ifndef __ASSEMBLY__

typedef unsigned short umode_t;

#endif /* __ASSEMBLY__ */

/*
 * These aren't exported outside the kernel to avoid name space clashes
 */
#endif

#ifdef __KERNEL__

#define BITS_PER_LONG 32

#endif /* __KERNEL__ */

#endif

