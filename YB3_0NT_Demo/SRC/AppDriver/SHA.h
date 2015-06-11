#ifndef  __SHA_H
#define  __SHA_H  

#include "config.h"

#ifdef __SHA_C
#define SHA_EXT
#else
#define SHA_EXT  extern
#endif

SHA_EXT	void	ComputeSHAEE(unsigned char *MT,long *A,long *B, long *C, long *D,long *E);
SHA_EXT	uint32	KTN(uint8 n);
SHA_EXT	uint32	NLF(uint32 B, uint32 C, uint32 D, uint8 n);

#endif
