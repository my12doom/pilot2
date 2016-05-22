#include "SAD.h"
#include <stdlib.h>

#ifndef noSSE2
#include <emmintrin.h>
int SAD16x16_SSE2(void *pdata1, void *pdata2, int stride)		// assume same stride
{
	char *p1 = (char*)pdata1;
	char *p2 = (char*)pdata2;

	__m128i a,b;
	__m128i t2;

	t2 = _mm_setzero_si128();

	for(int i=0; i<16; i++)
	{
		a = _mm_loadu_si128((__m128i*)p1);
		b = _mm_loadu_si128((__m128i*)p2);

		a = _mm_sad_epu8(a, b);
		t2 = _mm_add_epi64(t2, a);

		p1 += stride;
		p2 += stride;
	}

	__int64 *p = (__int64*) &t2;
	return (int)(p[0] + p[1]);
}
#else
#endif

int SAD16x16_C(void *pdata1, void *pdata2, int stride)		// assume same stride
{
	unsigned char *p1 = (unsigned char*)pdata1;
	unsigned char *p2 = (unsigned char*)pdata2;

	int o = 0;

	for(int i=0; i<16; i++)
	{
		for(int x=0; x<16; x++)
			o += abs(p1[x]-p2[x]);


		p1 += stride;
		p2 += stride;
	}

	return o;
}
