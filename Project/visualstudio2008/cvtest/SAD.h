


extern "C"
{
int SAD16x16_C(void *pdata1, void *pdata2, int stride);		// assume same stride

int SAD16x16_ARM(void *pdata1, void *pdata2, int stride);		// assume same stride
#ifndef noSSE2
#define SAD16x16 SAD16x16_SSE2
int SAD16x16_SSE2(void *pdata1, void *pdata2, int stride);		// assume same stride
#else
#define SAD16x16 SAD16x16_ARM
#endif

}