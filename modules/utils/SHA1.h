#pragma  once 

#include <stdint.h>

typedef struct SHA1_STATETYPE_TAG
{   
	uint32_t wbuf[16];
	uint32_t hash[5];
	uint32_t count[2];
} SHA1_STATETYPE;
void SHA1Hash(unsigned char *_pOutDigest, const unsigned char *_pData, uint32_t nSize);
void SHA1_Start(SHA1_STATETYPE* _pcsha1);
void SHA1_Finish(unsigned char* _pShaValue, SHA1_STATETYPE* _pcsha1);
void SHA1_Hash(const unsigned char *_pData, unsigned int _iSize, SHA1_STATETYPE* _pcsha1);
