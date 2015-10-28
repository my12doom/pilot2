#pragma once
#include <assert.h>
#ifdef WIN32
#include <Windows.h>
static uint32_t __USAD8(uint32_t op1, uint32_t op2)
{
	uint8_t *d1 = (uint8_t*)&op1;
	uint8_t *d2 = (uint8_t*)&op2;	

	return abs((int)d1[0]-d2[0])+abs((int)d1[0]-d2[0])+abs((int)d1[0]-d2[0])+abs((int)d1[0]-d2[0]);
}

static uint32_t __UADD8(uint32_t op1, uint32_t op2)
{
	uint32_t o;
	uint8_t *d1 = (uint8_t*)&op1;
	uint8_t *d2 = (uint8_t*)&op2;	
	uint8_t *po = (uint8_t*)&o;

	po[0] = d1[0] + d1[0];
	po[1] = d1[1] + d1[1];
	po[2] = d1[2] + d1[2];
	po[3] = d1[3] + d1[3];

	return o;
}

static uint32_t __UHADD8(uint32_t op1, uint32_t op2)
{
	uint32_t o;
	uint8_t *d1 = (uint8_t*)&op1;
	uint8_t *d2 = (uint8_t*)&op2;	
	uint8_t *po = (uint8_t*)&o;

	po[0] = (d1[0] + d1[0])>>1;
	po[1] = (d1[1] + d1[1])>>1;
	po[2] = (d1[2] + d1[2])>>1;
	po[3] = (d1[3] + d1[3])>>1;

	return o;
}

static uint32_t __USADA8(uint32_t op1, uint32_t op2, uint32_t op3)
{
	uint8_t *d1 = (uint8_t*)&op1;
	uint8_t *d2 = (uint8_t*)&op2;	

	return op3+abs((int)d1[0]-d2[0])+abs((int)d1[0]-d2[0])+abs((int)d1[0]-d2[0])+abs((int)d1[0]-d2[0]);
}
#endif