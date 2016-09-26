/*****************************
 * 
 *
 * Multiplication and Arithmetic on Galois Field GF(256)
 *
 * From Mee, Daniel, "Magnetic Recording, Volume III", Ch. 5 by Patel.
 * 
 * (c) 1991 Henry Minsky
 *
 *
 ******************************/

#include <stdlib.h>
#include "GFMath.h"

/* This is one of 14 irreducible polynomials
 * of degree 8 and cycle length 255. (Ch 5, pp. 275, Magnetic Recording)
 * The high order 1 bit is implicit */
/* x^8 + x^4 + x^3 + x^2 + 1 */

unsigned char gexp[512];
unsigned char glog[256];
unsigned char mult_table[256][256];
unsigned char EKR255_table[256][256];
unsigned char genpoly_cache[MAX_NPAR][256];
unsigned char genpoly_23_mult_cache[256][32];

void init_genpoly_cache(int par);

void init_exp_table (void)
{
	static bool inited = false;
	if (inited ) return;
	inited = true;

	int i, z;
	int pinit,p1,p2,p3,p4,p5,p6,p7,p8;

	pinit = p2 = p3 = p4 = p5 = p6 = p7 = p8 = 0;
	p1 = 1;

	gexp[0] = 1;
	gexp[255] = gexp[0];
	glog[0] = 0;			// shouldn't log[0] be an error?

	for (i = 1; i < 256; i++)
	{
			pinit = p8;
			p8 = p7;
			p7 = p6;
			p6 = p5;
			p5 = p4 ^ pinit;
			p4 = p3 ^ pinit;
			p3 = p2 ^ pinit;
			p2 = p1;
			p1 = pinit;
			gexp[i] = p1 + p2*2 + p3*4 + p4*8 + p5*16 + p6*32 + p7*64 + p8*128;
			gexp[i+255] = gexp[i];
	}

	// glog init
	for (i = 1; i < 256; i++) 
	{
		for (z = 0; z < 256; z++) 
		{
			if (gexp[z] == i) 
			{
				glog[i] = z;
				break;
			}
		}
	}

	// mult table init
	for (int a=0; a<256; a++)
	{
		for (int b=0; b<256; b++)
		{
			mult_table[a][b] = c_gmult(a,b);
		}
	}

	// EKR255 table init
	for (int a=0; a<256; a++)
	{
		for (int b=0; b<256; b++)
		{
			EKR255_table[a][b] = gexp[(a*b)%255];
		}
	}

	for (int i=0; i<MAX_NPAR; i++)
		init_genpoly_cache(i+1);
}

// ********** polynomial arithmetic *******************

void add_polys (unsigned char *dst, unsigned char *src, int size) 
{
	int i;
	for (i = 0; i < size; i++) dst[i] ^= src[i];
}

void copy_poly (unsigned char *dst, unsigned char *src, int size) 
{
	memcpy(dst, src, size);
}

void scale_poly (int k, unsigned char *poly, int size) 
{	
	int i;
	for (i = 0; i < size; i++) poly[i] = gmult(k, poly[i]);
}


void zero_poly (unsigned char *poly, int size) 
{
	memset(poly, 0, size);
}


// multiply by z, i.e., shift right by 1
void c_mul_z_poly (unsigned char *src, int size)
{
	memmove(src+1, src, size-1);
	src[0] = 0;
}

/* polynomial multiplication */
void mult_polys (unsigned char *dst, unsigned char *p1, unsigned char *p2, int size)
{
#if 1
	int i, j;
	memset(dst, 0, size*2);

	for (i = 0; i < size; i++) 
	{
		for(j=0; j<size; j++) 
			dst[j+i]^=gmult(p1[i], p2[j]);
	 }
#else
	
	int i, j;
	int tmp1[256*2*2];

	for (i=0; i < (size*2); i++) 
		dst[i] = 0;

	for (i = 0; i < size; i++) 
	{
			for(j=size; j<(size*2); j++) tmp1[j]=0;
				
			/* scale tmp1 by p1[i] */
			for(j=0; j<size; j++) 
				tmp1[j]=gmult(p2[j], p1[i]);
			/* and mult (shift) tmp1 right by i */
			for (j = (size*2)-1; j >= i; j--) 
				tmp1[j] = tmp1[j-i];
			for (j = 0; j < i; j++) 
				tmp1[j] = 0;

			/* add into partial product */
			for(j=0; j < (size*2); j++) 
				dst[j] ^= tmp1[j];
	 }

#endif
}

// Create a generator polynomial for an n byte RS code. 
void init_genpoly_cache(int par)
{
	int i;
	unsigned char tp[256], tp1[256];
	int tmp;

	unsigned char *genpoly = genpoly_cache[par-1];

	//multiply (x + a^n) for n = 1 to nbytes

	zero_poly(tp1, par*2);
	tp1[0] = 1;

	for (i = 1; i <= par; i++) 
	{
		zero_poly(tp, par*2);
		tp[0] = gexp[i];		// set up x+a^n
		tp[1] = 1;

		mult_polys(genpoly, tp, tp1, par*2);
		copy_poly(tp1, genpoly, par*2);
	}

	for(i=0; i<par/2; i++)
	{
		tmp = genpoly[i] ;
		genpoly[i] = genpoly[par-1-i];
		genpoly[par-1-i]=tmp;
	}

	for(i=0; i<256; i++)
	{
		for(int j=0; j<23; j++)
			genpoly_23_mult_cache[i][j] = gmult(i, genpoly_cache[22][j]);
		genpoly_23_mult_cache[i][23] = 0;
	}
}