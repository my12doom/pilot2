#ifndef GFMATH_H
#define GFMATH_H

#define MAX_NPAR 128

// galois arithmetic tables
void init_exp_table (void);
extern unsigned char gexp[];
extern unsigned char glog[];
extern unsigned char mult_table[256][256];
extern unsigned char EKR255_table[256][256];

//gmult opt
//#define _USE_INLINE_GMULT
#ifndef _USE_INLINE_GMULT
#define gmult(a,b) mult_table[a][b]
#define gmult2(a,b) mult_table2[((a)<<8) | (b)]
#define mul_z_poly(src, size) memmove(src+1, src, size-1);src[0]=0
#else
#define gmult c_gmult
#define mul_z_poly c_mul_z_poly
#endif

inline int c_gmult(int a, int b)
{
	//unsigned char i,j;
	if (a==0 || b == 0) return (0);
	//i = glog[a];
	//j = glog[b];
	//return (gexp[i+j]);
	return (gexp[glog[a]+glog[b]]);
}

inline int ginv (int elt)
{
	return (gexp[255-glog[elt]]);
}

// polynomial arithmetic
void add_polys(unsigned char *dst, unsigned char *src, int size) ;
void scale_poly(int k, unsigned char *poly, int size);
void mult_polys(unsigned char *dst, unsigned char *p1, unsigned char *p2, int size);
void c_mul_z_poly (unsigned char *src, int size);
void copy_poly(unsigned char *dst, unsigned char *src, int size);
void zero_poly(unsigned char *poly, int size);

// my12doom's gen poly cache
extern unsigned char genpoly_cache[MAX_NPAR][256];
extern unsigned char genpoly_23_mult_cache[256][32];
#define get_genpoly(n) genpoly_cache[n-1]		// for backward compatibility

#endif