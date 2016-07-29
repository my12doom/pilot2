#include "matrix.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdint.h>

#ifndef WIN32
#define assert(...)
#endif

static void matrix_error(const char*msg)
{

}


matrix::matrix()
{
	n = m =0;		// invalid matrix
}

matrix::matrix(int d)
{
	n = m = d;
	identity();
}

matrix::matrix(const matrix &v)
{
	n = v.n;
	m = v.m;
	memcpy(data, v.data, sizeof(float)*n*m);
}

matrix::matrix(int m, int n, float data[])
{
	this->m = m;
	this->n = n;
	memcpy(this->data, data, sizeof(float)*n*m);
}

matrix::matrix(int m, int n, ...)
{
	this->m = m;
	this->n = n;
	int count = m*n;
	
	va_list vl;
	va_start(vl,n);

	for(int i=0; i<count; i++)
		this->data[i] = va_arg(vl,double);
	va_end(vl);
}
matrix::~matrix()
{

}

void matrix::operator =(const matrix &v)
{
	n = v.n;
	m = v.m;
	memcpy(data, v.data, sizeof(float)*n*m);
}

void matrix::operator +=(const matrix &v)
{
	assert(n == v.n && m == v.m);
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] += v.data[i];
}
matrix matrix::operator +(const matrix &v)
{
	matrix o(*this);
	o += v;
	return o;
}
void matrix::operator -=(const matrix &v)
{
	assert(n == v.n && m == v.m);
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] -= v.data[i];
}
matrix matrix::operator -(const matrix &v)
{
	matrix o(*this);
	o -= v;
	return o;
}
void matrix::operator *=(const matrix &v)
{
	assert(n == v.m);	// invalid matrix_mul
	matrix o;
	o.m = m;
	o.n = v.n;

#if 0

	for(int x1 = 0; x1<v.n; x1++)
	{
		for(int y1 = 0; y1<m; y1++)
		{
			o.data[y1*v.n+x1] = 0;
			for(int k = 0; k<n; k++)
				o.data[y1*v.n+x1] += data[y1*n+k] * v.data[k*v.n+x1];
		}
	}
#else
  float *pIn1 = this->data;                /* input data matrix pointer A */
  const float *pIn2 = v.data;                /* input data matrix pointer B */
  float *pInA = this->data;                /* input data matrix pointer A  */
  float *pOut = o.data;                 /* output data matrix pointer */
  float *px;                                 /* Temporary output data matrix pointer */
  float sum;                                 /* Accumulator */
  uint16_t numRowsA = this->m;            /* number of rows of input matrix A */
  uint16_t numColsB = v.n;            /* number of columns of input matrix B */
  uint16_t numColsA = this->n;            /* number of columns of input matrix A */

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  float in1, in2, in3, in4;
  uint16_t col, i = 0u, j, row = numRowsA, colCnt;      /* loop counters */
//   arm_status status;                             /* status of matrix multiplication */

  {
    /* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
    /* row loop */
    do
    {
      /* Output pointer is set to starting address of the row being processed */
      px = pOut + i;

      /* For every row wise process, the column loop counter is to be initiated */
      col = numColsB;

      /* For every row wise process, the pIn2 pointer is set    
       ** to the starting address of the pSrcB data */
      pIn2 = v.data;

      j = 0u;

      /* column loop */
      do
      {
        /* Set the variable sum, that acts as accumulator, to zero */
        sum = 0.0f;

        /* Initiate the pointer pIn1 to point to the starting address of the column being processed */
        pIn1 = pInA;

        /* Apply loop unrolling and compute 4 MACs simultaneously. */
        colCnt = numColsA >> 2u;

        /* matrix multiplication        */
        while(colCnt > 0u)
        {
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          in3 = *pIn2;
          pIn2 += numColsB;
          in1 = pIn1[0];
          in2 = pIn1[1];
          sum += in1 * in3;
          in4 = *pIn2;
          pIn2 += numColsB;
          sum += in2 * in4;

          in3 = *pIn2;
          pIn2 += numColsB;
          in1 = pIn1[2];
          in2 = pIn1[3];
          sum += in1 * in3;
          in4 = *pIn2;
          pIn2 += numColsB;
          sum += in2 * in4;
          pIn1 += 4u;

          /* Decrement the loop count */
          colCnt--;
        }

        /* If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.    
         ** No loop unrolling is used. */
        colCnt = numColsA % 0x4u;

        while(colCnt > 0u)
        {
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;

          /* Decrement the loop counter */
          colCnt--;
        }

        /* Store the result in the destination buffer */
        *px++ = sum;

        /* Update the pointer pIn2 to point to the  starting address of the next column */
        j++;
        pIn2 = v.data + j;

        /* Decrement the column loop counter */
        col--;

      } while(col > 0u);


      /* Update the pointer pInA to point to the  starting address of the next row */
      i = i + numColsB;
      pInA = pInA + numColsA;

      /* Decrement the row loop counter */
      row--;

    } while(row > 0u);

    /* Set status as ARM_MATH_SUCCESS */
    //status = ARM_MATH_SUCCESS;
  }

  /* Return to application */
  //return (status);
#endif

	*this = o;
}
void matrix::operator *=(const float &v)
{
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] *= v;
}
matrix matrix::operator *(const matrix &v)
{
	matrix o(*this);
	o *= v;
	return o;
}
matrix matrix::operator *(const float &v)
{
	matrix o(*this);
	o *= v;
	return o;
}

float matrix::det()
{
	if (n == 2)
		return data[0] * data[3] - data[1] * data[2];

	if (n == 1)
		return data[0];

	float det = 0;
	for(int i=0; i<n; i++)
	{
		int symbol = i % 2 == 0 ? 1 : -1;
		matrix a_star = cofactor(i+1, 1);
		det += a_star.det() * symbol * data[i];
	}

	return det;
}
matrix matrix::inversef()
{
	assert(m==n);
	matrix y(*this);

	matrix A;
	A.m = m;
	A.n = n;

	int i0;
	signed char ipiv[MAX_DIMENSION];
	int j;
	int c;
	int jBcol;
	int ix;
	float smax;
	int k;
	float s;
	int i;
	int kAcol;
	signed char p[MAX_DIMENSION];
	int count = n*n;
	for (i0 = 0; i0 < count; i0++) {
		y[i0] = 0.0F;
		A[i0] = data[i0];
	}

	for (i0 = 0; i0 < m; i0++) {
		ipiv[i0] = (signed char)(1 + i0);
	}

	for (j = 0; j < m-1; j++) {
		c = j * (m+1);
		jBcol = 0;
		ix = c;
		smax = (float)fabs(A[c]);
		for (k = 2; k <= m - j; k++) {
			ix++;
			s = (float)fabs(A[ix]);
			if (s > smax) {
				jBcol = k - 1;
				smax = s;
			}
		}

		if (A[c + jBcol] != 0.0F) {
			if (jBcol != 0) {
				ipiv[j] = (signed char)((j + jBcol) + 1);
				ix = j;
				jBcol += j;
				for (k = 0; k < m; k++) {
					smax = A[ix];
					A[ix] = A[jBcol];
					A[jBcol] = smax;
					ix += m;
					jBcol += m;
				}
			}

			i0 = (c - j) + m;
			for (i = c + 1; i + 1 <= i0; i++) {
				A[i] /= A[c];
			}
		}

		jBcol = c;
		kAcol = c + m;
		for (i = 1; i <= m-1 - j; i++) {
			smax = A[kAcol];
			if (A[kAcol] != 0.0F) {
				ix = c + 1;
				i0 = (jBcol - j) + m*2;
				for (k = m+1 + jBcol; k + 1 <= i0; k++) {
					A[k] += A[ix] * -smax;
					ix++;
				}
			}

			kAcol += m;
			jBcol += m;
		}
	}

	for (i0 = 0; i0 < m; i0++) {
		p[i0] = (signed char)(1 + i0);
	}

	for (k = 0; k < m-1; k++) {
		if (ipiv[k] > 1 + k) {
			jBcol = p[ipiv[k] - 1];
			p[ipiv[k] - 1] = p[k];
			p[k] = (signed char)jBcol;
		}
	}

	for (k = 0; k < m; k++) {
		y[k + ((p[k] - 1) *m)] = 1.0F;
		for (j = k; j + 1 < m+1; j++) {
			if (y[j + ((p[k] - 1) * m)] != 0.0F) {
				for (i = j + 1; i + 1 < m+1; i++) {
					y[i + ((p[k] - 1) * m)] -= y[j + ((p[k] - 1) * m)] * A[i + (j * m)];
				}
			}
		}
	}

	for (j = 0; j < m; j++) {
		jBcol = j * m;
		for (k = m-1; k > -1; k += -1) {
			kAcol = k * m;
			if (y[k + jBcol] != 0.0F) {
				y[k + jBcol] /= A[k + kAcol];
				for (i = 0; i + 1 <= k; i++) {
					y[i + jBcol] -= y[k + jBcol] * A[i + kAcol];
				}
			}
		}
	}

	return y;	
}
matrix matrix::inverse()
{
	assert(m==n);
	matrix out(*this);

	if (m == 1)
	{
		out.data[0] = 1/data[0];
		return out;
	}
	float *o = (float*) out.data;
	for(int y=0; y<n; y++)
		for(int x=0; x<n; x++)
		{
			int daishu = x+1 + y+1;
			int symbol = daishu % 2 == 1 ? -1 : 1;
			matrix a_star = cofactor(x+1, y+1);
			o[x*n+y] =  a_star.det() * symbol;
		}

	out /= det();

	return out;
}

matrix matrix::cofactor(int x, int y)
{
	matrix out;
	out.n = n-1;
	out.m = m-1;

	float *p = (float*) data;
	float *o = (float*) out.data;

	int yy = 0;
	for(int iy=0; iy<n; iy++)
	{
		if (y != iy+1)
		{
			int xx = 0;
			for(int ix=0; ix<n; ix++)
			{
				if (x != ix+1)
				{
					o[yy*out.n+xx] = p[iy*n+ix];
					xx++;
				}
			}

			yy ++;
		}
	}

	return out;
}

matrix matrix::transpos()
{
	matrix out;

	out.m = n;
	out.n = m;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			out.data[i*m+j] = data[j*n+i];
		}
	}

	return out;
}

void matrix::identity()
{
	if (m!=n)
		return;
	memset(data, 0, m*n*sizeof(float));
	for(int i=0; i<m; i++)
		data[i*(m+1)] = 1;
}


void matrix::operator /=(const matrix &v)
{
	assert(v.n == v.m);
	matrix tmp(v);
	*this *= tmp.inverse();
}

matrix matrix::operator /(const matrix &v)
{
	matrix o(*this);
	o /= v;
	return o;
}
void matrix::operator /=(const float &v)
{
	int count = n*m;
	for(int i=0; i<count; i++)
		data[i] /= v;
}

matrix matrix::operator /(const float &v)
{
	matrix o(*this);
	o /= v;
	return o;
}

matrix matrix::diag(int n, ...)
{
	matrix o;
	o.n = n;
	o.m = n;

	memset(o.data, 0, n*n*sizeof(float));

	va_list vl;
	va_start(vl,n);
	for(int i=0; i<n; i++)
		o.data[i*(n+1)] = va_arg(vl,double);
	va_end(vl);
	
	return o;
}

matrix matrix::diag(int n, float data[])
{
	matrix o;
	o.n = n;
	o.m = n;

	memset(o.data, 0, n*n*sizeof(float));

	for(int i=0; i<n; i++)
		o.data[i*(n+1)] = data[i];

	return o;
}