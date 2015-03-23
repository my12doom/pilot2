#include "mag_offset.h"
#include <string.h>
#include <math.h>

#define ABS(beta) (beta)>0?(beta):-(beta)
#define SWAP(m_a,m_b) {temp=(m_a);(m_a)=(m_b);(m_b)=temp;}


mag_offset::mag_offset()
{
	memset(&m_a[0][0], 0, sizeof(m_a));
	memset(&m_b[0], 0, sizeof(m_b));
}
void mag_offset::add_value(float **v, int n)
{
	for(int i=0; i<n; i++)
		add_value(v[i]);
}
void mag_offset::add_value(float *v)
{
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
			m_a[i][j] += v[i] * v[j];
		m_b[i]+=v[i]*(-v[0]*v[0]-v[1]*v[1]-v[2]*v[2]);
	}
}

void mag_offset::get_result(float *center, float*r)
{
	float temp;
	int ik;
	float a[4][4];
	float b[4];
	float beta[4];
	memcpy(&a[0][0], &m_a[0][0], sizeof(m_a));
	memcpy(&b[0], &m_b[0], sizeof(m_b));
	for(int k=0;k<4;k++)
	{
		int mik=-1;
		for(int i=k;i<4;i++)
			if(ABS(a[i][k])>mik)
			{
				mik=ABS(a[i][k]);
				ik=i;
			}
		for(int j=k;j<4;j++)
			SWAP(a[ik][j],a[k][j]);
		SWAP(b[k],b[ik]);
		b[k]/=a[k][k];
		for(int i=4-1;i>=k;i--)
			a[k][i]/=a[k][k];
		for(int i=k+1;i<4;i++)
		{
			b[i]-=a[i][k]*b[k];
			for(int j=4-1;j>=k;j--)
				a[i][j]-=a[i][k]*a[k][j];
		}
	}
	for(int i=4-1;i>=0;i--)
	{
		beta[i]=b[i];
		for(int j=i+1;j<4;j++)
			beta[i]-=a[i][j]*beta[j];
	}

	for(int i=0; i<3; i++)
		center[i] = -beta[i]/2;
	r[0] = sqrt(center[0]*center[0]+center[1]*center[1]+center[2]*center[2]-beta[3]);
}
