#pragma once

#ifdef WIN32
#define MAX_DIMENSION 20
#else
#define MAX_DIMENSION 16
#endif
#define MAX_MATRIX_ELEMENTS (MAX_DIMENSION*MAX_DIMENSION)

#define real_t float

class matrix
{
public:
	int m;		// m rows
	int n;		// n columns
	real_t data[MAX_MATRIX_ELEMENTS];		// m * n matrix, row first
	//  [0 ...      n-1 ]
	//  [m ...       .. ]
	//  [..          .. ]
	//  [(m-1)*n   n*m-1]

	matrix(int m, int n, ...);		// be aware of variable types
	matrix(int m, int n, real_t data[]);
	matrix();
	matrix(int d);				// d*d identity matrix
	matrix(const matrix&v);		// copy constructor
	~matrix();

	real_t & operator [](int index){return data[index];}
	real_t & operator ()(int _m, int _n){return data[_m*n+_n];}
	void operator =(const matrix &v);
	void operator +=(const matrix &v);
	matrix operator +(const matrix &v);
	void operator -=(const matrix &v);
	matrix operator -(const matrix &v);
	void operator *=(const matrix &v);
	matrix operator *(const matrix &v);
	void operator *=(const real_t &v);
	matrix operator *(const real_t &v);
	void operator /=(const real_t &v);
	void operator /=(const matrix &v);
	matrix operator /(const matrix &v);
	matrix operator /(const real_t &v);
	matrix inverse();
	matrix inversef();
	real_t det();
	matrix cofactor(int a, int b);
	void identity();
	matrix transpos();
	static matrix diag(int n, ...);
	static matrix diag(int n, real_t data[]);

private:
	real_t det2x2();
	real_t det3x3();
};
