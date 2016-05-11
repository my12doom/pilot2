#pragma once

#define MAX_DIMENSION 16
#define MAX_MATRIX_ELEMENTS (MAX_DIMENSION*MAX_DIMENSION)



class matrix
{
public:
	int m;		// m rows
	int n;		// n columns
	float data[MAX_MATRIX_ELEMENTS];		// m * n matrix, row first
	//  [0 ...      n-1 ]
	//  [m ...       .. ]
	//  [..          .. ]
	//  [(m-1)*n   n*m-1]

	matrix(int m, int n, ...);		// be aware of variable types
	matrix(int m, int n, float data[]);
	matrix();
	matrix(int d);				// d*d identity matrix
	matrix(const matrix&v);		// copy constructor
	~matrix();

	float & operator [](int index){return data[index];}
	float & operator ()(int _m, int _n){return data[_m*n+_n];}
	void operator =(const matrix &v);
	void operator +=(const matrix &v);
	matrix operator +(const matrix &v);
	void operator -=(const matrix &v);
	matrix operator -(const matrix &v);
	void operator *=(const matrix &v);
	matrix operator *(const matrix &v);
	void operator *=(const float &v);
	matrix operator *(const float &v);
	void operator /=(const float &v);
	void operator /=(const matrix &v);
	matrix operator /(const matrix &v);
	matrix operator /(const float &v);
	matrix inverse();
	matrix inversef();
	float det();
	matrix cofactor(int a, int b);
	void identity();
	matrix transpos();
	static matrix diag(int n, ...);

private:
	float det2x2();
	float det3x3();
};
