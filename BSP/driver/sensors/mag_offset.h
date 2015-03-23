#pragma once

class mag_offset
{
public:
	mag_offset();
	void add_value(float *v);						// v is a pointer to float[4]{x,y,z,1}
	void add_value(float **v, int n);
	void get_result(float *center, float*r);		// center is a pointer to float[3]{x,y,z}, r is radius
protected:
	float m_a[4][4];
	float m_b[4];
};
