#ifndef ORS_H
#define ORS_H


#define MAXDEG (NPAR*2)

#include "GFMath.h"


class rsEncoder
{
public:
	rsEncoder();
	rsEncoder(int par);
	~rsEncoder();
	void init(int par);
	void resetData();
	void append_data (unsigned char msg[], int nbytes);
	/*inline void append_1byte(int data)
	{

		int j;
		int dbyte;

		dbyte = data ^ LFSR[NPAR-1];
		for (j = NPAR-1; j > 0; j--) 
		{
			LFSR[j] = LFSR[j-1] ^ gmult(genPoly[j], dbyte);
		}
		LFSR[0] = gmult(genPoly[0], dbyte);
	}*/
	void output(unsigned char *dst);
private:
	unsigned char *LFSR;
	int NPAR;
};

class rsDecoder
{
public:
	rsDecoder();
	rsDecoder(int par);
	~rsDecoder();
	void init(int par);
	int decode_data(unsigned char *data, int cwsize);
	int decode_data(unsigned char *msg, int msg_size, unsigned char *parity);
	int correct_errors_erasures (unsigned char codeword[], 
								int csize,
								int nerasures,
								int erasures[]);

private:
	int NPAR;
	unsigned char *synBytes;
};

#endif