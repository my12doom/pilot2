#include "GFMath.h"
#include "append.h"

#define NORMAL_INST

void rs_append1(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(1);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = gmult(dbyte, genPoly[0]);
	}
}

void rs_append2(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(2);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = gmult(dbyte, genPoly[1]);
	}
}

void rs_append3(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(3);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = gmult(dbyte, genPoly[2]);
	}
}

void rs_append4(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(4);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = gmult(dbyte, genPoly[3]);
	}
}

void rs_append5(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(5);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = gmult(dbyte, genPoly[4]);
	}
}

void rs_append6(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(6);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = gmult(dbyte, genPoly[5]);
	}
}

void rs_append7(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(7);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = gmult(dbyte, genPoly[6]);
	}
}

void rs_append8(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(8);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = gmult(dbyte, genPoly[7]);
	}
}

void rs_append9(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(9);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = gmult(dbyte, genPoly[8]);
	}
}

void rs_append10(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(10);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = gmult(dbyte, genPoly[9]);
	}
}

void rs_append11(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(11);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = gmult(dbyte, genPoly[10]);
	}
}

void rs_append12(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(12);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = gmult(dbyte, genPoly[11]);
	}
}

void rs_append13(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(13);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = gmult(dbyte, genPoly[12]);
	}
}

void rs_append14(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(14);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = gmult(dbyte, genPoly[13]);
	}
}

void rs_append15(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(15);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = gmult(dbyte, genPoly[14]);
	}
}

void rs_append16(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(16);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = gmult(dbyte, genPoly[15]);
	}
}

void rs_append17(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(17);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = gmult(dbyte, genPoly[16]);
	}
}

void rs_append18(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(18);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = gmult(dbyte, genPoly[17]);
	}
}

void rs_append19(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(19);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = gmult(dbyte, genPoly[18]);
	}
}

void rs_append20(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(20);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = gmult(dbyte, genPoly[19]);
	}
}

void rs_append21(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(21);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = gmult(dbyte, genPoly[20]);
	}
}

void rs_append22(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(22);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = gmult(dbyte, genPoly[21]);
	}
}

void rs_append23(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(23);
#ifdef NORMAL_INST
	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = gmult(dbyte, genPoly[22]);
	}
#else
	LFSR[23] = 0;
	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];

		__m128i m0 = _mm_load_si128((__m128i*)(LFSR));
		__m128i m01= _mm_load_si128((__m128i*)(LFSR+16));
		__m128i tmp = _mm_slli_si128(m01, 15);
		m0 = _mm_srli_si128(m0, 1);
		m0 = _mm_or_si128(m0, tmp);
		m01 = _mm_srli_si128(m01, 1);
		__m128i m1 = _mm_load_si128((__m128i*)genpoly_23_mult_cache[dbyte]);
		__m128i m11= _mm_load_si128((__m128i*)(genpoly_23_mult_cache[dbyte]+16));


		m0 = _mm_xor_si128(m0, m1);
		m1 = _mm_xor_si128(m01, m11);

		_mm_store_si128((__m128i*) LFSR, m0);
		_mm_store_si128((__m128i*) (LFSR+16), m1);
	}
#endif
}

void rs_append24(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(24);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = gmult(dbyte, genPoly[23]);
	}
}

void rs_append25(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(25);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = gmult(dbyte, genPoly[24]);
	}
}

void rs_append26(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(26);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = gmult(dbyte, genPoly[25]);
	}
}

void rs_append27(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(27);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = gmult(dbyte, genPoly[26]);
	}
}

void rs_append28(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(28);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = gmult(dbyte, genPoly[27]);
	}
}

void rs_append29(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(29);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = gmult(dbyte, genPoly[28]);
	}
}

void rs_append30(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(30);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = gmult(dbyte, genPoly[29]);
	}
}

void rs_append31(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(31);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = gmult(dbyte, genPoly[30]);
	}
}

void rs_append32(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(32);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = gmult(dbyte, genPoly[31]);
	}
}

void rs_append33(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(33);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = gmult(dbyte, genPoly[32]);
	}
}

void rs_append34(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(34);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = LFSR[33] ^ gmult(dbyte, genPoly[32]);
		LFSR[33] = gmult(dbyte, genPoly[33]);
	}
}

void rs_append35(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(35);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = LFSR[33] ^ gmult(dbyte, genPoly[32]);
		LFSR[33] = LFSR[34] ^ gmult(dbyte, genPoly[33]);
		LFSR[34] = gmult(dbyte, genPoly[34]);
	}
}

void rs_append36(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(36);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = LFSR[33] ^ gmult(dbyte, genPoly[32]);
		LFSR[33] = LFSR[34] ^ gmult(dbyte, genPoly[33]);
		LFSR[34] = LFSR[35] ^ gmult(dbyte, genPoly[34]);
		LFSR[35] = gmult(dbyte, genPoly[35]);
	}
}

void rs_append37(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(37);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = LFSR[33] ^ gmult(dbyte, genPoly[32]);
		LFSR[33] = LFSR[34] ^ gmult(dbyte, genPoly[33]);
		LFSR[34] = LFSR[35] ^ gmult(dbyte, genPoly[34]);
		LFSR[35] = LFSR[36] ^ gmult(dbyte, genPoly[35]);
		LFSR[36] = gmult(dbyte, genPoly[36]);
	}
}

void rs_append38(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(38);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = LFSR[33] ^ gmult(dbyte, genPoly[32]);
		LFSR[33] = LFSR[34] ^ gmult(dbyte, genPoly[33]);
		LFSR[34] = LFSR[35] ^ gmult(dbyte, genPoly[34]);
		LFSR[35] = LFSR[36] ^ gmult(dbyte, genPoly[35]);
		LFSR[36] = LFSR[37] ^ gmult(dbyte, genPoly[36]);
		LFSR[37] = gmult(dbyte, genPoly[37]);
	}
}

void rs_append39(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(39);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = LFSR[33] ^ gmult(dbyte, genPoly[32]);
		LFSR[33] = LFSR[34] ^ gmult(dbyte, genPoly[33]);
		LFSR[34] = LFSR[35] ^ gmult(dbyte, genPoly[34]);
		LFSR[35] = LFSR[36] ^ gmult(dbyte, genPoly[35]);
		LFSR[36] = LFSR[37] ^ gmult(dbyte, genPoly[36]);
		LFSR[37] = LFSR[38] ^ gmult(dbyte, genPoly[37]);
		LFSR[38] = gmult(dbyte, genPoly[38]);
	}
}

void rs_append40(unsigned char *msg, int nbytes, unsigned char *LFSR)
{
	int i;
	unsigned char dbyte;
	unsigned char *genPoly = get_genpoly(40);

	for (i=0; i<nbytes; i++)
	{
		dbyte = msg[i] ^ LFSR[0];
		LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
		LFSR[1] = LFSR[2] ^ gmult(dbyte, genPoly[1]);
		LFSR[2] = LFSR[3] ^ gmult(dbyte, genPoly[2]);
		LFSR[3] = LFSR[4] ^ gmult(dbyte, genPoly[3]);
		LFSR[4] = LFSR[5] ^ gmult(dbyte, genPoly[4]);
		LFSR[5] = LFSR[6] ^ gmult(dbyte, genPoly[5]);
		LFSR[6] = LFSR[7] ^ gmult(dbyte, genPoly[6]);
		LFSR[7] = LFSR[8] ^ gmult(dbyte, genPoly[7]);
		LFSR[8] = LFSR[9] ^ gmult(dbyte, genPoly[8]);
		LFSR[9] = LFSR[10] ^ gmult(dbyte, genPoly[9]);
		LFSR[10] = LFSR[11] ^ gmult(dbyte, genPoly[10]);
		LFSR[11] = LFSR[12] ^ gmult(dbyte, genPoly[11]);
		LFSR[12] = LFSR[13] ^ gmult(dbyte, genPoly[12]);
		LFSR[13] = LFSR[14] ^ gmult(dbyte, genPoly[13]);
		LFSR[14] = LFSR[15] ^ gmult(dbyte, genPoly[14]);
		LFSR[15] = LFSR[16] ^ gmult(dbyte, genPoly[15]);
		LFSR[16] = LFSR[17] ^ gmult(dbyte, genPoly[16]);
		LFSR[17] = LFSR[18] ^ gmult(dbyte, genPoly[17]);
		LFSR[18] = LFSR[19] ^ gmult(dbyte, genPoly[18]);
		LFSR[19] = LFSR[20] ^ gmult(dbyte, genPoly[19]);
		LFSR[20] = LFSR[21] ^ gmult(dbyte, genPoly[20]);
		LFSR[21] = LFSR[22] ^ gmult(dbyte, genPoly[21]);
		LFSR[22] = LFSR[23] ^ gmult(dbyte, genPoly[22]);
		LFSR[23] = LFSR[24] ^ gmult(dbyte, genPoly[23]);
		LFSR[24] = LFSR[25] ^ gmult(dbyte, genPoly[24]);
		LFSR[25] = LFSR[26] ^ gmult(dbyte, genPoly[25]);
		LFSR[26] = LFSR[27] ^ gmult(dbyte, genPoly[26]);
		LFSR[27] = LFSR[28] ^ gmult(dbyte, genPoly[27]);
		LFSR[28] = LFSR[29] ^ gmult(dbyte, genPoly[28]);
		LFSR[29] = LFSR[30] ^ gmult(dbyte, genPoly[29]);
		LFSR[30] = LFSR[31] ^ gmult(dbyte, genPoly[30]);
		LFSR[31] = LFSR[32] ^ gmult(dbyte, genPoly[31]);
		LFSR[32] = LFSR[33] ^ gmult(dbyte, genPoly[32]);
		LFSR[33] = LFSR[34] ^ gmult(dbyte, genPoly[33]);
		LFSR[34] = LFSR[35] ^ gmult(dbyte, genPoly[34]);
		LFSR[35] = LFSR[36] ^ gmult(dbyte, genPoly[35]);
		LFSR[36] = LFSR[37] ^ gmult(dbyte, genPoly[36]);
		LFSR[37] = LFSR[38] ^ gmult(dbyte, genPoly[37]);
		LFSR[38] = LFSR[39] ^ gmult(dbyte, genPoly[38]);
		LFSR[39] = gmult(dbyte, genPoly[39]);
	}
}

void rs_append(unsigned char *msg, int nbytes, unsigned char *LFSR, int NPAR)
{
	switch(NPAR)
	{
	case 1:rs_append1(msg, nbytes, LFSR);break;
	case 2:rs_append2(msg, nbytes, LFSR);break;
	case 3:rs_append3(msg, nbytes, LFSR);break;
	case 4:rs_append4(msg, nbytes, LFSR);break;
	case 5:rs_append5(msg, nbytes, LFSR);break;
	case 6:rs_append6(msg, nbytes, LFSR);break;
	case 7:rs_append7(msg, nbytes, LFSR);break;
	case 8:rs_append8(msg, nbytes, LFSR);break;
	case 9:rs_append9(msg, nbytes, LFSR);break;
	case 10:rs_append10(msg, nbytes, LFSR);break;
	case 11:rs_append11(msg, nbytes, LFSR);break;
	case 12:rs_append12(msg, nbytes, LFSR);break;
	case 13:rs_append13(msg, nbytes, LFSR);break;
	case 14:rs_append14(msg, nbytes, LFSR);break;
	case 15:rs_append15(msg, nbytes, LFSR);break;
	case 16:rs_append16(msg, nbytes, LFSR);break;
	case 17:rs_append17(msg, nbytes, LFSR);break;
	case 18:rs_append18(msg, nbytes, LFSR);break;
	case 19:rs_append19(msg, nbytes, LFSR);break;
	case 20:rs_append20(msg, nbytes, LFSR);break;
	case 21:rs_append21(msg, nbytes, LFSR);break;
	case 22:rs_append22(msg, nbytes, LFSR);break;
	case 23:rs_append23(msg, nbytes, LFSR);break;
	case 24:rs_append24(msg, nbytes, LFSR);break;
	case 25:rs_append25(msg, nbytes, LFSR);break;
	case 26:rs_append26(msg, nbytes, LFSR);break;
	case 27:rs_append27(msg, nbytes, LFSR);break;
	case 28:rs_append28(msg, nbytes, LFSR);break;
	case 29:rs_append29(msg, nbytes, LFSR);break;
	case 30:rs_append30(msg, nbytes, LFSR);break;
	case 31:rs_append31(msg, nbytes, LFSR);break;
	case 32:rs_append32(msg, nbytes, LFSR);break;
	case 33:rs_append33(msg, nbytes, LFSR);break;
	case 34:rs_append34(msg, nbytes, LFSR);break;
	case 35:rs_append35(msg, nbytes, LFSR);break;
	case 36:rs_append36(msg, nbytes, LFSR);break;
	case 37:rs_append37(msg, nbytes, LFSR);break;
	case 38:rs_append38(msg, nbytes, LFSR);break;
	case 39:rs_append39(msg, nbytes, LFSR);break;
	case 40:rs_append40(msg, nbytes, LFSR);break;
	default:
		{
			unsigned char *genPoly = get_genpoly(NPAR);

			for (int i=0; i<nbytes; i++)
			{
// 					dbyte = msg[i] ^ LFSR[0];
// 					LFSR[0] = LFSR[1] ^ gmult(dbyte, genPoly[0]);
// 					LFSR[1] = gmult(dbyte, genPoly[1]);
				unsigned char dbyte = msg[i] ^ LFSR[0];
				for (int j = 0; j < NPAR-1; j++) 
				{
					LFSR[j] = LFSR[j+1] ^ gmult(dbyte, genPoly[j]);
				}
				LFSR[NPAR-1] = gmult(dbyte, genPoly[NPAR-1]);
			}
		}
		break;
	}
}