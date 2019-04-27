#include "../Orchid/myrand.h"


namespace Orchid
{

	void _dorand48(unsigned short xseed[3])
	{
		unsigned long accu;
		unsigned short temp[2];

		accu = (unsigned long)_rand48_mult[0] * (unsigned long)xseed[0] +
			(unsigned long)_rand48_add;
		temp[0] = (unsigned short)accu;        /* lower 16 bits */
		accu >>= sizeof(unsigned short) * 8;
		accu += (unsigned long)_rand48_mult[0] * (unsigned long)xseed[1] +
			(unsigned long)_rand48_mult[1] * (unsigned long)xseed[0];
		temp[1] = (unsigned short)accu;        /* middle 16 bits */
		accu >>= sizeof(unsigned short) * 8;
		accu += _rand48_mult[0] * xseed[2] + _rand48_mult[1] * xseed[1] + _rand48_mult[2] * xseed[0];
		xseed[0] = temp[0];
		xseed[1] = temp[1];
		xseed[2] = (unsigned short)accu;
	}

	double myrand(unsigned short xseed[3])
	{
		_dorand48(xseed);
		return ldexp((double)xseed[0], -48) +
			ldexp((double)xseed[1], -32) +
			ldexp((double)xseed[2], -16);
	}
}

