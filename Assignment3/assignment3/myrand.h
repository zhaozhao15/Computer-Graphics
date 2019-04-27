#ifdef _MSC_VER
#pragma once
#endif // _MSC_VER
#ifndef _MYRANDH_
#define _MYRANDH_


#include <cmath>
#include "../Orchid/myrand.h"

#define RAND48_SEED_0   (0x330e)
#define RAND48_SEED_1   (0xabcd)
#define RAND48_SEED_2   (0x1234)
#define RAND48_MULT_0   (0xe66d)
#define RAND48_MULT_1   (0xdeec)
#define RAND48_MULT_2   (0x0005)
#define RAND48_ADD      (0x000b)
namespace Orchid
{
	const unsigned short _rand48_add = RAND48_ADD;
	const unsigned short _rand48_mult[3] = {
		RAND48_MULT_0,
		RAND48_MULT_1,
		RAND48_MULT_2
	};
	void _dorand48(unsigned short xseed[3]);

	double myrand(unsigned short xseed[3]);
}

#endif // !_MYRANDH_