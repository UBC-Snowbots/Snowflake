/*
 * Copyright (c) 1993 Martin Birgmeier
 * All rights reserved.
 *
 * You may redistribute unmodified or modified versions of this source
 * code provided that the above copyright notice and this and the
 * following conditions are retained.
 *
 * This software is provided ``as is'', and comes with no warranties
 * of any kind. I shall in no event be liable for anything that happens
 * to anyone/anything when using this software.
 */

#include "mos_random.h"

/*
 * The rand48() family of functions generates pseudo-random numbers using a
 * linear congruential algorithm working on integers 48 bits in size.  The
 * particular formula employed is r(n+1) = (a * r(n) + c) mod m where the
 * default values are for the multiplicand a = 0x5deece66d = 25214903917 and
 * the addend c = 0xb = 11.  The modulo is always fixed at m = 2 ** 48.
 * r(n) is called the seed of the random number generator.
 *
 * For all the six generator routines described next, the first computa-
 * tional step is to perform a single iteration of the algorithm.
 *
 * The lrand48() and nrand48() functions return values of type long in the
 * range [0, 2**31-1].  The high-order (31) bits of r(n+1) are loaded into
 * the lower bits of the returned value, with the topmost (sign) bit set to
 * zero.
 *
 * The mrand48() and jrand48() functions return values of type long in the
 * range [-2**31, 2**31-1].  The high-order (32) bits of r(n+1) are loaded
 * into the returned value.
 *
 * The lrand48(), and mrand48() functions use an internal buffer
 * to store r(n).  For these functions the initial value of r(0) =
 * 0x1234abcd330e = 20017429951246.
 *
 * On the other hand, nrand48(), and jrand48() use a user-supplied buffer
 * to store the seed r(n), which consists of an array of 3
 * shorts, where the zeroth member holds the least significant bits.
 * All functions share the same multiplicand and addend.
 *
 * The srand48() function is used to initialize the internal buffer r(n) of
 * lrand48(), and mrand48() such that the 32 bits of the seed
 * value are copied into the upper 32 bits of r(n), with the lower 16 bits
 * of r(n) arbitrarily being set to 0x330e.  Additionally, the constant mul-
 * tiplicand and addend of the algorithm are reset to the default values
 * given above.
 *
 * The seed48() function also initializes the internal buffer r(n) of
 * lrand48(), and mrand48(), but here all 48 bits of the seed can
 * be specified in an array of 3 shorts, where the zeroth member specifies
 * the lowest bits.  Again, the constant multiplicand and addend of the
 * algorithm are reset to the default values given above.  The seed48()
 * function returns a pointer to an array of 3 shorts which contains the old
 * seed.  This array is statically allocated, thus its contents are lost
 * after each new call to seed48().
 *
 * Finally, lcong48() allows full control over the multiplicand and addend
 * used in lrand48(), nrand48(), mrand48(), and jrand48(), and the seed
 * used in lrand48(), and mrand48().  An array of 7 shorts is passed as
 * argument; the first three shorts are used to initialize the seed;
 * the second three are used to initialize the multiplicand; and the last
 * short is used to initialize the addend.  It is thus not possible to use
 * values greater than 0xffff as the addend.
 */

#define	RAND48_SEED_0	(0x330e)
#define	RAND48_SEED_1	(0xabcd)
#define	RAND48_SEED_2	(0x1234)
#define	RAND48_MULT_0	(0xe66d)
#define	RAND48_MULT_1	(0xdeec)
#define	RAND48_MULT_2	(0x0005)
#define	RAND48_ADD		(0x000b)

static uint16_t _rand48_seed[3] = {
	RAND48_SEED_0,
	RAND48_SEED_1,
	RAND48_SEED_2
};

static uint16_t _rand48_mult[3] = {
	RAND48_MULT_0,
	RAND48_MULT_1,
	RAND48_MULT_2
};

static uint16_t _rand48_add = RAND48_ADD;

static void
_dorand48(uint16_t xseed[3]) {
	uint16_t temp[2];
	uint32_t accu;

	accu = (uint32_t) _rand48_mult[0] * (uint32_t) xseed[0] +
	 (uint32_t) _rand48_add;
	temp[0] = (uint16_t) accu;	/* lower 16 bits */
	accu >>= sizeof(uint16_t) * 8;
	accu += (uint32_t) _rand48_mult[0] * (uint32_t) xseed[1] +
	 (uint32_t) _rand48_mult[1] * (uint32_t) xseed[0];
	temp[1] = (uint16_t) accu;	/* middle 16 bits */
	accu >>= sizeof(uint16_t) * 8;
	accu += _rand48_mult[0] * xseed[2] + _rand48_mult[1] *
	  xseed[1] + _rand48_mult[2] * xseed[0];
	xseed[0] = temp[0];
	xseed[1] = temp[1];
	xseed[2] = (uint16_t) accu;
}


MOSAPI void MOSCConv
mos_lcong48(uint16_t p[7]) {

	_rand48_seed[0] = p[0];
	_rand48_seed[1] = p[1];
	_rand48_seed[2] = p[2];
	_rand48_mult[0] = p[3];
	_rand48_mult[1] = p[4];
	_rand48_mult[2] = p[5];
	_rand48_add = p[6];
}

MOSAPI int32_t MOSCConv
mos_mrand48(void) {

	_dorand48(_rand48_seed);
	return ((int32_t) _rand48_seed[2] << 16) + (int32_t) _rand48_seed[1];
}

MOSAPI int32_t MOSCConv
mos_jrand48(uint16_t xseed[3]) {

	_dorand48(xseed);
	return ((int32_t) xseed[2] << 16) + (int32_t) xseed[1];
}

MOSAPI int32_t MOSCConv
mos_lrand48(void) {

	_dorand48(_rand48_seed);
	return ((int32_t) _rand48_seed[2] << 15) + ((int32_t) _rand48_seed[1] >> 1);
}

MOSAPI int32_t MOSCConv
mos_nrand48(uint16_t xseed[3]) {

	_dorand48(xseed);
	return ((int32_t) xseed[2] << 15) + ((int32_t) xseed[1] >> 1);
}

MOSAPI uint16_t * MOSCConv
mos_seed48(uint16_t xseed[3]) {
	static uint16_t sseed[3];

	sseed[0] = _rand48_seed[0];
	sseed[1] = _rand48_seed[1];
	sseed[2] = _rand48_seed[2];

	_rand48_seed[0] = xseed[0];
	_rand48_seed[1] = xseed[1];
	_rand48_seed[2] = xseed[2];
	_rand48_mult[0] = RAND48_MULT_0;
	_rand48_mult[1] = RAND48_MULT_1;
	_rand48_mult[2] = RAND48_MULT_2;
	_rand48_add = RAND48_ADD;

	return (sseed);
}

MOSAPI void MOSCConv
mos_srand48(int32_t seed) {

	_rand48_seed[0] = RAND48_SEED_0;
	_rand48_seed[1] = (uint16_t) seed;
	_rand48_seed[2] = (uint16_t) (seed >> 16);
	_rand48_mult[0] = RAND48_MULT_0;
	_rand48_mult[1] = RAND48_MULT_1;
	_rand48_mult[2] = RAND48_MULT_2;
	_rand48_add = RAND48_ADD;
}
