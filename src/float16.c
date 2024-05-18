#include "float16.h"

// This is an implementation of conversion functions between normal 
// IEEE 754 binary32 floats and binary16 half floats.
//
// Extracting the relevant code and building this small library was
// done by Nathan Holmes and Iowa Scaled Engineering
//
// This code borrows heavily from two functions from Industrial Light &
// Magic's OpenEXR code, which is released under a modified BSD license.
// As such, the required copyright notice is preserved here:
//
// Copyright (c) 2002-2011, Industrial Light & Magic, a division of Lucasfilm 
// Entertainment Company Ltd. All rights reserved. 
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// * Neither the name of Industrial Light & Magic nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//


/*
// IEEE 754 binary16 format

1111 1100 0000 0000
5432 1098 7654 3210
SEEE EEMM MMMM MMMM

S = Sign bit - 0=positive, 1=negative
E = Exponent, bias = 15
M = Mantissa / significand, 10 bits


// IEEE 754 binary32 format

3322 2222 2222 1111 1111 1100 0000 0000
1098 7654 3210 9876 5432 1098 7654 3210
SEEE EEEE EMMM MMMM MMMM MMMM MMMM MMMM

S = Sign bit - 0=positive, 1=negative
E = Exponent, bias = 127
M = Mantissa / significand, 23 bits

*/


float16_t F32toF16(float in)
{
	void* ptr = &in;
	uint32_t f = *((uint32_t*)ptr);
	float16_t f16 = 0;
	int32_t e = (f >> 23) & 0x000000ff;
	int32_t s = (f >> 16) & 0x00008000;
	int32_t m = f & 0x007fffff;

	e -= 127; // Remove exponent bias from float

	if (128 == e)
	{
		// infinity or NAN
		f16 = s | 0x7C00 | (m >> 13);
	} else if (e > 15) {
		// exponent too bit to represent in half float, just overflow 
		// to positive or negative infinity, as appropriate
		f16 = s | 0x7C00;
	} else if (e > -15) {
		if ((m & 0x00003fff) == 0x00001000)
		{
			// tie, round down to even
			f16 |= s | ((e+15) << 10) | (m >> 13);
		} else {
			// all non-ties, and tie round up to even
			//   (note that a mantissa of all 1's will round up to all 0's with
			//   the exponent being increased by 1, which is exactly what we want;
			//   for example, "0.5-epsilon" rounds up to 0.5, and 65535.0 rounds
			//   up to infinity.)
			f16 = s | (((e+15) << 10) + ((m + 0x00001000) >> 13));
		}
	} else if (e > -25) {
		// convert to subnormal
		m |= 0x00800000; // restore the implied bit
		e = -14 - e; // shift count
		m >>= e; // M now in position but 2^13 too big
		if ((m & 0x00003fff) != 0x00001000) {
			// all non-ties, and tie round up to even
			m += (1 << 12); // m += 0x00001000
		}
		m >>= 13;
		f16 = s | m;
	} else {
		// zero, or underflow
		f16 = s;
	}
	return f16;
}

float F16toF32(float16_t in)
{
	int32_t s = in & 0x8000;
	int32_t e = (in & 0x7c00) >> 10;
	int32_t m = in & 0x03ff;
	uint32_t out = 0;
	void* ptr;

	s <<= 16;
	if (e == 31) 
	{
		// infinity or NAN
		e = 255L << 23;
		m <<= 13;
		out = s | e | m;
	} else if (e > 0) {
		// normalized
		e += (127 - 15);
		e <<= 23;
		m <<= 13;
		out = s | e | m;
	} else if (m == 0) {
		// zero
		out = s;
	} else {
		// subnormal, value is m times 2^-24
		float f = ((float) m);
		ptr = &f;
		out = s | (*(uint32_t*)ptr - (24L << 23));
	}
	ptr = &out;
	return(*((float*)ptr));
}


