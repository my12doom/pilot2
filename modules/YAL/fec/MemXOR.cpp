/*
	Copyright (c) 2012-2014 Christopher A. Taylor.  All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	* Redistributions of source code must retain the above copyright notice,
	  this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright notice,
	  this list of conditions and the following disclaimer in the documentation
	  and/or other materials provided with the distribution.
	* Neither the name of LibCat nor the names of its contributors may be
	  used to endorse or promote products derived from this software without
	  specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

#include "MemXOR.hpp"
using namespace cat;

#ifdef CAT_HAS_VECTOR_EXTENSIONS
typedef uint64_t vec_block CAT_VECTOR_SIZE(uint64_t, 16);
#endif

void cat::memxor(void * CAT_RESTRICT voutput, const void * CAT_RESTRICT vinput, int bytes)
{
	/*
		Often times the output is XOR'd in-place so this version is
		faster than the one below with two inputs.

		There would be a decent performance improvement by using MMX
		if the buffers are all aligned.  However, I don't really have
		control over how the buffers are aligned because the block
		sizes are not always multiples of 16 bytes.  So after an hour
		of tuning this is the best version I found.
	*/

	// Primary engine
#ifdef CAT_WORD_64
	uint64_t * CAT_RESTRICT output_w = reinterpret_cast<uint64_t *>( voutput );
	const uint64_t * CAT_RESTRICT input_w = reinterpret_cast<const uint64_t *>( vinput );
#else
	uint32_t * CAT_RESTRICT output_w = reinterpret_cast<uint32_t *>( voutput );
	const uint32_t * CAT_RESTRICT input_w = reinterpret_cast<const uint32_t *>( vinput );
#endif

#ifdef CAT_HAS_VECTOR_EXTENSIONS
#ifdef CAT_WORD_64
	if ((*(uint64_t*)&output_w | *(uint64_t*)&input_w) & 15) {
#else
	if ((*(uint32_t*)&output_w | *(uint32_t*)&input_w) & 15) {
#endif
#endif
		while (bytes >= 128)
		{
#ifdef CAT_WORD_64
			for (int ii = 0; ii < 16; ++ii) {
				output_w[ii] ^= input_w[ii];
			}
			output_w += 16;
			input_w += 16;
#else
			for (int ii = 0; ii < 32; ++ii) {
				output_w[ii] ^= input_w[ii];
			}
			output_w += 32;
			input_w += 32;
#endif
			bytes -= 128;
		}
#ifdef CAT_HAS_VECTOR_EXTENSIONS
	} else {
		vec_block * CAT_RESTRICT in_block = (vec_block * CAT_RESTRICT)input_w;
		vec_block * CAT_RESTRICT out_block = (vec_block * CAT_RESTRICT)output_w;

		while (bytes >= 128)
		{
			*out_block++ ^= *in_block++;
			bytes -= 128;
		}

		output_w = (uint64_t * CAT_RESTRICT)out_block;
		input_w = (uint64_t * CAT_RESTRICT)in_block;
	}
#endif

	// Handle remaining multiples of 8 bytes
	while (bytes >= 8)
	{
#ifdef CAT_WORD_64
		*output_w++ ^= *input_w++;
#else
		output_w[0] ^= input_w[0];
		output_w[1] ^= input_w[1];
		output_w += 2;
		input_w += 2;
#endif
		bytes -= 8;
	}

	// Handle final <8 bytes
	uint8_t * CAT_RESTRICT output = reinterpret_cast<uint8_t *>( output_w );
	const uint8_t * CAT_RESTRICT input = reinterpret_cast<const uint8_t *>( input_w );

	switch (bytes)
	{
	case 7:	output[6] ^= input[6];
	case 6:	output[5] ^= input[5];
	case 5:	output[4] ^= input[4];
	case 4:	*(uint32_t*)output ^= *(uint32_t*)input;
		break;
	case 3:	output[2] ^= input[2];
	case 2:	output[1] ^= input[1];
	case 1:	output[0] ^= input[0];
	case 0:
	default:
		break;
	}
}

void cat::memxor_set(void * CAT_RESTRICT voutput, const void * CAT_RESTRICT va, const void * CAT_RESTRICT vb, int bytes)
{
	/*
		This version exists to avoid an expensive memory copy operation when
		an input block is being calculated from a row and some other blocks.
	*/

	// Primary engine
#ifdef CAT_WORD_64
	uint64_t * CAT_RESTRICT output_w = reinterpret_cast<uint64_t *>( voutput );
	const uint64_t * CAT_RESTRICT a_w = reinterpret_cast<const uint64_t *>( va );
	const uint64_t * CAT_RESTRICT b_w = reinterpret_cast<const uint64_t *>( vb );
#else
	uint32_t * CAT_RESTRICT output_w = reinterpret_cast<uint32_t *>( voutput );
	const uint32_t * CAT_RESTRICT a_w = reinterpret_cast<const uint32_t *>( va );
	const uint32_t * CAT_RESTRICT b_w = reinterpret_cast<const uint32_t *>( vb );
#endif

#ifdef CAT_HAS_VECTOR_EXTENSIONS
#ifdef CAT_WORD_64
	if ((*(uint64_t*)&output_w | *(uint64_t*)&a_w | *(uint64_t*)&b_w) & 15) {
#else
	if ((*(uint32_t*)&output_w | *(uint32_t*)&a_w | *(uint32_t*)&b_w) & 15) {
#endif
#endif
		while (bytes >= 128)
		{
#ifdef CAT_WORD_64
			for (int ii = 0; ii < 16; ++ii) {
				output_w[ii] = a_w[ii] ^ b_w[ii];
			}
			output_w += 16;
			a_w += 16;
			b_w += 16;
#else
			for (int ii = 0; ii < 32; ++ii) {
				output_w[ii] = a_w[ii] ^ b_w[ii];
			}
			output_w += 32;
			a_w += 32;
			b_w += 32;
#endif
			bytes -= 128;
		}
#ifdef CAT_HAS_VECTOR_EXTENSIONS
	} else {
		vec_block * CAT_RESTRICT in_block1 = (vec_block * CAT_RESTRICT)a_w;
		vec_block * CAT_RESTRICT in_block2 = (vec_block * CAT_RESTRICT)b_w;
		vec_block * CAT_RESTRICT out_block = (vec_block * CAT_RESTRICT)output_w;

		while (bytes >= 128)
		{
			*out_block++ = *in_block1++ ^ *in_block2++;
			bytes -= 128;
		}

		output_w = (uint64_t * CAT_RESTRICT)out_block;
		a_w = (uint64_t * CAT_RESTRICT)in_block1;
		b_w = (uint64_t * CAT_RESTRICT)in_block2;
	}
#endif

	// Handle remaining multiples of 8 bytes
	while (bytes >= 8)
	{
#ifdef CAT_WORD_64
		*output_w++ = *a_w++ ^ *b_w++;
#else
		output_w[0] = a_w[0] ^ b_w[0];
		output_w[1] = a_w[1] ^ b_w[1];
		output_w += 2;
		a_w += 2;
		b_w += 2;
#endif
		bytes -= 8;
	}

	// Handle final <8 bytes
	uint8_t * CAT_RESTRICT output = reinterpret_cast<uint8_t *>( output_w );
	const uint8_t * CAT_RESTRICT a = reinterpret_cast<const uint8_t *>( a_w );
	const uint8_t * CAT_RESTRICT b = reinterpret_cast<const uint8_t *>( b_w );

	switch (bytes)
	{
	case 7:	output[6] = a[6] ^ b[6];
	case 6:	output[5] = a[5] ^ b[5];
	case 5:	output[4] = a[4] ^ b[4];
	case 4:	*(uint32_t*)output = *(uint32_t*)a ^ *(uint32_t*)b;
		break;
	case 3:	output[2] = a[2] ^ b[2];
	case 2:	output[1] = a[1] ^ b[1];
	case 1:	output[0] = a[0] ^ b[0];
	case 0:
	default:
		break;
	}
}

void cat::memxor_add(void * CAT_RESTRICT voutput, const void * CAT_RESTRICT va, const void * CAT_RESTRICT vb, int bytes)
{
	/*
		This version adds to the output instead of overwriting it.
	*/

	// Primary engine
#ifdef CAT_WORD_64
	uint64_t * CAT_RESTRICT output_w = reinterpret_cast<uint64_t *>( voutput );
	const uint64_t * CAT_RESTRICT a_w = reinterpret_cast<const uint64_t *>( va );
	const uint64_t * CAT_RESTRICT b_w = reinterpret_cast<const uint64_t *>( vb );
#else
	uint32_t * CAT_RESTRICT output_w = reinterpret_cast<uint32_t *>( voutput );
	const uint32_t * CAT_RESTRICT a_w = reinterpret_cast<const uint32_t *>( va );
	const uint32_t * CAT_RESTRICT b_w = reinterpret_cast<const uint32_t *>( vb );
#endif

#ifdef CAT_HAS_VECTOR_EXTENSIONS
#ifdef CAT_WORD_64
	if ((*(uint64_t*)&output_w | *(uint64_t*)&a_w | *(uint64_t*)&b_w) & 15) {
#else
	if ((*(uint32_t*)&output_w | *(uint32_t*)&a_w | *(uint32_t*)&b_w) & 15) {
#endif
#endif
		while (bytes >= 128)
		{
#ifdef CAT_WORD_64
			for (int ii = 0; ii < 16; ++ii) {
				output_w[ii] ^= a_w[ii] ^ b_w[ii];
			}
			output_w += 16;
			a_w += 16;
			b_w += 16;
#else
			for (int ii = 0; ii < 32; ++ii) {
				output_w[ii] ^= a_w[ii] ^ b_w[ii];
			}
			output_w += 32;
			a_w += 32;
			b_w += 32;
#endif
			bytes -= 128;
		}
#ifdef CAT_HAS_VECTOR_EXTENSIONS
	} else {
		vec_block * CAT_RESTRICT in_block1 = (vec_block * CAT_RESTRICT)a_w;
		vec_block * CAT_RESTRICT in_block2 = (vec_block * CAT_RESTRICT)b_w;
		vec_block * CAT_RESTRICT out_block = (vec_block * CAT_RESTRICT)output_w;

		while (bytes >= 128)
		{
			*out_block++ ^= *in_block1++ ^ *in_block2++;
			bytes -= 128;
		}

		output_w = (uint64_t * CAT_RESTRICT)out_block;
		a_w = (uint64_t * CAT_RESTRICT)in_block1;
		b_w = (uint64_t * CAT_RESTRICT)in_block2;
	}
#endif

	// Handle remaining multiples of 8 bytes
	while (bytes >= 8)
	{
#ifdef CAT_WORD_64
		*output_w++ ^= *a_w++ ^ *b_w++;
#else
		output_w[0] ^= a_w[0] ^ b_w[0];
		output_w[1] ^= a_w[1] ^ b_w[1];
		output_w += 2;
		a_w += 2;
		b_w += 2;
#endif
		bytes -= 8;
	}

	// Handle final <8 bytes
	uint8_t * CAT_RESTRICT output = reinterpret_cast<uint8_t *>( output_w );
	const uint8_t * CAT_RESTRICT a = reinterpret_cast<const uint8_t *>( a_w );
	const uint8_t * CAT_RESTRICT b = reinterpret_cast<const uint8_t *>( b_w );

	switch (bytes)
	{
	case 7:	output[6] ^= a[6] ^ b[6];
	case 6:	output[5] ^= a[5] ^ b[5];
	case 5:	output[4] ^= a[4] ^ b[4];
	case 4:	*(uint32_t*)output ^= *(uint32_t*)a ^ *(uint32_t*)b;
		break;
	case 3:	output[2] ^= a[2] ^ b[2];
	case 2:	output[1] ^= a[1] ^ b[1];
	case 1:	output[0] ^= a[0] ^ b[0];
	case 0:
	default:
		break;
	}
}

