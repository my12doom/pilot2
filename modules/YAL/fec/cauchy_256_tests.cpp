#include "cauchy_256.h"

#include <iostream>
#include <iomanip>
#include <cassert>
#include <fstream>
#include <time.h>

#include "cauchy_helper.h"

using namespace std;

// timer
#include <Windows.h>
LARGE_INTEGER start;
LARGE_INTEGER fre;
void timer_init()
{
	QueryPerformanceFrequency(&fre);
	QueryPerformanceCounter(&start);
}

int64_t getus()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);

	return (li.QuadPart - start.QuadPart) * 1000000 / fre.QuadPart;
}

static void print(const void *data, int bytes) {
	const uint8_t *in = reinterpret_cast<const uint8_t *>( data );

	cout << hex;
	for (int ii = 0; ii < bytes; ++ii) {
		cout << (int)in[ii] << " ";
	}
	cout << dec << endl;
}


//#define CAT_ENCODE_TIMES_ONLY
#define CAT_WORST_CASE_BENCHMARK
#define CAT_REASONABLE_RECOVERY_COUNT

// Test to make sure that Longhair works well with input ordered like this for
// k = 4 and m = 2
// 0
// 2
// 3
// 5
// 6
void order_test() {
	int block_bytes = 8 * 162; // a multiple of 8

	srand(time(NULL));

	int block_count = 4;
	int recovery_block_count = 2;

	uint8_t *data = new uint8_t[block_bytes * block_count];
	uint8_t *recovery_blocks = new uint8_t[block_bytes * recovery_block_count];
	Block *blocks = new Block[block_count];

	const uint8_t *data_ptrs[256];
	for (int ii = 0; ii < block_count; ++ii) {
		data_ptrs[ii] = data + ii * block_bytes;
	}

	for (int ii = 0; ii < block_bytes * block_count; ++ii) {
		data[ii] = (uint8_t)rand();
	}

	assert(!cauchy_256_encode(block_count, recovery_block_count, data_ptrs, recovery_blocks, block_bytes));

	for (int ii = 0; ii < block_count; ++ii) {
		blocks[ii].data = (uint8_t*)data_ptrs[ii];
		blocks[ii].row = (uint8_t)ii;
	}

	int rem = block_count;
	for (int ii = 0; ii < recovery_block_count; ++ii) {
		int jj = (rand()&0xff) % rem;

		--rem;

		for (int kk = jj; kk < rem; ++kk) {
			blocks[kk].data = blocks[kk + 1].data;
			blocks[kk].row = blocks[kk + 1].row;
		}

		blocks[rem].data = recovery_blocks + ii * block_bytes;
		blocks[rem].row = block_count + ii;
	}

	cout << "Before decode:" << endl;
	for (int ii = 0; ii < block_count; ++ii) {
		cout << (int)blocks[ii].row << endl;
	}

	assert(!cauchy_256_decode(block_count, recovery_block_count, blocks, block_bytes));

	cout << "After decode:" << endl;
	for (int ii = 0; ii < block_count; ++ii) {
		cout << (int)blocks[ii].row << endl;
	}

	for (int ii = 0; ii < block_count; ++ii) {
		assert(!memcmp(blocks[ii].data, data_ptrs[blocks[ii].row], block_bytes));
	}
}

int main() {

	cauchy_256_init();
	timer_init();

	getus();

	cout << "Cauchy RS Codec Unit Tester" << endl;

	order_test();

	int block_bytes = 8 * 162; // a multiple of 8

	cout << "Using " << block_bytes << " bytes per block (ie. packet/chunk size); must be a multiple of 8 bytes" << endl;

	uint8_t heat_map[256 * 256] = { 0 };

	for (int block_count = 1; block_count < 256; ++block_count) {
#ifdef CAT_REASONABLE_RECOVERY_COUNT
		for (int recovery_block_count = 1; recovery_block_count <= (block_count / 2) && recovery_block_count < (256 - block_count); ++recovery_block_count) {
#else
		for (int recovery_block_count = 1; recovery_block_count < (256 - block_count); ++recovery_block_count) {
#endif
			uint8_t *data = new uint8_t[block_bytes * block_count];
			uint8_t *recovery_blocks = new uint8_t[block_bytes * recovery_block_count];
			Block *blocks = new Block[block_count];

			const uint8_t *data_ptrs[256];
			for (int ii = 0; ii < block_count; ++ii) {
				data_ptrs[ii] = data + ii * block_bytes;
			}

			double sum_encode = 0;

			int erasures_count;
#ifdef CAT_WORST_CASE_BENCHMARK
            erasures_count = recovery_block_count;
            if (block_count < erasures_count)
            {
                erasures_count = block_count;
            }
            {
#else
# ifdef CAT_ENCODE_TIMES_ONLY
			for (erasures_count = 1; erasures_count <= 4 && erasures_count <= recovery_block_count && erasures_count <= block_count; ++erasures_count) {
# else
			for (erasures_count = 1; erasures_count <= recovery_block_count && erasures_count <= block_count; ++erasures_count) {
# endif
#endif
				for (int ii = 0; ii < block_bytes * block_count; ++ii) {
					data[ii] = (uint8_t)(rand()&0xff);
				}

				double t0 = getus();

				assert(!cauchy_256_encode(block_count, recovery_block_count, data_ptrs, recovery_blocks, block_bytes));

				double t1 = getus();
				double encode_time = t1 - t0;
				sum_encode += encode_time;

				cout << "Encoded k=" << block_count << " data blocks with m=" << recovery_block_count << " recovery blocks in " << encode_time << " usec : " << (block_bytes * block_count / encode_time) << " MB/s" << endl;

#ifndef CAT_ENCODE_TIMES_ONLY
				for (int ii = 0; ii < erasures_count; ++ii) {
					int erasure_index = recovery_block_count - ii - 1;
					blocks[ii].data = recovery_blocks + erasure_index * block_bytes;
					blocks[ii].row = block_count + erasure_index;
				}

				for (int ii = erasures_count; ii < block_count; ++ii) {
					blocks[ii].data = data + ii * block_bytes;
					blocks[ii].row = ii;
				}

				t0 = getus();

				assert(!cauchy_256_decode(block_count, recovery_block_count, blocks, block_bytes));

				t1 = getus();
				double decode_time = t1 - t0;

				cout << "+ Decoded " << erasures_count << " erasures in " << decode_time << " usec : " << (block_bytes * block_count / decode_time) << " MB/s" << endl;
/*
				for (int ii = 0; ii < erasures_count; ++ii) {
					const u8 *orig = data + ii * block_bytes;
					print(orig, block_bytes / 8);
					print(blocks[ii].data, block_bytes / 8);
				}
*/
				for (int ii = 0; ii < erasures_count; ++ii) {
					const uint8_t *orig = data + ii * block_bytes;
					assert(!memcmp(blocks[ii].data, orig, block_bytes));
				}
#endif // CAT_ENCODE_TIMES_ONLY
			}

			double avg_encode = sum_encode / erasures_count;
			int speed = block_bytes * block_count / avg_encode;

			uint8_t map_value = 0;

			if (speed < 10) {
				map_value = 1;
			} else if (speed < 50) {
				map_value = 2;
			} else if (speed < 100) {
				map_value = 3;
			} else if (speed < 200) {
				map_value = 4;
			} else if (speed < 300) {
				map_value = 5;
			} else if (speed < 400) {
				map_value = 6;
			} else if (speed < 500) {
				map_value = 7;
			} else {
				map_value = 8;
			}

			heat_map[block_count * 256 + recovery_block_count] = map_value;

			delete []data;
			delete []recovery_blocks;
			delete []blocks;
		}
	}

	ofstream file;
	file.open("docs/heatmap.txt");

	for (int ii = 0; ii < 256; ++ii) {
		for (int jj = 0; jj < 256; ++jj) {
			uint8_t map_value = heat_map[ii * 256 + jj];

			file << (int)map_value << " ";
		}
		file << endl;
	}

	return 0;
}

