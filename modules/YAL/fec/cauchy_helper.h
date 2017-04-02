#pragma once

#include <stdint.h>

#define CAT_ROL64(n, r) ( (uint64_t)((uint64_t)(n) << (r)) | (uint64_t)((uint64_t)(n) >> (64 - (r))) ) /* only works for u64 */
#define CAT_ROR64(n, r) ( (uint64_t)((uint64_t)(n) >> (r)) | (uint64_t)((uint64_t)(n) << (64 - (r))) ) /* only works for u64 */

#define CAT_RESTRICT