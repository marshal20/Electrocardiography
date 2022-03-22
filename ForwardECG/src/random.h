#pragma once
#include <stdint.h>
#include "math.h"
#include <Eigen/Dense>

// XorShift64 pseudorandom number generator
class Random
{
private:
	uint64_t m_state;

public:
	static uint64_t MAX_RANDOM_UINT64;

public:
	Random();
	Random(uint64_t seed);

	uint32_t next_u32();
	int32_t next_i32();
	uint64_t next_u64();
	int64_t next_i64();
	Real next_real(); // in range [0 -1]
	Eigen::Vector3<Real> next_vector3(Real max_radius = 1.0);
};
