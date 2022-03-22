#include "random.h"
#include <time.h>

uint64_t Random::MAX_RANDOM_UINT64 = UINT64_MAX;

Random::Random()
{
	m_state = time(NULL);
}

Random::Random(uint64_t seed) :
	m_state(seed)
{
}

uint32_t Random::next_u32()
{
	uint64_t x = next_u64();
	return *(uint32_t*)(&x);
}

int32_t Random::next_i32()
{
	uint64_t x = next_u64();
	return *(int32_t*)(&x);
}

uint64_t Random::next_u64()
{
	uint64_t x = m_state;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	m_state = x;
	return x;
}

int64_t Random::next_i64()
{
	uint64_t x = next_u64();
	return *(int64_t*)(&x);
}

Real Random::next_real()
{
	return (Real)next_u64() / (Real)MAX_RANDOM_UINT64;
}

Eigen::Vector3<Real> Random::next_vector3(Real max_radius)
{
	Real radius = max_radius*sqrt(next_real()); // 0:1
	Real theta = next_real() * 2 * PI; // 0:2*PI (0:360)
	Real alpha = next_real() * PI; // 0:PI (0:180)
	Real r_xy_proj = radius * sin(alpha);
	return Eigen::Vector3<Real>(r_xy_proj * cos(theta), r_xy_proj * sin(theta), radius * cos(alpha));

}
