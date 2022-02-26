#include "math.h"


Real rclamp(Real val, Real min, Real max)
{
	if (val > max)
	{
		return max;
	}
	else if (val < min)
	{
		return min;
	}

	return val;
}

Real rmax(Real a, Real b)
{
	if (a > b)
	{
		return a;
	}

	return b;
}

Real rabs(Real val)
{
	if (val < 0)
	{
		return -val;
	}

	return val;
}

Eigen::Vector3<Real> glm2eigen(const glm::vec3& v3)
{
	return { v3.x, v3.y, v3.z };
}
