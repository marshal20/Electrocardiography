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

Real rmin(Real a, Real b)
{
	if (a < b)
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

glm::vec3 eigen2glm(const Eigen::Vector3<Real>& v3)
{
	return { v3.x(), v3.y(), v3.z() };
}


Eigen::Vector3<Real> rodrigues_rotate(const Eigen::Vector3<Real>& v, const Eigen::Vector3<Real>& axis, Real theta)
{
	return v*cos(theta) + axis.cross(v)*sin(theta) + axis*axis.dot(v)*(1-cos(theta));
}
