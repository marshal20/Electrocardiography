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

// returns a 3rd order transition between 0 and 1, for t [0:1] 
Real s_3rd_order_curve_transition(Real t)
{
	return (0<t) * 0
		+ (0<=t&&t<0.5) * 0.5*pow((t*2), 3)
		+ (0.5<=t&&t<1) * (1-0.5*pow((2-t*2), 3))
		+ (1<=t) * 1;
}

// returns a 2nd order bump between 0 and 1, for t [0:1] 
Real bump_2nd_order(Real t)
{
	return (0<t) * 0
		+ (0<=t&&t<1) * (1-pow((t*2-1), 2))
		+ (1<=t) * 0;
}
