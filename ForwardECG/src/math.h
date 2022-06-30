#pragma once
//#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glm/glm.hpp>


#define PI 3.14159265359

typedef double Real;


Real rclamp(Real val, Real min, Real max);
Real rmax(Real a, Real b);
Real rmin(Real a, Real b);
Real rabs(Real val);
Eigen::Vector3<Real> glm2eigen(const glm::vec3& v3);
glm::vec3 eigen2glm(const Eigen::Vector3<Real>& v3);


template<typename T>
void swap(T& a, T& b)
{
	T temp = a;
	a = b;
	b = temp;
}

template<typename T>
T clamp_value(T value, T min, T max)
{
	if (value < min)
	{
		return min;
	}
	else if (value > max)
	{
		return max;
	}

	return value;
}

// maps a value between range 1 [min1 : max1] to range 2 [min2 : max2]
template<typename T>
T map_value_to_range(T value, T min1, T max1, T min2, T max2)
{
	T t = (value-min1)/(max1-min1);
	return min2 + t*(max2-min2);
}


Eigen::Vector3<Real> rodrigues_rotate(const Eigen::Vector3<Real>& v, const Eigen::Vector3<Real>& axis, Real theta);

// returns a 3rd order transition between 0 and 1, for t [0:1] 
Real s_3rd_order_curve_transition(Real t);

// returns a 2nd order bump between 0 and 1, for t [0:1] 
Real bump_2nd_order(Real t);

