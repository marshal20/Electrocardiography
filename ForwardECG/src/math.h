#pragma once
#include <Eigen/Dense>
#include <glm/glm.hpp>


#define PI 3.14159265359

typedef double Real;


Real rclamp(Real val, Real min, Real max);
Real rmax(Real a, Real b);
Real rmin(Real a, Real b);
Real rabs(Real val);
Eigen::Vector3<Real> glm2eigen(const glm::vec3& v3);
glm::vec3 eigen2glm(const Eigen::Vector3<Real>& v3);

