#pragma once
#include <Eigen/Dense>
#include "math.h"
#include "mesh_plot.h"


struct Triangle
{
	Eigen::Vector3<Real> a, b, c;
};

// Checks if a point (p) is inside a triangle (a, b, c).
bool is_point_in_triangle(const Triangle& triangle, const Eigen::Vector3<Real>& p);

Real perpendicular_distance(const Eigen::Vector3<Real>& a, const Eigen::Vector3<Real>& b, const Eigen::Vector3<Real>& point);

struct Ray
{
	Eigen::Vector3<Real> origin;
	Eigen::Vector3<Real> direction;

	Eigen::Vector3<Real> point_at_dir(const Real& scaler) const;

	// Checks if a ray (r) intersects a plane (p, n).
	Real intersect_plane(const Eigen::Vector3<Real>& p, const Eigen::Vector3<Real>& n) const;

	bool intersect_triangle(const Triangle& triangle, Real& t) const;
};

Eigen::Vector2<Real> get_point_in_triangle_basis(const Triangle& triangle, const Eigen::Vector3<Real>& p);

bool ray_mesh_intersect(const MeshPlot& mesh, const Ray& ray, Real& t, int& tri_idx);

