#pragma once
#include "math.h"
#include "mesh_plot.h"
#include "camera.h"
#include "probe.h"


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

bool ray_mesh_intersect(const MeshPlot& mesh, const Eigen::Vector3<Real>& mesh_position, const Ray& ray, Real& t, int& tri_idx);

// checks if line (v1, v2) intersected a plane (p, n)
bool line_plane_intersect(const Eigen::Vector3<Real>& v1, const Eigen::Vector3<Real>& v2, const Eigen::Vector3<Real>& p, const Eigen::Vector3<Real>& n, Real& t);

bool is_line_plane_intersect(const Eigen::Vector3<Real>& v1, const Eigen::Vector3<Real>& v2, const Eigen::Vector3<Real>& p, const Eigen::Vector3<Real>& n);

// casts a ray from screen coordinates (x_norm, y_norm) to world coordinates
// x_norm = [-1:1]
// y_norm = [-1:1]
Ray camera_screen_to_world_ray(const LookAtCamera& camera, Real x_norm, Real y_norm);

Eigen::Vector3<Real> calculate_triangle_normal(MeshPlot* mesh, int tri_idx);

std::vector<Probe> cast_probes_in_sphere(const std::string& prefix, const MeshPlot& mesh, int rows, int cols);
std::vector<Probe> cast_probes_in_plane(const std::string& prefix, const MeshPlot& mesh, int rows, int cols, Real z_plane, Real z_direction, Real x_min, Real x_max, Real y_min, Real y_max);
