#include "geometry.h"


using namespace Eigen;


// Checks if a point (p) is inside a triangle (a, b, c).
bool is_point_in_triangle(const Triangle& triangle, const Eigen::Vector3<Real>& p)
{
	Eigen::Vector3<Real> u = triangle.b - triangle.a;
	Eigen::Vector3<Real> v = triangle.c - triangle.a;
	Eigen::Vector3<Real> w = p - triangle.a;

	float uu = u.dot(u);
	float uv = u.dot(v);
	float vv = v.dot(v);
	float wu = w.dot(u);
	float wv = w.dot(v);
	float dominator = uv * uv - uu * vv;

	float s = (uv * wv - vv * wu) / dominator;
	float t = (uv * wu - uu * wv) / dominator;
	if (s >= 0.0 && t >= 0.0 && (s + t) <= 1.0)
		return true;

	return false;
}

Real perpendicular_distance(const Eigen::Vector3<Real>& a, const Eigen::Vector3<Real>& b, const Eigen::Vector3<Real>& point)
{
	Eigen::Vector3<Real> ab_norm = (b-a).normalized();
	Real ab_dot = (point-a).dot(ab_norm);
	Real pointa = (point-a).norm();
	return sqrt(pointa*pointa - ab_dot*ab_dot);
}


Eigen::Vector3<Real> Ray::point_at_dir(const Real& scaler) const
{
	return origin + scaler*direction;
}

// Checks if a ray (r) intersects a plane (p, n).
Real Ray::intersect_plane(const Eigen::Vector3<Real>& p, const Eigen::Vector3<Real>& n) const
{
	float RHS = n.dot(p) - n.dot(origin);
	float LHS = direction.dot(n);
	return RHS / LHS;
}

bool Ray::intersect_triangle(const Triangle& triangle, Real& t) const
{
	Eigen::Vector3<Real> u = triangle.b - triangle.a;
	Eigen::Vector3<Real> v = triangle.c - triangle.a;
	Eigen::Vector3<Real> n = u.cross(v).normalized();
	t = intersect_plane(triangle.a, n);
	if (is_point_in_triangle(triangle, point_at_dir(t)))
	{
		return true;
	}
	return false;
}


Eigen::Vector2<Real> get_point_in_triangle_basis(const Triangle& triangle, const Eigen::Vector3<Real>& p)
{
	Eigen::Vector3<Real> u = triangle.b - triangle.a;
	Eigen::Vector3<Real> v = triangle.c - triangle.a;
	Eigen::Vector3<Real> w = p - triangle.a;

	float uu = u.dot(u);
	float uv = u.dot(v);
	float vv = v.dot(v);
	float wu = w.dot(u);
	float wv = w.dot(v);
	float dominator = uv * uv - uu * vv;

	float s = (uv * wv - vv * wu) / dominator;
	float t = (uv * wu - uu * wv) / dominator;

	return { s, t };
}

bool ray_mesh_intersect(const MeshPlot& mesh, const Eigen::Vector3<Real>& mesh_position, const Ray& ray, Real& t, int& tri_idx)
{
	bool intersected = false;
	Real min_t = INFINITY;

	for (int i = 0; i < mesh.faces.size(); i++)
	{
		const MeshPlotFace& face = mesh.faces[i];
		Triangle tri = { mesh_position + glm2eigen(mesh.vertices[face.idx[0]].pos),
						 mesh_position + glm2eigen(mesh.vertices[face.idx[1]].pos),
						 mesh_position + glm2eigen(mesh.vertices[face.idx[2]].pos) };
		Real new_t;
		if (ray.intersect_triangle(tri, new_t))
		{
			intersected = true;
			if (new_t < min_t)
			{
				min_t = new_t;
				tri_idx = i;
			}
		}
	}

	t = min_t;
	return intersected;
}

bool line_plane_intersect(const Eigen::Vector3<Real>& v1, const Eigen::Vector3<Real>& v2, const Eigen::Vector3<Real>& p, const Eigen::Vector3<Real>& n, Real& t)
{
	Eigen::Vector3<Real> line_direction = (v2-v1).normalized();
	float RHS = n.dot(p) - n.dot(v1); // n.p - n.origin
	float LHS = line_direction.dot(n); // direction . n
	t = RHS / LHS;

	return t <= (v1-v2).norm();
}

bool is_line_plane_intersect(const Eigen::Vector3<Real>& v1, const Eigen::Vector3<Real>& v2, const Eigen::Vector3<Real>& p, const Eigen::Vector3<Real>& n)
{
	Real d1 = (v1-p).dot(n);
	Real d2 = (v2-p).dot(n);
	return  (d1 < 0 && d2 > 0) || (d1 > 0 && d2 < 0);
}

Ray camera_screen_to_world_ray(const LookAtCamera & camera, Real x_norm, Real y_norm)
{
	// camera axis
	Vector3<Real> forward = glm2eigen(camera.look_at-camera.eye).normalized();
	Vector3<Real> up = glm2eigen(camera.up).normalized();
	Vector3<Real> right = forward.cross(up).normalized();
	// calculate the pointer direction
	Vector3<Real> direction = forward + up*tan(0.5*y_norm*camera.fov) + right*tan(0.5*x_norm*camera.aspect*camera.fov);
	direction = direction.normalized();

	return Ray{ glm2eigen(camera.eye), direction };

}

