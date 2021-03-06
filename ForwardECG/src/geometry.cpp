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

Real Ray::point_perpendicular_distance(const Eigen::Vector3<Real>& p) const
{
	Real ab_dot = (p-origin).dot(direction.normalized());
	Real pointa = (p-origin).norm();
	return sqrt(pointa*pointa - ab_dot*ab_dot);
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

bool ray_mesh_intersect(const MeshPlot& mesh, const Eigen::Vector3<Real>& mesh_position, const Ray& ray, Real& t, int& tri_idx, int group_index)
{
	bool intersected = false;
	Real min_t = INFINITY;
	tri_idx = -1;

	for (int i = 0; i < mesh.faces.size(); i++)
	{
		const MeshPlotFace& face = mesh.faces[i];

		// skip if no vertex is from the selected group
		if (group_index != -1
			&& mesh.vertices[face.idx[0]].group != group_index
			&& mesh.vertices[face.idx[1]].group != group_index
			&& mesh.vertices[face.idx[2]].group != group_index)
		{
			continue;
		}

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

Eigen::Vector3<Real> calculate_triangle_normal(MeshPlot* mesh, int tri_idx)
{
	Vector3<Real> a = glm2eigen(mesh->vertices[mesh->faces[tri_idx].idx[0]].pos);
	Vector3<Real> b = glm2eigen(mesh->vertices[mesh->faces[tri_idx].idx[1]].pos);
	Vector3<Real> c = glm2eigen(mesh->vertices[mesh->faces[tri_idx].idx[2]].pos);
	return (b-a).cross(c-a).normalized();
}



std::vector<Probe> cast_probes_in_sphere(const std::string& prefix, const MeshPlot& mesh, int rows, int cols, Real z_rot, const Eigen::Vector3<Real>& rays_origin, int group_index, bool cast_from_outside)
{
	std::vector<Probe> probes;

	// spherical coordinates to cartisian
	for (int theta_i = 0; theta_i < rows; theta_i++)
	{
		Real theta = map_value_to_range<Real>(((Real)theta_i+0.5), 0, rows, -PI/2, PI/2);
		for (int phi_i = 0; phi_i < cols; phi_i++)
		{
			Real phi = map_value_to_range<Real>(((Real)phi_i+0.5), 0, cols, 0, 2*PI);
			// calculate ray
			Vector3<Real> ray_direction = { cos(theta)*cos(phi), sin(theta), cos(theta)*sin(phi) };
			ray_direction = rodrigues_rotate(ray_direction, { 0, 0, 1 }, z_rot); // apply rotation
			ray_direction.normalize();
			Ray cast_ray = { rays_origin, ray_direction };

			// cast from the outside
			if (cast_from_outside)
			{
				cast_ray = { cast_ray.point_at_dir(100), -cast_ray.direction };
			}

			// intersect ray with mesh
			Real t;
			int tri_idx;
			if (ray_mesh_intersect(mesh, Vector3<Real>(0, 0, 0), cast_ray, t, tri_idx, group_index))
			{
				Vector3<Real> intersection_point = cast_ray.point_at_dir(t);
				std::string probe_name = prefix + "_" + std::to_string(theta_i) + "_" + std::to_string(phi_i);
				probes.push_back(Probe{ tri_idx, intersection_point, probe_name });
			}
		}
	}

	return probes;
}

std::vector<Probe> cast_probes_in_plane(const std::string& prefix, const MeshPlot& mesh, int rows, int cols, Real z_plane, Real z_direction, Real x_min, Real x_max, Real y_min, Real y_max, int group_index)
{
	std::vector<Probe> probes;

	// spherical coordinates to cartisian
	for (int i = 0; i < rows; i++)
	{
		Real y = map_value_to_range<Real>((Real)i, 0, rows-1, y_min, y_max);
		for (int j = 0; j < cols; j++)
		{
			Real x = map_value_to_range<Real>((Real)j, 0, cols-1, x_min, x_max);

			// calculate ray
			Ray cast_ray = { {x, y, z_plane}, {0, 0, z_direction} };

			// intersect ray with mesh
			Real t;
			int tri_idx;
			if (ray_mesh_intersect(mesh, Vector3<Real>(0, 0, 0), cast_ray, t, tri_idx, group_index))
			{
				Vector3<Real> intersection_point = cast_ray.point_at_dir(t);
				std::string probe_name = prefix + "_" + std::to_string(i) + "_" + std::to_string(j);
				probes.push_back(Probe{ tri_idx, intersection_point, probe_name });
			}
		}
	}

	return probes;
}


static void get_breadth_first_add_vertex_neighbours(const MeshPlot& mesh, int vertex_idx, std::vector<int>& vertex_sequence, std::vector<int>& vertex_parent, std::vector<bool>& visited)
{
	// check if vertex was visited before
	if (visited[vertex_idx])
	{
		return;
	}

	// set visited flag
	visited[vertex_idx] = true;

	// add neighbours
	for (int neighbour_idx : mesh.vertex_neighbour_vertices[vertex_idx])
	{
		vertex_sequence.push_back(neighbour_idx);
		vertex_parent.push_back(vertex_idx);
	}
}

static void get_breadth_first_vertex_sequence(const MeshPlot& mesh, int vertex_idx, std::vector<int>& vertex_sequence, std::vector<int>& vertex_parent, std::vector<bool>& visited)
{
	vertex_sequence.clear();
	vertex_parent.clear();
	size_t old_sequence_size = 0;

	// push the main vertex
	vertex_sequence.push_back(vertex_idx);
	vertex_parent.push_back(vertex_idx); // indicating it has no parents

	// add vertices in depth levels untill nothing left
	while (old_sequence_size != vertex_sequence.size())
	{
		size_t new_vertices = vertex_sequence.size() - old_sequence_size;
		for (int i = 0; i < new_vertices; i++)
		{
			int new_idx = vertex_sequence[old_sequence_size+i];
			get_breadth_first_add_vertex_neighbours(mesh, new_idx, vertex_sequence, vertex_parent, visited);
		}
		old_sequence_size += new_vertices;
	}

}

static void calculate_distance_from_a_vertex(const MeshPlot& mesh, int vertex_idx, const Real base_distance, std::vector<Real>& distances)
{
	// using breadth first search
	
	// get breadth first sequence
	std::vector<int> vertex_sequence;
	std::vector<int> vertex_parent;
	distances[vertex_idx] = base_distance;
	std::vector<bool> visited = std::vector<bool>(mesh.vertices.size(), false);
	get_breadth_first_vertex_sequence(mesh, vertex_idx, vertex_sequence, vertex_parent, visited);

	// calculate distance at each vertex
	for (int i = 0; i < vertex_sequence.size(); i++)
	{
		Real vertex_distance = base_distance;
		if (vertex_sequence[i] != vertex_parent[i])
		{
			vertex_distance = distances[vertex_parent[i]] + (glm2eigen(mesh.vertices[vertex_parent[i]].pos) - glm2eigen(mesh.vertices[vertex_sequence[i]].pos)).norm();
		}
		// set the distance
		distances[vertex_sequence[i]] = rmin(vertex_distance, distances[vertex_sequence[i]]);
	}

	/*
	// check if vertex was visited before
	if (visited[vertex_idx])
	{
		return;
	}

	// set visited flag
	visited[vertex_idx] = true;

	// set distance (minimum of the two if already set)
	distances[vertex_idx] = rmin(base_distance, distances[vertex_idx]);
	
	// visit neighbours
	for (int neighbour_idx : mesh.vertex_neighbour_vertices[vertex_idx])
	{
		// calculate next vertex total distance
		Real next_vertex_distance = base_distance + (glm2eigen(mesh.vertices[neighbour_idx].pos) - glm2eigen(mesh.vertices[vertex_idx].pos)).norm();
		// visit the neighbour vertex
		calculate_distance_from_a_vertex(mesh, neighbour_idx, next_vertex_distance, distances, visited);
	}
	*/
}

std::vector<Real> probe_to_vertices_distance_across_the_surface(const MeshPlot & mesh, const Probe & probe)
{
	// initialize the distances to infinity (this value will be assigned to the disconnected vertices at the end)
	std::vector<Real> distances(mesh.vertices.size(), FLT_MAX);

	// calculate distance from each vertex
	// v0
	Real d0 = (glm2eigen(mesh.vertices[mesh.faces[probe.triangle_idx].idx[0]].pos) - probe.point).norm();
	calculate_distance_from_a_vertex(mesh, mesh.faces[probe.triangle_idx].idx[0], d0, distances);
	// v1
	Real d1 = (glm2eigen(mesh.vertices[mesh.faces[probe.triangle_idx].idx[1]].pos) - probe.point).norm();
	calculate_distance_from_a_vertex(mesh, mesh.faces[probe.triangle_idx].idx[1], d1, distances);
	// v2
	Real d2 = (glm2eigen(mesh.vertices[mesh.faces[probe.triangle_idx].idx[2]].pos) - probe.point).norm();
	calculate_distance_from_a_vertex(mesh, mesh.faces[probe.triangle_idx].idx[2], d2, distances);

	return distances;
}
