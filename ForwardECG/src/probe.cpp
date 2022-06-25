#include "probe.h"
#include "geometry.h"


Real evaluate_probe(const MeshPlot& mesh, const Probe& probe)
{
	// check that tringle index exists
	if (probe.triangle_idx >= mesh.faces.size())
	{
		return 0;
	}

	const MeshPlotFace& face = mesh.faces[probe.triangle_idx];
	Triangle tri = { glm2eigen(mesh.vertices[face.idx[0]].pos),
					 glm2eigen(mesh.vertices[face.idx[1]].pos),
					 glm2eigen(mesh.vertices[face.idx[2]].pos) };

	if (is_point_in_triangle(tri, probe.point))
	{
		Real scaler_a = perpendicular_distance(tri.b, tri.c, probe.point);
		Real scaler_b = perpendicular_distance(tri.a, tri.c, probe.point);
		Real scaler_c = perpendicular_distance(tri.a, tri.b, probe.point);
		Real total = scaler_a+scaler_b+scaler_c;
		scaler_a /= total;
		scaler_b /= total;
		scaler_c /= total;

		return scaler_a*mesh.vertices[face.idx[0]].value + scaler_b*mesh.vertices[face.idx[1]].value + scaler_c*mesh.vertices[face.idx[2]].value;
	}

	return 0;
}


struct ProbeSerialized
{
	int tri;
	double px, py, pz;
	char name[16];
};

bool import_probes(const std::string& file_name, std::vector<Probe>& probes)
{
	FILE* file = fopen(file_name.c_str(), "rb");
	if (!file)
	{
		return false;
	}

	std::vector<Probe> new_probes;

	// read probes count
	int probes_count = 0;
	fread(&probes_count, sizeof(probes_count), 1, file);

	// read probes
	ProbeSerialized serialized_probe;
	new_probes.reserve(probes_count);
	for (int i = 0; i < probes_count; i++)
	{
		fread(&serialized_probe, sizeof(serialized_probe), 1, file);
		new_probes.push_back({ serialized_probe.tri, {serialized_probe.px, serialized_probe.py, serialized_probe.pz}, serialized_probe.name });
	}

	// assign imported probes
	probes = new_probes;

	fclose(file);
	return true;
}

bool export_probes(const std::string& file_name, const std::vector<Probe>& probes)
{
	FILE* file = fopen(file_name.c_str(), "wb");
	if (!file)
	{
		return false;
	}

	// write probes count
	const int probes_count = probes.size();
	fwrite(&probes_count, sizeof(probes_count), 1, file);

	// write probes
	ProbeSerialized serialized_probe;
	for (int i = 0; i < probes_count; i++)
	{
		const Probe& probe = probes[i];
		serialized_probe = { probe.triangle_idx, probe.point.x(), probe.point.y(), probe.point.z() };
		strncpy(serialized_probe.name, probe.name.c_str(), sizeof(serialized_probe.name)-1);
		serialized_probe.name[sizeof(serialized_probe.name)-1] = '\0';

		fwrite(&serialized_probe, sizeof(serialized_probe), 1, file);
	}

	fclose(file);
	return true;
}
