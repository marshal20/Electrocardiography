#pragma once
#include "math.h"
#include "mesh_plot.h"

struct Probe
{
	int triangle_idx;
	Eigen::Vector3<Real> point;

	std::string name;
};

Real evaluate_probe(const MeshPlot& mesh, const Probe& probe);

bool import_probes(const std::string& file_name, std::vector<Probe>& probes);
bool export_probes(const std::string& file_name, const std::vector<Probe>& probes);

