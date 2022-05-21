#pragma once
#include "mesh_plot.h"
#include <vector>
#include <Eigen/Dense>
#include "math.h"


using namespace Eigen;


class WavePropagationSimulation
{
public:
	WavePropagationSimulation() = default;
	~WavePropagationSimulation() = default;

	void set_mesh(MeshPlot* mesh);
	void reset();

	int get_sample_count();
	int get_current_sample();
	const VectorX<Real>& get_potentials() const;
	void simulation_step();
	//void update_mesh_values();
	void render();
	void render_gui();

private:
	void recalculate_links();

private:
	// vertex variables
	struct VertexVars
	{
		bool is_depolarized;
		Real depolarization_time;
	};

	// vertex link
	struct VertexLink
	{
		int v1_idx; // first vertex index
		int v2_idx; // second vertex index
		Real multiply_speed; // scaler multiplied to the base speed
		Real constant_speed; // constant speed added to the base speed
	};

	MeshPlot* m_mesh = nullptr;
	Real m_t; // simulation time
	Real m_duration = 1; // simulation duration
	Real m_dt = 0.001; // simulation time step
	int m_sample_count;
	int m_sample = 0; // current sample
	Real m_base_speed = 2;
	Real m_depolarization_duration = 0.5;
	Real m_depolarization_slope_duration = 0.050;
	Real m_repolarization_slope_duration = 0.200;
	std::vector<VertexLink> m_links; // vertex links
	std::vector<VertexVars> m_vars; // vertex vars
	VectorX<Real> m_potentials;
};

