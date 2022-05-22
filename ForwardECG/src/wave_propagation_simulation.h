#pragma once
#include "mesh_plot.h"
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "math.h"
#include <memory>


using namespace Eigen;


class WavePropagationSimulation;

class WavePropagationOperator
{
public:
	WavePropagationOperator(const std::string& type);
	virtual ~WavePropagationOperator() = default;

	virtual void apply_reset(WavePropagationSimulation& prop_sim);
	virtual void apply_links(WavePropagationSimulation& prop_sim);

	virtual void render();
	virtual void render_gui();
	virtual void handle_input();

private:
	std::string m_type;

};

class WavePropagationPlaneCut: public WavePropagationOperator
{
public:
	WavePropagationPlaneCut(const Vector3<Real>& point = {0, 0, 0}, const Vector3<Real>& normal = {0, 0, 1});
	virtual ~WavePropagationPlaneCut() = default;

	virtual void apply_links(WavePropagationSimulation& prop_sim) override;

	//virtual void render() override;
	//virtual void render_gui() override;
	//virtual void handle_input() override;

private:
	Vector3<Real> m_point;
	Vector3<Real> m_normal;

};


class WavePropagationSimulation
{
public:
	WavePropagationSimulation() = default;
	~WavePropagationSimulation() = default;

	void set_mesh(MeshPlot* mesh, const Eigen::Vector3<Real>& mesh_pos = {0, 0, 0});
	void set_mesh_pos(const Eigen::Vector3<Real>& mesh_pos);
	void reset();

	int get_sample_count();
	int get_current_sample();
	const VectorX<Real>& get_potentials() const;
	void simulation_step();
	//void update_mesh_values();
	void render();
	void render_gui();
	void handle_input();

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
	Vector3<Real> m_mesh_pos = { 0, 0, 0 };
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
	std::vector<std::shared_ptr<WavePropagationOperator>> m_operators;

	friend class WavePropagationOperator;
	friend class WavePropagationPlaneCut;

};

