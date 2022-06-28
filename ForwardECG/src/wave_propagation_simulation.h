#pragma once
#include "mesh_plot.h"
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "math.h"
#include <memory>
#include "input.h"
#include "camera.h"


using namespace Eigen;


class WavePropagationSimulation;

class WavePropagationOperator
{
public:
	WavePropagationOperator(WavePropagationSimulation* prop_sim, const std::string& type);
	virtual ~WavePropagationOperator() = default;

	virtual void apply_reset();
	virtual void apply_links();

	virtual void render();
	virtual void render_gui();
	virtual void handle_input(const LookAtCamera& camera);

	virtual std::string get_type() const;

protected:
	WavePropagationSimulation* m_prop_sim;

private:
	std::string m_type;

};

class WavePropagationPlaneCut: public WavePropagationOperator
{
public:
	WavePropagationPlaneCut(WavePropagationSimulation* prop_sim, const Vector3<Real>& point = {0, 0, 0}, const Vector3<Real>& normal = {0, 0, 1});
	virtual ~WavePropagationPlaneCut() = default;

	virtual void apply_links() override;

	virtual void render() override;
	virtual void render_gui() override;
	virtual void handle_input(const LookAtCamera& camera) override;

private:
	Vector3<Real> m_point;
	Vector3<Real> m_normal;
	std::vector<glm::vec3> m_cut_links_lines;

};

class CircularBrush
{
public:
	CircularBrush(bool enable_drawing = false, Real brush_radius = 0.1, bool only_vertices_facing_camera = true);
	~CircularBrush() = default;
	
	void render();
	void render_gui();
	void handle_input(const LookAtCamera& camera, MeshPlot* mesh, const Vector3<Real>& mesh_pos = {0, 0, 0});
	const std::vector<bool>& get_intersected() const;

private:
	std::vector<bool> m_intersected;
	bool m_enable_drawing;
	Real m_brush_radius;
	bool m_only_vertices_facing_camera;
	// preview
	bool m_drawing_is_intersected;
	std::vector<glm::vec3> m_drawing_values_preview;
	int m_drawing_values_points_count = 32;
};

class WavePropagationForceDepolarization : public WavePropagationOperator
{
public:
	WavePropagationForceDepolarization(WavePropagationSimulation* prop_sim, Real depolarization_time = 0.1);
	virtual ~WavePropagationForceDepolarization() = default;

	virtual void apply_reset();

	virtual void render() override;
	virtual void render_gui() override;
	virtual void handle_input(const LookAtCamera& camera) override;

private:
	std::vector<bool> m_selected;
	Real m_depolarization_time;
	CircularBrush m_brush;
	bool m_brush_select = false;
	bool m_view_drawing = false;
};

class WavePropagationLinkTwoGroups : public WavePropagationOperator
{
public:
	WavePropagationLinkTwoGroups(WavePropagationSimulation* prop_sim);
	virtual ~WavePropagationLinkTwoGroups() = default;

	virtual void apply_links() override;

	virtual void render() override;
	virtual void render_gui() override;
	virtual void handle_input(const LookAtCamera& camera) override;

private:
	Real m_multiply_speed = 1; // scaler multiplied to the base speed
	Real m_constant_speed = 0; // constant speed added to the base speed
	Real m_constant_delay = 0; // constant delay added to the links between the two groups
	int m_selected_group = 0; // 0 = group A, 1 = group B
	std::vector<bool> m_group_a_selected;
	std::vector<bool> m_group_b_selected;
	CircularBrush m_brush;
	bool m_brush_select = false;
	bool m_view_drawing = false;
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
	void handle_input(const LookAtCamera& camera);

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
		Real constant_delay; // constant delay added to the links between the two groups
	};

	MeshPlot* m_mesh = nullptr;
	Vector3<Real> m_mesh_pos = { 0, 0, 0 };
	Real m_t; // simulation time
	Real m_duration = 1; // simulation duration
	Real m_dt = 0.001; // simulation time step
	int m_sample_count;
	int m_sample = 0; // current sample
	Real m_base_speed = 2;
	Real m_depolarization_duration = 0.250;
	Real m_depolarization_slope_duration = 0.050;
	Real m_repolarization_slope_duration = 0.200;
	std::vector<VertexLink> m_links; // vertex links
	std::vector<VertexVars> m_vars; // vertex vars
	VectorX<Real> m_potentials;
	std::vector<std::shared_ptr<WavePropagationOperator>> m_operators;
	std::vector<bool> m_operators_enable;
	std::vector<bool> m_operators_render;

	// gui
	int m_selected_operator = -1;

	friend class WavePropagationOperator;
	friend class WavePropagationPlaneCut;
	friend class WavePropagationForceDepolarization;
	friend class WavePropagationLinkTwoGroups;

};

