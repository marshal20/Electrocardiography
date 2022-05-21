#include "wave_propagation_simulation.h"
#include "imgui/imgui.h"
#include "imgui/imgui_my_types.h"


#define POTENTIAL_REST -0.08
#define POTENTIAL_PEAK 0.015


struct ActionPotentialParameters
{
	Real resting_potential;
	Real peak_potential;
	Real depolarization_time;
	Real repolarization_time;
};

static Real action_potential_value_2(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.05, Real repolarization_slope_duration = 0.1);


void WavePropagationSimulation::set_mesh(MeshPlot * mesh)
{
	m_mesh = mesh;

	m_vars.resize(m_mesh->vertices.size());
	m_links.resize(m_mesh->vertices.size());
	m_potentials.resize(m_mesh->vertices.size());
	recalculate_links();
	reset();
}

void WavePropagationSimulation::reset()
{
	m_sample = 0;

	// reset to initial values
	for (int i = 0; i < m_mesh->vertices.size(); i++)
	{
		m_vars[i] = { false, 0 };
	}

	// apply force depolarization
	// (TODO: add with operator)
	m_vars[0].is_depolarized = true;
	m_vars[0].depolarization_time = 0.1;

}

int WavePropagationSimulation::get_sample_count()
{
	return m_sample_count;
}

int WavePropagationSimulation::get_current_sample()
{
	return m_sample;
}

const VectorX<Real>& WavePropagationSimulation::get_potentials() const
{
	return m_potentials;
}

void WavePropagationSimulation::simulation_step()
{
	// recalculate sample count
	m_sample_count = m_duration/m_dt;

	// go to next sample
	m_sample++;
	m_sample %= m_sample_count;
	if (m_sample == 0)
	{
		reset();
	}

	// calculate t
	m_t = m_dt*m_sample;

	// propagate the depolarization wave
	for (const VertexLink& link : m_links)
	{
		Real distance = glm2eigen(m_mesh->vertices[link.v1_idx].pos - m_mesh->vertices[link.v2_idx].pos).norm();
		Real link_speed = m_base_speed*link.multiply_speed + link.constant_speed;
		Real link_lag = distance/link_speed;

		if (m_vars[link.v1_idx].is_depolarized && !m_vars[link.v2_idx].is_depolarized)
		{
			// propagate depolarization to v2
			if (m_t >= (m_vars[link.v1_idx].depolarization_time + link_lag))
			{
				m_vars[link.v2_idx].is_depolarized = true;
				m_vars[link.v2_idx].depolarization_time = m_vars[link.v1_idx].depolarization_time + link_lag;
			}
		}

		if (m_vars[link.v2_idx].is_depolarized && !m_vars[link.v1_idx].is_depolarized)
		{
			// propagate depolarization to v1
			if (m_t >= (m_vars[link.v2_idx].depolarization_time + link_lag))
			{
				m_vars[link.v1_idx].is_depolarized = true;
				m_vars[link.v1_idx].depolarization_time = m_vars[link.v2_idx].depolarization_time + link_lag;
			}
		}
	}

	// update potentials
	for (int i = 0; i < m_mesh->vertices.size(); i++)
	{
		if (m_vars[i].is_depolarized && m_t > m_vars[i].depolarization_time)
		{
			m_potentials(i) = action_potential_value_2(m_t, { POTENTIAL_REST, POTENTIAL_PEAK, m_vars[i].depolarization_time, m_vars[i].depolarization_time+m_depolarization_duration }, m_depolarization_slope_duration, m_repolarization_slope_duration);
		}
		else
		{
			m_potentials(i) = POTENTIAL_REST;
		}
	}

}

/*
void WavePropagationSimulation::update_mesh_values()
{
	// update mesh values
	for (int i = 0; i < m_mesh->vertices.size(); i++)
	{
		m_mesh->vertices[i].value = potentials(i);
	}
}
*/

void WavePropagationSimulation::render()
{


	// maybe render selected plane
	// [TODO]

}

void WavePropagationSimulation::render_gui()
{
	ImGui::Text("Time: %.4f s, Sample: %d", m_t, m_sample);
	ImGui::InputReal("Wave Simulation Duration", &m_duration);
	ImGui::InputReal("Time Simulation Step", &m_dt, 0.001, 0.01, "%.5f");
	m_dt = clamp_value<Real>(m_dt, 0.000001, 5);

	ImGui::InputReal("Wave Propagation Speed", &m_base_speed);
	ImGui::InputReal("Wave Depolarization Duration", &m_depolarization_duration);
	ImGui::InputReal("Depolarization Slope Duration", &m_depolarization_slope_duration);
	ImGui::InputReal("Repolarization Slope Duration", &m_repolarization_slope_duration);
	if (ImGui::Button("Recalculate Links"))
	{
		recalculate_links();
	}
	if (ImGui::Button("Reset Wave"))
	{
		reset();
	}

}

void WavePropagationSimulation::recalculate_links()
{
	m_links.clear();

	// add links for vertices connected together with faces
	for (const MeshPlotFace& face : m_mesh->faces)
	{
		m_links.push_back({ face.idx[0], face.idx[1], 1, 0 });
		m_links.push_back({ face.idx[0], face.idx[2], 1, 0 });
		m_links.push_back({ face.idx[1], face.idx[2], 1, 0 });
	}
	
	// apply operators
	// [TODO]
}


// returns a 3rd order transition between 0 and 1, for t [0:1] 
static Real s_3rd_order_curve_transition(Real t)
{
	return (0<t) * 0
		+ (0<=t&&t<0.5) * 0.5*pow((t*2), 3)
		+ (0.5<=t&&t<1) * (1-0.5*pow((2-t*2), 3))
		+ (1<=t) * 1;
}

static Real action_potential_value_2(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration, Real repolarization_slope_duration)
{
	Real mixing_percentage = 0;

	if (t < params.depolarization_time)
	{
		mixing_percentage = 0;
	}
	else if (t < params.depolarization_time+depolarization_slope_duration)
	{
		mixing_percentage = s_3rd_order_curve_transition((t-params.depolarization_time)/depolarization_slope_duration);
	}
	else if (params.depolarization_time+depolarization_slope_duration <= t && t <= params.repolarization_time)
	{
		mixing_percentage = 1;
	}
	else if (t > params.repolarization_time && t <= params.repolarization_time+repolarization_slope_duration)
	{
		mixing_percentage = 1-s_3rd_order_curve_transition((t-params.repolarization_time)/repolarization_slope_duration);
	}
	else
	{
		mixing_percentage = 0;
	}

	return params.resting_potential + (params.peak_potential-params.resting_potential)*mixing_percentage;
}
