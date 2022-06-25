#include "wave_propagation_simulation.h"
#include "imgui/imgui.h"
#include "imgui/imgui_my_types.h"
#include "geometry.h"
#include "renderer3d.h"
#include "opengl/gl_headers.h"
#include "GLFW/glfw3.h"
#include "main_dev.h"


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


void WavePropagationSimulation::set_mesh(MeshPlot * mesh, const Eigen::Vector3<Real>& mesh_pos)
{
	// test operator (plane cut)
	m_operators.push_back(std::shared_ptr<WavePropagationPlaneCut>(new WavePropagationPlaneCut(this, mesh_pos + Vector3<Real>(0, 0, 0), {-1, 1, 0})));
	m_operators.push_back(std::shared_ptr<WavePropagationForceDepolarization>(new WavePropagationForceDepolarization(this, mesh->vertices.size())));

	m_mesh = mesh;
	m_mesh_pos = mesh_pos;

	m_vars.resize(m_mesh->vertices.size());
	m_links.resize(m_mesh->vertices.size());
	m_potentials.resize(m_mesh->vertices.size());
	recalculate_links();
	reset();
}

void WavePropagationSimulation::set_mesh_pos(const Eigen::Vector3<Real>& mesh_pos)
{
	m_mesh_pos = mesh_pos;
}

void WavePropagationSimulation::reset()
{
	m_sample = 0;

	// reset to initial values
	for (int i = 0; i < m_mesh->vertices.size(); i++)
	{
		m_vars[i] = { false, 0 };
	}

	// apply operators reset
	for (std::shared_ptr<WavePropagationOperator> op : m_operators)
	{
		op->apply_reset();
	}

	recalculate_links();
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
	// operators render
	if (m_selected_operator != -1 && m_selected_operator < m_operators.size())
	{
		m_operators[m_selected_operator]->render();
	}
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

	// operators list
	if (ImGui::ListBoxHeader("Operators", { 0, 120 }))
	{
		for (int i = 0; i < m_operators.size(); i++)
		{
			bool is_selected = i==m_selected_operator;
			ImGui::Selectable(std::to_string(i).c_str(), &is_selected);
			if (is_selected)
			{
				m_selected_operator = i;
			}
			
			// type
			ImGui::SameLine();
			ImGui::Text("%s", m_operators[i]->get_type().c_str());

		}
		ImGui::ListBoxFooter();
	}
	
	// operator controls
	if (m_selected_operator != -1 && m_selected_operator < m_operators.size())
	{
		bool operator_controls = true;
		ImGui::Begin("Operator Controls", &operator_controls);
		m_operators[m_selected_operator]->render_gui();
		ImGui::End();
		if (!operator_controls)
		{
			m_selected_operator = -1;
		}
	}

	
}

void WavePropagationSimulation::handle_input(const LookAtCamera& camera)
{
	// operators handle input
	if (m_selected_operator != -1 && m_selected_operator < m_operators.size())
	{
		m_operators[m_selected_operator]->handle_input(camera);
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
	for (std::shared_ptr<WavePropagationOperator> op : m_operators)
	{
		op->apply_links();
	}

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

// Wave Propagation Operator

WavePropagationOperator::WavePropagationOperator(WavePropagationSimulation* prop_sim, const std::string & type)
{
	m_type = type;
	m_prop_sim = prop_sim;
}

void WavePropagationOperator::apply_reset()
{
}

void WavePropagationOperator::apply_links()
{
}

void WavePropagationOperator::render()
{
}

void WavePropagationOperator::render_gui()
{
}

void WavePropagationOperator::handle_input(const LookAtCamera& camera)
{
}

std::string WavePropagationOperator::get_type() const
{
	return m_type;
}

// Paper Cut

WavePropagationPlaneCut::WavePropagationPlaneCut(WavePropagationSimulation* prop_sim, const Vector3<Real>& point, const Vector3<Real>& normal)
	:WavePropagationOperator(prop_sim, "Plane Cut")
{
	m_point = point;
	m_normal = normal.normalized();
}

void WavePropagationPlaneCut::apply_links()
{
	m_cut_links_lines.clear();

	//line_plane_intersect
	for (int i = 0; i < m_prop_sim->m_links.size(); i++)
	{
		WavePropagationSimulation::VertexLink& link = m_prop_sim->m_links[i];
		Vector3<Real> v1 = m_prop_sim->m_mesh_pos + glm2eigen(m_prop_sim->m_mesh->vertices[link.v1_idx].pos);
		Vector3<Real> v2 = m_prop_sim->m_mesh_pos + glm2eigen(m_prop_sim->m_mesh->vertices[link.v2_idx].pos);

		// delete current link if intersected with out plane
		if (is_line_plane_intersect(v1, v2, m_point, m_normal))
		{
			m_prop_sim->m_links.erase(m_prop_sim->m_links.begin() + i);
			i--;

			// add a new cut line
			m_cut_links_lines.push_back(eigen2glm(v1));
			m_cut_links_lines.push_back(eigen2glm(v2));
		}
	}
}

void WavePropagationPlaneCut::render()
{
	gdevGet()->depthTest(STATE_ENABLED);
	Renderer3D::setStyle(Renderer3D::Style(true, 1, { 0.8, 0, 0, 1 }, false, { 0.75, 0, 0 ,1 }));
	Renderer3D::drawLineList(m_cut_links_lines);

}

void WavePropagationPlaneCut::render_gui()
{
	ImGui::DragVector3Eigen("Point", m_point);
	ImGui::DragVector3Eigen("Normal", m_normal);
}

void WavePropagationPlaneCut::handle_input(const LookAtCamera & camera)
{
}

// Force Depolarization

WavePropagationForceDepolarization::WavePropagationForceDepolarization(WavePropagationSimulation* prop_sim, int vertex_count, Real depolarization_time)
	:WavePropagationOperator(prop_sim, "Force Depolarization")
{
	m_selected.resize(vertex_count);
	m_depolarization_time = depolarization_time;
}

void WavePropagationForceDepolarization::apply_reset()
{
	m_selected.resize(m_prop_sim->m_mesh->vertices.size());
	for (int i = 0; i < m_prop_sim->m_mesh->vertices.size(); i++)
	{
		if (m_selected[i])
		{
			m_prop_sim->m_vars[i] = WavePropagationSimulation::VertexVars{ true, m_depolarization_time };
		}
	}
}

void WavePropagationForceDepolarization::render()
{
	m_brush.render();


	// update mesh values
	if (m_view_drawing)
	{
		for (int i = 0; i < m_prop_sim->m_mesh->vertices.size(); i++)
		{
			m_prop_sim->m_mesh->vertices[i].value = -0.085*!m_selected[i] + 0.015*m_selected[i];
		}
	}
}

void WavePropagationForceDepolarization::render_gui()
{
	m_brush.render_gui();
	ImGui::Checkbox("Select", &m_brush_select);
	ImGui::InputReal("Depolarization Time", &m_depolarization_time);
	ImGui::Checkbox("View Drawing", &m_view_drawing);

}

void WavePropagationForceDepolarization::handle_input(const LookAtCamera & camera)
{
	m_brush.handle_input(camera, m_prop_sim->m_mesh, m_prop_sim->m_mesh_pos);

	for (int i = 0; i < m_brush.get_intersected().size(); i++)
	{
		if (m_brush.get_intersected()[i])
		{
			m_selected[i] = m_brush_select;
		}
	}
}

CircularBrush::CircularBrush(bool enable_drawing, Real brush_radius, bool only_vertices_facing_camera)
{
	m_enable_drawing = enable_drawing;
	m_brush_radius = brush_radius;
	m_only_vertices_facing_camera = only_vertices_facing_camera;
	m_drawing_values_preview.resize(m_drawing_values_points_count);
}

void CircularBrush::render()
{
	// render drawing preview
	if (m_enable_drawing && m_drawing_is_intersected && m_drawing_values_preview.size() >= 2)
	{
		Renderer3D::setStyle(Renderer3D::Style(true, 1, { 1, 1, 1, 1 }, true, { 0.75, 0, 0, 0.2 }));
		m_drawing_values_preview.push_back(m_drawing_values_preview[0]);
		Renderer3D::drawPolygon(&m_drawing_values_preview[0], m_drawing_values_preview.size(), false);
	}

}

void CircularBrush::render_gui()
{
	ImGui::Checkbox("Enable Drawing", &m_enable_drawing);
	ImGui::Checkbox("Draw Only Vertices Facing Camera", &m_only_vertices_facing_camera);
	Real im_brush_radius = log10(m_brush_radius);
	ImGui::DragReal("Drawing Brush Radius", &im_brush_radius, 0.01);
	m_brush_radius = pow(10, im_brush_radius);
	// HANDLED BY THE CALLER
	//ImGui::InputReal("drawing value", &drawing_value);
	//ImGui::Checkbox("view target channel", &drawing_view_target_channel);
}

void CircularBrush::handle_input(const LookAtCamera & camera, MeshPlot * mesh, const Vector3<Real>& mesh_pos)
{
	m_drawing_is_intersected = false;
	m_intersected.resize(mesh->vertices.size());

	for (int i = 0; i < m_intersected.size(); i++)
	{
		m_intersected[i] = false;
	}

	if (!m_enable_drawing)
	{
		return;
	}

	// drawing
	
	// normalized screen coordinates
	Real x = Input::getCursorXPosNorm();
	Real y = Input::getCursorYPosNorm();

	// camera axis
	Vector3<Real> forward = glm2eigen(camera.look_at-camera.eye).normalized();
	Vector3<Real> up = glm2eigen(camera.up).normalized();
	Vector3<Real> right = forward.cross(up).normalized();
	// calculate the pointer direction
	Vector3<Real> direction = forward + up*tan(0.5*y*camera.fov) + right*tan(0.5*x*camera.aspect*camera.fov);
	direction = direction.normalized();

	std::vector<Vector3<Real>> origin_points(m_drawing_values_points_count, Vector3<Real>(0, 0, 0));

	// generate origin points
	for (int i = 0; i < m_drawing_values_points_count; i++)
	{
		Real theta = ((Real)i/(Real)m_drawing_values_points_count)*2*PI;
		origin_points[i] = glm2eigen(camera.eye) + m_brush_radius*sin(theta)*up + m_brush_radius*cos(theta)*right;
	}

	// intersect preview
	m_drawing_values_preview.resize(0);
	Ray ray = { glm2eigen(camera.eye), direction };
	// calculate average t
	Real t_min = 10e19;
	Real t_average = 0;
	int intersected_points = 0;
	for (int i = 0; i < m_drawing_values_points_count; i++)
	{
		Real t;
		int tri_idx;
		ray.origin = origin_points[i];
		if (ray_mesh_intersect(*mesh, mesh_pos, ray, t, tri_idx))
		{
			m_drawing_is_intersected = true;
			t_average += t;
			t_min = rmin(t_min, t);
			intersected_points++;
		}
	}
	t_average /= intersected_points;

	// actual intersection
	Real t;
	int tri_idx;
	for (int i = 0; i < m_drawing_values_points_count; i++)
	{
		ray.origin = origin_points[i];
		if (!ray_mesh_intersect(*mesh, mesh_pos, ray, t, tri_idx))
		{
			t = t_min; // set t to t_average when no intersection
		}

		m_drawing_values_preview.push_back(eigen2glm(ray.point_at_dir(t)));
	}

	// apply drawing value to mesh vertices
	if (Input::isButtonDown(GLFW_MOUSE_BUTTON_LEFT))
	{
		std::vector<int> intersected_values;
		for (int i = 0; i < mesh->vertices.size(); i++)
		{
			if (perpendicular_distance(glm2eigen(camera.eye), glm2eigen(camera.eye)+direction, mesh_pos+glm2eigen(mesh->vertices[i].pos)) < m_brush_radius)
			{
				if (m_only_vertices_facing_camera && glm2eigen(mesh->vertices[i].normal).dot(-forward) > 0)
				{
					intersected_values.push_back(i);
				}
				else if (!m_only_vertices_facing_camera)
				{
					intersected_values.push_back(i);
				}
			}
		}

		// set value
		for (int i = 0; i < intersected_values.size(); i++)
		{
			m_intersected[intersected_values[i]] = true;
		}
	}
}

const std::vector<bool>& CircularBrush::get_intersected() const
{
	return m_intersected;
}

