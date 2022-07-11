#include "wave_propagation_simulation.h"
#include "imgui/imgui.h"
#include "imgui/imgui_my_types.h"
#include "geometry.h"
#include "renderer3d.h"
#include "opengl/gl_headers.h"
#include "GLFW/glfw3.h"
#include "main_dev.h"
#include "action_potential.h"
#include "file_io.h"
#include "filedialog.h"
#include "network/serializer.h"


void WavePropagationSimulation::set_mesh(MeshPlot * mesh, const Eigen::Vector3<Real>& mesh_pos)
{
	// set mesh
	m_mesh = mesh;
	m_mesh_pos = mesh_pos;

	// test operator (plane cut)
	m_operators.push_back(std::shared_ptr<WavePropagationPlaneCut>(new WavePropagationPlaneCut(this, mesh_pos + Vector3<Real>(0, 0, 0), {-1, 1, 0})));
	m_operators.push_back(std::shared_ptr<WavePropagationForceDepolarization>(new WavePropagationForceDepolarization(this)));
	m_operators.push_back(std::shared_ptr<WavePropagationLinkTwoGroups>(new WavePropagationLinkTwoGroups(this)));
	m_operators.push_back(std::shared_ptr<WavePropagationConductionPath>(new WavePropagationConductionPath(this)));

	// resize operators options
	m_operators_enable.resize(m_operators.size(), true);
	m_operators_render.resize(m_operators.size(), false);

	// resize group speeds
	m_mesh_groups_speed.resize(m_mesh->groups_vertices.size(), 1.0);

	// update variables size and reset
	m_vars.resize(m_mesh->vertices.size());
	m_params.resize(m_mesh->vertices.size());
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

	// reset parameters
	for (int i = 0; i < m_mesh->vertices.size(); i++)
	{
		m_params[i] = { 0.250, 1 };
	}

	// reset to initial values
	for (int i = 0; i < m_mesh->vertices.size(); i++)
	{
		m_vars[i] = { false, 0 };
	}

	// apply operators reset
	for (int i = 0; i < m_operators.size(); i++)
	{
		if (m_operators_enable[i])
		{
			m_operators[i]->apply_reset();
		}
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
		Real link_lag = distance/link_speed + link.constant_delay;

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
		ActionPotentialParameters vertex_action_potential_param = { ACTION_POTENTIAL_RESTING_POTENTIAL, ACTION_POTENTIAL_PEAK_POTENTIAL, m_vars[i].depolarization_time, m_vars[i].depolarization_time+m_params[i].deplorized_duration };

		if (m_vars[i].is_depolarized && m_t > m_vars[i].depolarization_time)
		{
			// select from different extracellular potential shapes
			switch (m_selected_extracellular_potential_curve)
			{
			case 0:
				m_potentials(i) = action_potential_value(m_t, vertex_action_potential_param);
				break;
			case 1:
				m_potentials(i) = action_potential_value_2(m_t, vertex_action_potential_param, m_depolarization_slope_duration, m_repolarization_slope_duration);
				break;
			case 2:
				m_potentials(i) = action_potential_value_with_hyperdepolarizaton(m_t, vertex_action_potential_param, m_depolarization_slope_duration, m_repolarization_slope_duration);
				break;
			case 3:
				m_potentials(i) = action_potential_value_with_hyperdepolarizaton_new(m_t, vertex_action_potential_param, m_depolarization_slope_duration, m_repolarization_slope_duration);
				break;
			default:
				m_potentials(i) = ACTION_POTENTIAL_RESTING_POTENTIAL;
				break;
			}

			// apply the amplitude multiplier
			m_potentials(i) = ACTION_POTENTIAL_RESTING_POTENTIAL + m_params[i].amplitude_multiplier*(m_potentials(i)-ACTION_POTENTIAL_RESTING_POTENTIAL);
		}
		else
		{
			m_potentials(i) = ACTION_POTENTIAL_RESTING_POTENTIAL;
		}
	}

}

void WavePropagationSimulation::render()
{
	// reset m_mesh_in_preview
	m_mesh_in_preview = false;

	// operators render
	for (int i = 0; i < m_operators.size(); i++)
	{
		if (m_operators_render[i] || i == m_selected_operator)
		{
			m_operators[i]->render();
		}
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
	// connect close vertices from different groups
	ImGui::InputReal("Close Vertices Threshold", &m_close_vertices_threshold);
	// heart groups opacity
	if (ImGui::BeginTable("Mesh Groups Speed", m_mesh_groups_speed.size(), ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_Borders))
	{
		for (int i = 0; i < m_mesh_groups_speed.size(); i++)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Text("Group(%d)", i);
			ImGui::TableNextColumn();
			ImGui::InputReal((std::string("Speed_") + std::to_string(i)).c_str(), &m_mesh_groups_speed[i], 0, 1);
		}
		ImGui::EndTable();
	}
	ImGui::Checkbox("Connect Close Vertices From Different Groups", &m_connect_close_vertices_from_different_groups);
	ImGui::Checkbox("Connect Close Vertices From The Same Groups", &m_connect_close_vertices_from_same_group);
	if (ImGui::Button("Recalculate Links"))
	{
		recalculate_links();
	}
	if (ImGui::Button("Reset Wave"))
	{
		reset();
	}
	
	// extracellular potential select
	const char* im_extracellular_potential_curve_items[] = { 
		"TMP Potential 1 (OLD)", 
		"TMP Potential 2", 
		"TMP Potential Over Depolarization",
		"TMP Potential Over Depolarization (NEW)",
	};
	ImGui::Combo("TMP Potential Curve", &m_selected_extracellular_potential_curve, im_extracellular_potential_curve_items, IM_ARRAYSIZE(im_extracellular_potential_curve_items));
	static Real preview_dep_time = 0.2;
	static Real preview_rep_time = 0.7;
	ImGui::InputReal("Preview Depolarization Time", &preview_dep_time);
	ImGui::InputReal("Preview Repolarization Time", &preview_rep_time);
	static std::vector<float> im_extracellular_potential_curve_preview;
	im_extracellular_potential_curve_preview.resize(250);
	ActionPotentialParameters preview_parameters = { ACTION_POTENTIAL_RESTING_POTENTIAL, ACTION_POTENTIAL_PEAK_POTENTIAL, preview_dep_time, preview_rep_time };
	for (int i = 0; i < im_extracellular_potential_curve_preview.size(); i++)
	{
		float t = (float)i/(float)im_extracellular_potential_curve_preview.size();
		switch(m_selected_extracellular_potential_curve)
		{
		case 0:
			im_extracellular_potential_curve_preview[i] = action_potential_value(t, preview_parameters);
			break;
		case 1:
			im_extracellular_potential_curve_preview[i] = action_potential_value_2(t, preview_parameters, m_depolarization_slope_duration, m_repolarization_slope_duration);
			break;
		case 2:
			im_extracellular_potential_curve_preview[i] = action_potential_value_with_hyperdepolarizaton(t, preview_parameters, m_depolarization_slope_duration, m_repolarization_slope_duration);
			break;
		case 3:
			im_extracellular_potential_curve_preview[i] = action_potential_value_with_hyperdepolarizaton_new(t, preview_parameters, m_depolarization_slope_duration, m_repolarization_slope_duration);
			break;
		default:
			im_extracellular_potential_curve_preview[i] = 0;
			break;
		}
	}
	ImGui::PlotLines("Extracellular Potential Curve Plot", &im_extracellular_potential_curve_preview[0], im_extracellular_potential_curve_preview.size(), 0, NULL, FLT_MAX, FLT_MAX, ImVec2(0, 120));

	// resize operators options
	m_operators_enable.resize(m_operators.size(), true);
	m_operators_render.resize(m_operators.size(), false);

	// operators list
	ImGui::Text("Operators");
	if (ImGui::BeginTable("Operators", 4, ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_Borders))
	{
		ImGui::TableNextRow();
		ImGui::TableNextColumn();
		ImGui::Text("Index");
		ImGui::TableNextColumn();
		ImGui::Text("Type");
		ImGui::TableNextColumn();
		ImGui::Text("Enabled");
		ImGui::TableNextColumn();
		ImGui::Text("Render");

		for (int i = 0; i < m_operators.size(); i++)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();

			bool is_selected = i==m_selected_operator;
			ImGui::Selectable(std::to_string(i).c_str(), &is_selected);
			if (is_selected)
			{
				m_selected_operator = i;
			}
			
			// type
			ImGui::TableNextColumn();
			ImGui::Text("%s", m_operators[i]->get_type().c_str());

			// is enabled
			ImGui::TableNextColumn();
			bool op_enable = m_operators_enable[i];
			ImGui::Checkbox(("E" + std::to_string(i)).c_str(), &op_enable);
			m_operators_enable[i] = op_enable;

			// render
			ImGui::TableNextColumn();
			bool op_render = m_operators_render[i];
			ImGui::Checkbox(("R" + std::to_string(i)).c_str(), &op_render);
			m_operators_render[i] = op_render;

		}
		ImGui::EndTable();
	}

	// selected operator add
	const char* im_add_operator_type_items[] = { "Plane Cut", "Force Depolarization", "Link Two Groups", "Conduction Path", "Set Params In Plane", "Set Params In Select"};
	ImGui::Combo("Add Operator Type", &m_selected_operator_add, im_add_operator_type_items, 6);
	if (ImGui::Button("Add Operator") && m_selected_operator_add != -1)
	{
		switch (m_selected_operator_add)
		{
		case 0:
			m_operators.push_back(std::shared_ptr<WavePropagationPlaneCut>(new WavePropagationPlaneCut(this, m_mesh_pos + Vector3<Real>(0, 0, 0), { -1, 1, 0 })));
			break;
		case 1:
			m_operators.push_back(std::shared_ptr<WavePropagationForceDepolarization>(new WavePropagationForceDepolarization(this)));
			break;
		case 2:
			m_operators.push_back(std::shared_ptr<WavePropagationLinkTwoGroups>(new WavePropagationLinkTwoGroups(this)));
			break;
		case 3:
			m_operators.push_back(std::shared_ptr<WavePropagationConductionPath>(new WavePropagationConductionPath(this)));
			break;
		case 4:
			m_operators.push_back(std::shared_ptr<WavePropagationSetParamsInPlane>(new WavePropagationSetParamsInPlane(this, m_mesh_pos + Vector3<Real>(0, 0, 0), { -1, 1, 0 })));
			break;
		case 5:
			m_operators.push_back(std::shared_ptr<WavePropagationSetParamsInSelect>(new WavePropagationSetParamsInSelect(this)));
			break;
		default:
			break;
		}
	}

	// selected operator remove
	if (ImGui::Button("Remove Operator"))
	{
		if (m_selected_operator != -1 && m_selected_operator < m_operators.size())
		{
			m_operators.erase(m_operators.begin() + m_selected_operator);
		}
	}

	// Import probes locations
	if (ImGui::Button("Load Wave Propagation Configuration"))
	{
		// open file dialog
		std::string file_name = open_file_dialog("", "All\0*.*\0");

		// import
		if (file_name != "")
		{
			if (load_from_file(file_name))
			{
				printf("Loaded wave propagation configuration to \"%s\" to file\n", file_name.c_str());
			}
			else
			{
				printf("Failed to load wave propagation configuration to \"%s\" file\n", file_name.c_str());
			}
		}
	}
	// Export probes locations
	if (ImGui::Button("Save Wave Propagation Configuration"))
	{
		// save file dialog
		std::string file_name = save_file_dialog("", "All\0*.*\0");

		// export
		if (file_name != "")
		{
			if (save_to_file(file_name))
			{
				printf("Saved wave propagation configuration to \"%s\" to file\n", file_name.c_str());
			}
			else
			{
				printf("Failed to save wave propagation configuration to \"%s\" file\n", file_name.c_str());
			}
		}
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

bool WavePropagationSimulation::is_mesh_in_preview()
{
	return m_mesh_in_preview;
}

Real WavePropagationSimulation::get_mesh_in_preview_min()
{
	return m_mesh_in_preview_min;
}

Real WavePropagationSimulation::get_mesh_in_preview_max()
{
	return m_mesh_in_preview_max;
}

void WavePropagationSimulation::recalculate_links()
{
	m_links.clear();

	// add links for vertices connected together with faces
	for (const MeshPlotFace& face : m_mesh->faces)
	{
		m_links.push_back({ face.idx[0], face.idx[1], 1, 0, 0 });
		m_links.push_back({ face.idx[0], face.idx[2], 1, 0, 0 });
		m_links.push_back({ face.idx[1], face.idx[2], 1, 0, 0 });
	}
	
	// connect close vertices
	for (int i = 0; i < m_mesh->vertices.size(); i++)
	{
		for (int j = 0; j < m_mesh->vertices.size(); j++)
		{
			// skip if in the same group
			if ((m_connect_close_vertices_from_different_groups && m_mesh->vertices[i].group != m_mesh->vertices[j].group)
				|| (m_connect_close_vertices_from_same_group && m_mesh->vertices[i].group == m_mesh->vertices[j].group))
			{
				Real distance_square =
					abs(m_mesh->vertices[i].pos.x - m_mesh->vertices[j].pos.x)
					+ abs(m_mesh->vertices[i].pos.y - m_mesh->vertices[j].pos.y)
					+ abs(m_mesh->vertices[i].pos.z - m_mesh->vertices[j].pos.z);
				if (distance_square <= m_close_vertices_threshold*m_close_vertices_threshold)
				{
					Real speed = m_mesh_groups_speed[m_mesh->vertices[i].group];
					m_links.push_back({ i, j, speed, 0, 0 });
				}
			}
		}
	}

	// apply operators links
	for (int i = 0; i < m_operators.size(); i++)
	{
		if (m_operators_enable[i])
		{
			m_operators[i]->apply_links();
		}
	}
}

bool WavePropagationSimulation::load_from_file(const std::string& path)
{
	size_t contents_size;
	uint8_t* contents = file_read(path.c_str(), &contents_size);
	if (!contents)
	{
		return false;
	}

	Deserializer des(contents, contents_size);

	//  magic number
	// check WavePropagationConfiguration header
	std::string header = des.parse_string();
	if (header != "WavePropagationConfiguration")
	{
		return false;
	}

	// deserialize variables
	m_duration = des.parse_double();
	m_dt = des.parse_double();
	m_base_speed = des.parse_double();
	m_depolarization_duration = des.parse_double();
	m_depolarization_slope_duration = des.parse_double();
	m_repolarization_slope_duration = des.parse_double();
	m_close_vertices_threshold = des.parse_double();
	m_connect_close_vertices_from_different_groups = des.parse_u8();

	// serialize groups speeds
	uint32_t groups_speed_size = des.parse_u32(); // size
	m_mesh_groups_speed.resize(groups_speed_size);
	for (int i = 0; i < groups_speed_size; i++)
	{
		m_mesh_groups_speed[i] = des.parse_double();
	}

	// deserialize operators
	uint32_t operators_count = des.parse_u32();
	m_operators.clear();
	for (int i = 0; i < operators_count; i++)
	{
		std::string operator_type = des.parse_string();
		if (operator_type == "Plane Cut")
		{
			m_operators.push_back(std::shared_ptr<WavePropagationPlaneCut>(new WavePropagationPlaneCut(this, m_mesh_pos + Vector3<Real>(0, 0, 0), { -1, 1, 0 })));
		}
		else if (operator_type == "Force Depolarization")
		{
			m_operators.push_back(std::shared_ptr<WavePropagationForceDepolarization>(new WavePropagationForceDepolarization(this)));
		}
		else if (operator_type == "Link Two Groups")
		{
			m_operators.push_back(std::shared_ptr<WavePropagationLinkTwoGroups>(new WavePropagationLinkTwoGroups(this)));
		}
		else if (operator_type == "Conduction Path")
		{
			m_operators.push_back(std::shared_ptr<WavePropagationConductionPath>(new WavePropagationConductionPath(this)));
		}
		else if (operator_type == "Set Params In Plane")
		{
			m_operators.push_back(std::shared_ptr<WavePropagationSetParamsInPlane>(new WavePropagationSetParamsInPlane(this)));
		}
		else if (operator_type == "Set Params In Select")
		{
			m_operators.push_back(std::shared_ptr<WavePropagationSetParamsInSelect>(new WavePropagationSetParamsInSelect(this)));
		}
		else
		{
			return false;
		}

		m_operators.back()->deserialize(des);
	}

	return true;
}

bool WavePropagationSimulation::save_to_file(const std::string& path)
{
	Serializer ser;

	// serialize WavePropagationConfiguration header
	ser.push_string("WavePropagationConfiguration");

	// serialize variables
	ser.push_double(m_duration);
	ser.push_double(m_dt);
	ser.push_double(m_base_speed);
	ser.push_double(m_depolarization_duration);
	ser.push_double(m_depolarization_slope_duration);
	ser.push_double(m_repolarization_slope_duration);
	ser.push_double(m_close_vertices_threshold);
	ser.push_u8(m_connect_close_vertices_from_different_groups);

	// serialize groups speeds
	ser.push_u32(m_mesh_groups_speed.size()); // size
	for (int i = 0; i < m_mesh_groups_speed.size(); i++)
	{
		ser.push_double(m_mesh_groups_speed[i]);
	}

	// serialize operators
	ser.push_u32(m_operators.size());
	for (int i = 0; i < m_operators.size(); i++)
	{
		ser.push_string(m_operators[i]->get_type());
		m_operators[i]->serialize(ser);
	}

	return file_write(path.c_str(), ser.get_data());
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

void WavePropagationOperator::serialize(Serializer & ser)
{
}

void WavePropagationOperator::deserialize(Deserializer & des)
{
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

void WavePropagationPlaneCut::serialize(Serializer & ser)
{
	ser.push_double(m_point.x());
	ser.push_double(m_point.y());
	ser.push_double(m_point.z());
							 
	ser.push_double(m_normal.x());
	ser.push_double(m_normal.y());
	ser.push_double(m_normal.z());
}

void WavePropagationPlaneCut::deserialize(Deserializer & des)
{
	m_point.x() = des.parse_double();
	m_point.y() = des.parse_double();
	m_point.z() = des.parse_double();

	m_normal.x() = des.parse_double();
	m_normal.y() = des.parse_double();
	m_normal.z() = des.parse_double();
}

// Force Depolarization

WavePropagationForceDepolarization::WavePropagationForceDepolarization(WavePropagationSimulation* prop_sim, Real depolarization_time)
	:WavePropagationOperator(prop_sim, "Force Depolarization")
{
	if (m_prop_sim->m_mesh)
	{
		m_selected.resize(m_prop_sim->m_mesh->vertices.size());
	}
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
			m_prop_sim->m_mesh->vertices[i].value = 0*(!m_selected[i]) + 1*m_selected[i];
		}

		m_prop_sim->m_mesh_in_preview = true;
		m_prop_sim->m_mesh_in_preview_max = 1;
		m_prop_sim->m_mesh_in_preview_min = 0;
	}
}

void WavePropagationForceDepolarization::render_gui()
{
	ImGui::InputReal("Depolarization Time", &m_depolarization_time);

	ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
	m_brush.render_gui();
	ImGui::Checkbox("Select", &m_brush_select);
	ImGui::Checkbox("View Drawing", &m_view_drawing);

}

void WavePropagationForceDepolarization::handle_input(const LookAtCamera & camera)
{
	m_selected.resize(m_prop_sim->m_mesh->vertices.size());

	m_brush.handle_input(camera, m_prop_sim->m_mesh, m_prop_sim->m_mesh_pos);

	for (int i = 0; i < m_brush.get_intersected().size(); i++)
	{
		if (m_brush.get_intersected()[i])
		{
			m_selected[i] = m_brush_select;
		}
	}
}

void WavePropagationForceDepolarization::serialize(Serializer & ser)
{
	ser.push_u32(m_selected.size());
	for (int i = 0; i < m_selected.size(); i++)
	{
		ser.push_u8(m_selected[i]);
	}

	ser.push_double(m_depolarization_time);
}

void WavePropagationForceDepolarization::deserialize(Deserializer & des)
{
	uint32_t selected_count = des.parse_u32();
	m_selected.resize(selected_count);
	for (int i = 0; i < selected_count; i++)
	{
		m_selected[i] = des.parse_u8();
	}

	m_depolarization_time = des.parse_double();
}

// Circular Brush

CircularBrush::CircularBrush(bool enable_drawing, Real brush_radius, bool only_vertices_facing_camera)
{
	m_enable_drawing = enable_drawing;
	m_brush_radius = brush_radius;
	m_only_vertices_facing_camera = only_vertices_facing_camera;
	m_drawing_values_preview.resize(m_drawing_values_points_count);
	m_mesh_groups_count = 1;
	m_mesh_group_selected = 0;
}

void CircularBrush::render()
{
	// render drawing preview
	if (m_enable_drawing && m_drawing_is_intersected && m_drawing_values_preview.size() >= 2)
	{
		gdevGet()->depthTest(STATE_ENABLED);
		Renderer3D::setStyle(Renderer3D::Style(true, 1, { 1, 1, 1, 1 }, false, { 0.75, 0, 0, 0.2 }));
		m_drawing_values_preview.push_back(m_drawing_values_preview[0]);
		Renderer3D::drawPolygon(&m_drawing_values_preview[0], m_drawing_values_preview.size(), false);
	}

}

void CircularBrush::render_gui()
{
	ImGui::Checkbox("Enable Drawing", &m_enable_drawing);
	ImGui::Checkbox("Draw Only Vertices Facing Camera", &m_only_vertices_facing_camera);

	// mesh select group
	if (ImGui::ListBoxHeader("Mesh Selected Group", m_mesh_groups_count+1))
	{
		// ALL GROUPS
		if (ImGui::Selectable("ALL GROUPS", -1==m_mesh_group_selected))
		{
			m_mesh_group_selected = -1;
		}

		for (int i = 0; i < m_mesh_groups_count; i++)
		{
			std::string name = std::string("Group ") + std::to_string(i);
			if (ImGui::Selectable(name.c_str(), i==m_mesh_group_selected))
			{
				m_mesh_group_selected = i;
			}
		}
		ImGui::ListBoxFooter();
	}

	Real im_brush_radius = log10(m_brush_radius);
	ImGui::DragReal("Drawing Brush Radius", &im_brush_radius, 0.01);
	m_brush_radius = pow(10, im_brush_radius);
	// HANDLED BY THE CALLER
	//ImGui::InputReal("drawing value", &drawing_value);
	//ImGui::Checkbox("view target channel", &drawing_view_target_channel);
}

void CircularBrush::handle_input(const LookAtCamera & camera, MeshPlot * mesh, const Vector3<Real>& mesh_pos)
{
	m_mesh_groups_count = mesh->groups_vertices.size();
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
	m_drawing_values_preview.clear();
	Ray ray = { glm2eigen(camera.eye), direction };
	// calculate average t
	Real t_min = 1e3;
	Real t_average = 0;
	int intersected_points = 0;
	for (int i = 0; i < m_drawing_values_points_count; i++)
	{
		Real t;
		int tri_idx;
		ray.origin = origin_points[i];
		if (ray_mesh_intersect(*mesh, mesh_pos, ray, t, tri_idx, m_mesh_group_selected))
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
		if (!ray_mesh_intersect(*mesh, mesh_pos, ray, t, tri_idx, m_mesh_group_selected))
		{
			t = t_min; // set t to t_average when no intersection
		}
		t = t_min; // set t to t_average when no intersection

		m_drawing_values_preview.push_back(eigen2glm(ray.point_at_dir(t)));
	}

	// apply drawing value to mesh vertices
	if (Input::isButtonDown(GLFW_MOUSE_BUTTON_LEFT))
	{
		std::vector<int> intersected_values;
		for (int i = 0; i < mesh->vertices.size(); i++)
		{
			// skip current vertex if not in the selected group
			if (m_mesh_group_selected != -1 && mesh->vertices[i].group != m_mesh_group_selected)
			{
				continue;
			}

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


// Link Two Groups

WavePropagationLinkTwoGroups::WavePropagationLinkTwoGroups(WavePropagationSimulation* prop_sim)
	:WavePropagationOperator(prop_sim, "Link Two Groups")
{
	if (m_prop_sim->m_mesh)
	{
		m_group_a_selected.resize(m_prop_sim->m_mesh->vertices.size());
		m_group_b_selected.resize(m_prop_sim->m_mesh->vertices.size());
	}
}

void WavePropagationLinkTwoGroups::apply_links()
{
	m_group_a_selected.resize(m_prop_sim->m_mesh->vertices.size());
	m_group_b_selected.resize(m_prop_sim->m_mesh->vertices.size());

	for (int i = 0; i < m_group_a_selected.size(); i++)
	{
		for (int j = 0; j < m_group_b_selected.size(); j++)
		{
			if (m_group_a_selected[i] && m_group_b_selected[j])
			{
				// connect between two vertices in group A and B
				m_prop_sim->m_links.push_back({ i, j, m_multiply_speed, m_constant_speed, m_constant_delay });
			}
		}
	}
}

void WavePropagationLinkTwoGroups::render()
{
	m_brush.render();

	// update mesh values
	if (m_view_drawing)
	{
		for (int i = 0; i < m_prop_sim->m_mesh->vertices.size(); i++)
		{
			if (m_selected_group == 0)
			{
				m_prop_sim->m_mesh->vertices[i].value = 0*(!m_group_a_selected[i]) + 1*m_group_a_selected[i];
			}
			else if (m_selected_group == 1)
			{
				m_prop_sim->m_mesh->vertices[i].value = 0*(!m_group_b_selected[i]) + 1*m_group_b_selected[i];
			}
		}

		m_prop_sim->m_mesh_in_preview = true;
		m_prop_sim->m_mesh_in_preview_max = 1;
		m_prop_sim->m_mesh_in_preview_min = 0;
	}
}

void WavePropagationLinkTwoGroups::render_gui()
{
	ImGui::InputReal("Multiply Speed", &m_multiply_speed);
	ImGui::InputReal("Constant Speed", &m_constant_speed);
	ImGui::InputReal("Constant Delay", &m_constant_delay);

	ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
	const char* group_select_choices[] = {"Group A", "Group B"};
	ImGui::ListBox("Group Select", &m_selected_group, group_select_choices, 2, 2);
	m_brush.render_gui();
	ImGui::Checkbox("Select", &m_brush_select);
	ImGui::Checkbox("View Drawing", &m_view_drawing);
}

void WavePropagationLinkTwoGroups::handle_input(const LookAtCamera& camera)
{
	m_group_a_selected.resize(m_prop_sim->m_mesh->vertices.size());
	m_group_b_selected.resize(m_prop_sim->m_mesh->vertices.size());

	m_brush.handle_input(camera, m_prop_sim->m_mesh, m_prop_sim->m_mesh_pos);

	for (int i = 0; i < m_brush.get_intersected().size(); i++)
	{
		if (m_brush.get_intersected()[i])
		{
			if (m_selected_group == 0)
			{
				m_group_a_selected[i] = m_brush_select;
			}
			else if (m_selected_group == 1)
			{
				m_group_b_selected[i] = m_brush_select;
			}
		}
	}
}

void WavePropagationLinkTwoGroups::serialize(Serializer & ser)
{
	// group A
	ser.push_u32(m_group_a_selected.size());
	for (int i = 0; i < m_group_a_selected.size(); i++)
	{
		ser.push_u8(m_group_a_selected[i]);
	}

	// group A
	ser.push_u32(m_group_b_selected.size());
	for (int i = 0; i < m_group_b_selected.size(); i++)
	{
		ser.push_u8(m_group_b_selected[i]);
	}

	ser.push_double(m_multiply_speed);
	ser.push_double(m_constant_speed);
	ser.push_double(m_constant_delay);
}

void WavePropagationLinkTwoGroups::deserialize(Deserializer & des)
{
	// group A
	uint32_t selected_count_a = des.parse_u32();
	m_group_a_selected.resize(selected_count_a);
	for (int i = 0; i < selected_count_a; i++)
	{
		m_group_a_selected[i] = des.parse_u8();
	}

	// group B
	uint32_t selected_count_b = des.parse_u32();
	m_group_b_selected.resize(selected_count_b);
	for (int i = 0; i < selected_count_b; i++)
	{
		m_group_b_selected[i] = des.parse_u8();
	}

	m_multiply_speed = des.parse_double();
	m_constant_speed = des.parse_double();
	m_constant_delay = des.parse_double();
}

// Conduction Path

WavePropagationConductionPath::WavePropagationConductionPath(WavePropagationSimulation* prop_sim)
	:WavePropagationOperator(prop_sim, "Conduction Path")
{
	
}

void WavePropagationConductionPath::apply_links()
{
	// add links for the conduction path
	for (int i = 1; i < m_points_list.size(); i++)
	{
		m_prop_sim->m_links.push_back({ m_points_list[i-1], m_points_list[i], m_multiply_speed, m_constant_speed, m_constant_delay});
	}
}

void WavePropagationConductionPath::render()
{
	// construct line points
	std::vector<glm::vec3> points_list;
	points_list.resize(m_points_list.size());
	for (int i = 0; i < m_points_list.size(); i++)
	{
		points_list[i] = eigen2glm(m_prop_sim->m_mesh_pos) + m_prop_sim->m_mesh->vertices[m_points_list[i]].pos;
	}

	// render line
	Renderer3D::setStyle(Renderer3D::Style(true, 2, { 0.8, 0, 0, 1 }, false, { 0.75, 0, 0 ,1 }));
	Renderer3D::drawPolygon(&points_list[0], points_list.size());

	// selected point
	if (m_selected_point != -1 && m_selected_point < m_points_list.size())
	{
		Renderer3D::drawPoint(points_list[m_selected_point], { 0, 0.8, 0, 1 }, 3);
	}

	// adding point preview
	if (m_adding_points && m_adding_points_preview_idx != -1 && m_adding_points_preview_idx < m_points_list.size())
	{
		Renderer3D::drawPoint(eigen2glm(m_prop_sim->m_mesh_pos) + m_prop_sim->m_mesh->vertices[m_adding_points_preview_idx].pos, { 0, 0, 0.8, 1 }, 3);
	}
}

void WavePropagationConductionPath::render_gui()
{
	ImGui::InputReal("Multiply Speed", &m_multiply_speed);
	ImGui::InputReal("Constant Speed", &m_constant_speed);
	ImGui::InputReal("Constant Delay", &m_constant_delay);

	ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
	if (ImGui::ListBoxHeader("Points List", { 0, 120 }))
	{
		for (int i = 0; i < m_points_list.size(); i++)
		{
			bool is_selected = i==m_selected_point;
			ImGui::Selectable(std::to_string(i).c_str(), &is_selected);
			if (is_selected)
			{
				m_selected_point = i;
			}

			// value
			ImGui::SameLine();
			ImGui::Text(" %d, (%.3f, %.3f, %.3f)", m_points_list[i],
				(eigen2glm(m_prop_sim->m_mesh_pos) + m_prop_sim->m_mesh->vertices[m_points_list[i]].pos).x,
				(eigen2glm(m_prop_sim->m_mesh_pos) + m_prop_sim->m_mesh->vertices[m_points_list[i]].pos).y,
				(eigen2glm(m_prop_sim->m_mesh_pos) + m_prop_sim->m_mesh->vertices[m_points_list[i]].pos).z);
		}
		ImGui::ListBoxFooter();
	}

	// mesh select group
	if (ImGui::ListBoxHeader("Mesh Selected Group", m_prop_sim->m_mesh->groups_vertices.size()+1))
	{
		// ALL GROUPS
		if (ImGui::Selectable("ALL GROUPS", -1==m_selected_group))
		{
			m_selected_group = -1;
		}

		for (int i = 0; i < m_prop_sim->m_mesh->groups_vertices.size(); i++)
		{
			std::string name = std::string("Group ") + std::to_string(i);
			if (ImGui::Selectable(name.c_str(), i==m_selected_group))
			{
				m_selected_group = i;
			}
		}
		ImGui::ListBoxFooter();
	}

	if (ImGui::Button("Remove Point"))
	{
		if (m_selected_point != -1 && m_selected_point < m_points_list.size())
		{
			m_points_list.erase(m_points_list.begin() + m_selected_point);
		}
	}
	if (ImGui::Button("Clear Points List"))
	{
		m_points_list.clear();
	}
	ImGui::Checkbox("Adding Points", &m_adding_points);
}

void WavePropagationConductionPath::handle_input(const LookAtCamera& camera)
{
	if (m_adding_points)
	{
		// camera mouse ray
		Ray mouse_ray = camera_screen_to_world_ray(camera, Input::getCursorXPosNorm(), Input::getCursorYPosNorm());
		
		// find the nearest point
		int nearest_point_idx = -1;

		// hit ray to mesh and find the intersecting triangle
		Real t;
		int tri_idx;
		if (ray_mesh_intersect(*m_prop_sim->m_mesh, m_prop_sim->m_mesh_pos, mouse_ray, t, tri_idx, m_selected_group))
		{
			// find the nearest point on that triangle to our intersecting point
			Vector3<Real> intersection_point = mouse_ray.point_at_dir(t);
			Real d0 = (glm2eigen(m_prop_sim->m_mesh->vertices[m_prop_sim->m_mesh->faces[tri_idx].idx[0]].pos) - intersection_point).norm();
			Real d1 = (glm2eigen(m_prop_sim->m_mesh->vertices[m_prop_sim->m_mesh->faces[tri_idx].idx[1]].pos) - intersection_point).norm();
			Real d2 = (glm2eigen(m_prop_sim->m_mesh->vertices[m_prop_sim->m_mesh->faces[tri_idx].idx[2]].pos) - intersection_point).norm();
			if (d0 < d1 && d0 < d2)
			{
				nearest_point_idx = m_prop_sim->m_mesh->faces[tri_idx].idx[0];
			}
			else if (d1 < d2)
			{
				nearest_point_idx = m_prop_sim->m_mesh->faces[tri_idx].idx[1];
			}
			else
			{
				nearest_point_idx = m_prop_sim->m_mesh->faces[tri_idx].idx[2];
			}
		}

		// set preview
		m_adding_points_preview_idx = nearest_point_idx;

		// add point at left mouse click
		if (Input::isButtonPressed(GLFW_MOUSE_BUTTON_LEFT) && nearest_point_idx != -1)
		{
			m_points_list.push_back(nearest_point_idx);
		}
	}

	// Escape: cancel adding points
	if (Input::isKeyPressed(GLFW_KEY_ESCAPE))
	{
		m_adding_points = false;
	}
}

void WavePropagationConductionPath::serialize(Serializer & ser)
{
	ser.push_double(m_multiply_speed);
	ser.push_double(m_constant_speed);
	ser.push_double(m_constant_delay);

	// points list
	ser.push_u32(m_points_list.size());
	for (int i = 0; i < m_points_list.size(); i++)
	{
		ser.push_i32(m_points_list[i]);
	}
}

void WavePropagationConductionPath::deserialize(Deserializer & des)
{
	m_multiply_speed = des.parse_double();
	m_constant_speed = des.parse_double();
	m_constant_delay = des.parse_double();

	// points list
	uint32_t points_list_count = des.parse_u32();
	m_points_list.resize(points_list_count);
	for (int i = 0; i < points_list_count; i++)
	{
		m_points_list[i] = des.parse_i32();
	}
}

// Set Params In Plane

WavePropagationSetParamsInPlane::WavePropagationSetParamsInPlane(WavePropagationSimulation * prop_sim, const Vector3<Real>& point, const Vector3<Real>& normal)
	:WavePropagationOperator(prop_sim, "Set Params In Plane")
{
	m_point = point;
	m_normal = normal.normalized();
}

void WavePropagationSetParamsInPlane::apply_reset()
{
	for (int i = 0; i < m_prop_sim->m_mesh->vertices.size(); i++)
	{
		Vector3<Real> vertex_pos = m_prop_sim->m_mesh_pos + glm2eigen(m_prop_sim->m_mesh->vertices[i].pos);
		if ((vertex_pos - m_point).dot(m_normal) > 0)
		{
			m_prop_sim->m_params[i] = m_params;
		}
	}
}

void WavePropagationSetParamsInPlane::render()
{
	// update mesh values
	for (int i = 0; i < m_prop_sim->m_mesh->vertices.size(); i++)
	{
		// skip if vertex isn't from the selected group
		if (m_mesh_group_selected != -1 && m_prop_sim->m_mesh->vertices[i].group != m_mesh_group_selected)
		{
			continue;
		}

		Vector3<Real> vertex_pos = m_prop_sim->m_mesh_pos + glm2eigen(m_prop_sim->m_mesh->vertices[i].pos);
		bool is_selected = ((vertex_pos - m_point).dot(m_normal) > 0);
		m_prop_sim->m_mesh->vertices[i].value = 0*(!is_selected) + 1*is_selected;
	}

	m_prop_sim->m_mesh_in_preview = true;
	m_prop_sim->m_mesh_in_preview_max = 1;
	m_prop_sim->m_mesh_in_preview_min = 0;
}

void WavePropagationSetParamsInPlane::render_gui()
{
	ImGui::DragVector3Eigen("Point", m_point);
	ImGui::DragVector3Eigen("Normal", m_normal);

	// mesh select group
	if (ImGui::ListBoxHeader("Mesh Selected Group", m_prop_sim->m_mesh->groups_vertices.size()+1))
	{
		// ALL GROUPS
		if (ImGui::Selectable("ALL GROUPS", -1==m_mesh_group_selected))
		{
			m_mesh_group_selected = -1;
		}

		for (int i = 0; i < m_prop_sim->m_mesh->groups_vertices.size(); i++)
		{
			std::string name = std::string("Group ") + std::to_string(i);
			if (ImGui::Selectable(name.c_str(), i==m_mesh_group_selected))
			{
				m_mesh_group_selected = i;
			}
		}
		ImGui::ListBoxFooter();
	}

	ImGui::InputReal("Deplorized Duration", &m_params.deplorized_duration);
	ImGui::InputReal("Amplitude Multiplier", &m_params.amplitude_multiplier);
}

void WavePropagationSetParamsInPlane::handle_input(const LookAtCamera & camera)
{
}

void WavePropagationSetParamsInPlane::serialize(Serializer & ser)
{
	ser.push_double(m_point.x());
	ser.push_double(m_point.y());
	ser.push_double(m_point.z());

	ser.push_double(m_normal.x());
	ser.push_double(m_normal.y());
	ser.push_double(m_normal.z());

	ser.push_i32(m_mesh_group_selected);

	ser.push_double(m_params.deplorized_duration);
	ser.push_double(m_params.amplitude_multiplier);
}

void WavePropagationSetParamsInPlane::deserialize(Deserializer & des)
{
	m_point.x() = des.parse_double();
	m_point.y() = des.parse_double();
	m_point.z() = des.parse_double();

	m_normal.x() = des.parse_double();
	m_normal.y() = des.parse_double();
	m_normal.z() = des.parse_double();

	m_mesh_group_selected = des.parse_i32();

	m_params.deplorized_duration = des.parse_double();
	m_params.amplitude_multiplier = des.parse_double();
}

// Set Params In Select

WavePropagationSetParamsInSelect::WavePropagationSetParamsInSelect(WavePropagationSimulation * prop_sim, const Vector3<Real>& point, const Vector3<Real>& normal)
	:WavePropagationOperator(prop_sim, "Set Params In Select")
{
	if (m_prop_sim->m_mesh)
	{
		m_selected.resize(m_prop_sim->m_mesh->vertices.size());
	}
}

void WavePropagationSetParamsInSelect::apply_reset()
{
	for (int i = 0; i < m_prop_sim->m_mesh->vertices.size(); i++)
	{
		if (m_selected[i])
		{
			m_prop_sim->m_params[i] = m_params;
		}
	}
}

void WavePropagationSetParamsInSelect::render()
{
	m_brush.render();

	// update mesh values
	if (m_view_drawing)
	{
		for (int i = 0; i < m_prop_sim->m_mesh->vertices.size(); i++)
		{
			m_prop_sim->m_mesh->vertices[i].value = 0*(!m_selected[i]) + 1*m_selected[i];
		}

		m_prop_sim->m_mesh_in_preview = true;
		m_prop_sim->m_mesh_in_preview_max = 1;
		m_prop_sim->m_mesh_in_preview_min = 0;
	}
}

void WavePropagationSetParamsInSelect::render_gui()
{
	ImGui::InputReal("Deplorized Duration", &m_params.deplorized_duration);
	ImGui::InputReal("Amplitude Multiplier", &m_params.amplitude_multiplier);

	ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
	m_brush.render_gui();
	ImGui::Checkbox("Select", &m_brush_select);
	ImGui::Checkbox("View Drawing", &m_view_drawing);
}

void WavePropagationSetParamsInSelect::handle_input(const LookAtCamera & camera)
{
	m_selected.resize(m_prop_sim->m_mesh->vertices.size());

	m_brush.handle_input(camera, m_prop_sim->m_mesh, m_prop_sim->m_mesh_pos);

	for (int i = 0; i < m_brush.get_intersected().size(); i++)
	{
		if (m_brush.get_intersected()[i])
		{
			m_selected[i] = m_brush_select;
		}
	}
}

void WavePropagationSetParamsInSelect::serialize(Serializer & ser)
{
	ser.push_u32(m_selected.size());
	for (int i = 0; i < m_selected.size(); i++)
	{
		ser.push_u8(m_selected[i]);
	}

	ser.push_double(m_params.deplorized_duration);
	ser.push_double(m_params.amplitude_multiplier);
}

void WavePropagationSetParamsInSelect::deserialize(Deserializer & des)
{
	uint32_t selected_count = des.parse_u32();
	m_selected.resize(selected_count);
	for (int i = 0; i < selected_count; i++)
	{
		m_selected[i] = des.parse_u8();
	}

	m_params.deplorized_duration = des.parse_double();
	m_params.amplitude_multiplier = des.parse_double();
}

