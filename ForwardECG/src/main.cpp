#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <thread>
#include "opengl/gl_headers.h"
#include <GLFW/glfw3.h>
#include "window.h"
#include "input.h"
#include "main_dev.h"
#include "opengl/gl_graphics_device.h"
#include "transform.h"
#include "renderer2d.h"
#include "renderer3d.h"
#include "model.h"
#include "camera.h"
#include "forward_renderer.h"
#include "mesh_plot.h"
#include "math.h"
#include "axis_renderer.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_my_types.h"
#include "geometry.h"
#include "bezier_curve.h"
#include "filedialog.h"
#include "network/server.h"
#include "network/serializer.h"
#include "timer.h"
#include "random.h"
#include "file_io.h"
#include "wave_propagation_simulation.h"
#include "action_potential.h"
#include "probe.h"


using namespace Eigen;

const LookAtCamera default_camera({ 0, 0, 5 }, { 0, 0, 0 }, { 0, 1, 0 }, (float)(45*PI/180), 1.0f, 0.1f, 1000.0f);

static bool dump_probes_values_to_csv(
	const char* file_name,
	const Eigen::Vector3<Real>& dipole_pos,
	const BezierCurve& dipole_vec_curve,
	const Real dt,
	const int sample_count,
	const std::vector<Probe>& probes,
	const Eigen::MatrixX<Real> probes_values)
{
	FILE* file = fopen(file_name, "w");
	if (!file)
	{
		return false;
	}

	std::string line = "";

	// column names
	line += "sample, time, ";
	line += "dipole_posx, dipole_posy, dipole_posz, ";
	line += "dipole_vecx, dipole_vecy, dipole_vecz, ";
	for (int i = 0; i < probes_values.rows(); i++)
	{
		if (i != 0)
		{
			line += ", ";
		}
		line += probes[i].name;
	}
	line += "\n";
	fwrite(line.c_str(), sizeof(char), line.size(), file);

	// values
	for (int sample = 0; sample < sample_count; sample++)
	{
		Real t = sample*dt;
		Eigen::Vector3<Real> dipole_vec = dipole_vec_curve.point_at(t);

		line = "";
		line += std::to_string(sample) + ", " + std::to_string(t) + ", ";
		line += std::to_string(dipole_pos[0]) + ", " + std::to_string(dipole_pos[1]) + ", " + std::to_string(dipole_pos[2]) + ", ";
		line += std::to_string(dipole_vec[0]) + ", " + std::to_string(dipole_vec[1]) + ", " + std::to_string(dipole_vec[2]) + ", ";

		// probe values
		for (int i = 0; i < probes_values.rows(); i++)
		{
			if (i != 0)
			{
				line += ", ";
			}
			line += std::to_string(probes_values(i, sample));
		}
		line += "\n";
		fwrite(line.c_str(), sizeof(char), line.size(), file);
	}

	fclose(file);
	return true;
}

enum RequestType
{
	REQUEST_GET_VALUES = 1,
	REQUEST_SET_DIPOLE_VECTOR = 2,
	REQUEST_GET_PROBES_NAMES = 3,
	REQUEST_CALCULATE_VALUES_FOR_VECTOR = 4,
	REQUEST_CALCULATE_VALUES_FOR_RANDOM_VECTORS = 5,
	REQUEST_SET_DIPOLE_VECTOR_VALUES = 6,
	REQUEST_GET_TMP_BSP_VALUES = 7,
	REQUEST_SET_TMP_VALUES = 8,
	REQUEST_GET_TMP_BSP_VALUES_PROBES = 9,
	REQUEST_GET_TMP_BSP_VALUES_PROBES_2 = 10,
	REQUEST_GET_TMP_BSP_VALUES_PROBES_TRAIN = 11,
};

static std::string request_type_to_string(const RequestType req_type)
{
	switch (req_type)
	{
	case REQUEST_GET_VALUES:
		return "REQUEST_GET_VALUES";
	case REQUEST_SET_DIPOLE_VECTOR:
		return "REQUEST_SET_DIPOLE_VECTOR";
	case REQUEST_GET_PROBES_NAMES:
		return "REQUEST_GET_PROBES_NAMES";
	case REQUEST_CALCULATE_VALUES_FOR_VECTOR:
		return "REQUEST_CALCULATE_VALUES_FOR_VECTOR";
	case REQUEST_CALCULATE_VALUES_FOR_RANDOM_VECTORS:
		return "REQUEST_CALCULATE_VALUES_FOR_RANDOM_VECTORS";
	case REQUEST_SET_DIPOLE_VECTOR_VALUES:
		return "REQUEST_SET_DIPOLE_VECTOR_VALUES";
	case REQUEST_GET_TMP_BSP_VALUES:
		return "REQUEST_GET_TMP_BSP_VALUES";
	case REQUEST_SET_TMP_VALUES:
		return "REQUEST_SET_TMP_VALUES";
	case REQUEST_GET_TMP_BSP_VALUES_PROBES:
		return "REQUEST_GET_TMP_BSP_VALUES_PROBES";
	case REQUEST_GET_TMP_BSP_VALUES_PROBES_2:
		return "REQUEST_GET_TMP_BSP_VALUES_PROBES_2";
	case REQUEST_GET_TMP_BSP_VALUES_PROBES_TRAIN:
		return "REQUEST_GET_TMP_BSP_VALUES_PROBES_TRAIN";
	default:
		return "UNKNOWN";
	}
}


enum ValuesSource
{
	VALUES_SOURCE_CONSTANT = 1,
	VALUES_SOURCE_BEZIER_CURVE = 2,
	VALUES_SOURCE_VALUES_LIST = 3
};

enum TMPValuesSource
{
	TMP_SOURCE_ACTION_POTENTIAL_PARAMETERS = 1,
	TMP_SOURCE_TMP_DIRECT_VALUES = 2,
	TMP_SOURCE_WAVE_PROPAGATION = 3,
};


static bool import_tmp_direct_values(const std::string& file_name, MatrixX<Real>& tmp_direct_values, int tmp_points_count)
{
	size_t contents_size;
	uint8_t* contents = file_read(file_name.c_str(), &contents_size);
	if (!contents)
	{
		return false;
	}

	Deserializer des(contents, contents_size);

	// parse

	// vertex count
	uint32_t rows_count = des.parse_u32();
	uint32_t cols_count = des.parse_u32();
	if (cols_count != tmp_points_count)
	{
		printf("TMP points count doesn't match\n");
		free(contents);
		return false;
	}

	MatrixX<Real> new_tmp_direct_values = MatrixX<Real>::Zero(rows_count, cols_count);

	for (int i = 0; i < rows_count; i++)
	{
		for (int j = 0; j < cols_count; j++)
		{
			new_tmp_direct_values(i, j) = des.parse_double();
		}
	}

	// set parameters
	tmp_direct_values = new_tmp_direct_values;

	free(contents);
	return true;
}

static bool export_tmp_direct_values(const std::string& file_name, const MatrixX<Real>& tmp_direct_values)
{
	Serializer ser;

	// write

	// rows and columns count
	ser.push_u32(tmp_direct_values.rows());
	ser.push_u32(tmp_direct_values.cols());

	for (int i = 0; i < tmp_direct_values.rows(); i++)
	{
		for (int j = 0; j < tmp_direct_values.cols(); j++)
		{
			ser.push_double(tmp_direct_values(i, j));
		}
	}

	return file_write(file_name.c_str(), ser.get_data());
}

static bool export_tmp_bsp_values_csv(const std::string& file_name, const MatrixX<Real>& tmp_direct_values, const MatrixX<Real>& probes_values)
{
	FILE* file = fopen(file_name.c_str(), "w");
	if (!file)
	{
		return false;
	}

	std::string line = "";

	// column names
	// TMP values
	for (int i = 0; i < tmp_direct_values.cols(); i++)
	{
		if (i != 0)
		{
			line += ", ";
		}
		line += std::string("TMP_") + std::to_string(i);
	}
	// BSP probes values
	for (int i = 0; i < probes_values.rows(); i++)
	{
		if (i != 0)
		{
			line += ", ";
		}
		line += std::string("BSP_") + std::to_string(i);
	}
	line += "\n";
	fwrite(line.c_str(), sizeof(char), line.size(), file);

	// values
	for (int sample = 0; sample < tmp_direct_values.rows(); sample++)
	{
		line = "";
		
		// TMP values
		for (int i = 0; i < tmp_direct_values.cols(); i++)
		{
			if (i != 0)
			{
				line += ", ";
			}
			line += std::to_string(tmp_direct_values(sample, i));
		}
		// BSP probes values
		for (int i = 0; i < probes_values.rows(); i++)
		{
			if (i != 0)
			{
				line += ", ";
			}
			line += std::to_string(probes_values(i, sample));
		}

		line += "\n";
		fwrite(line.c_str(), sizeof(char), line.size(), file);
	}

	fclose(file);
	return true;
}

static bool dump_matrix_to_csv(const std::string& file_name, const std::vector<std::string>& names, const MatrixX<Real>& matrix)
{
	FILE* file = fopen(file_name.c_str(), "w");
	if (!file)
	{
		return false;
	}

	std::string line = "";

	// column names
	for (int i = 0; i < names.size(); i++)
	{
		if (i != 0)
		{
			line += ", ";
		}
		line += names[i];
	}
	line += "\n";
	fwrite(line.c_str(), sizeof(char), line.size(), file);

	// values
	for (int i = 0; i < matrix.rows(); i++)
	{
		line = "";

		// TMP values
		for (int j = 0; j < matrix.cols(); j++)
		{
			if (j != 0)
			{
				line += ", ";
			}
			line += std::to_string(matrix(i, j));
		}

		line += "\n";
		fwrite(line.c_str(), sizeof(char), line.size(), file);
	}

	fclose(file);
	return true;
}


enum DrawingMode
{
	//DRAW_ALL = 1,
	DRAW_REST_POTENTIAL = 1,
	DRAW_PEAK_POTENTIAL = 2,
	DRAW_DEPOLARIZATION_TIME = 3,
	DRAW_REPOLARIZATION_TIME = 4
};

static void print_progress_bar(int percentage, bool return_carriage = true)
{
	// limit percentage
	percentage = clamp_value<int>(percentage, 0, 100);

	// buffer the line before printing
	char buffer[128];
	int buffer_it = 0;

	// return carriage
	if (return_carriage)
	{
		buffer[buffer_it++] = '\r';
	}

	// progress bar
	buffer[buffer_it++] = '[';
	for (int i = 0; i < 100; i++)
	{
		buffer[buffer_it++] = (i < percentage) ? '=' : '-';
	}
	buffer[buffer_it++] = ']';

	// null termination
	buffer[buffer_it++] = '\0';

	printf("%s %d%%   ", buffer, percentage);
}

class ForwardECGApp
{
public:
	ForwardECGApp()
	{

	}

	~ForwardECGApp()
	{

	}

	int setup()
	{
		// create the window
		width = 800;
		height = 600;
		window = createOpenglWindow(width, height, "ForwardECG");
		if (!window)
		{
			printf("Couldn't create a window\n");
			glfwTerminate();
			return -1;
		}

		// main device and input
		hookInputCallbacks(window);
		gldev = createOpenglDevice(window);
		if (!gldev)
		{
			printf("Couldn't create an OpenGL context\n");
			return -1;
		}
		gdevSet(gldev);

		// axis renderer
		axis_renderer = new AxisRenderer(80, 80);

		// setup 2D renderer
		Renderer2D::init();
		const float view_size = 2;
		Renderer2D::setProjection(ortho(-view_size, view_size, view_size, -view_size, 1, -1));
		Renderer2D::setStyle(Renderer2D::Style(true, 2, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));

		// setup 3D renderer
		Renderer3D::init();
		Renderer3D::setStyle(Renderer3D::Style(true, 2, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));

		// initialize parameters
		t = 0;
		heart_pos = { 0.07, 0.4, 0.05 };
		dipole_pos = { 0.07, 0.4, 0.05 };
		dipole_vec = { 0, 0, 0 };
		air_conductivity = 0;
		toso_conductivity = 1;
		heart_conductivity = 10;

		// heart mesh plot: TODO: Add load_heart_model function or append it to load_torso_model
		heart_mesh = load_mesh_plot("models/heart_model_5_fixed.fbx");
		heart_action_potential_params.resize(heart_mesh->vertices.size(), 
			ActionPotentialParameters{ ACTION_POTENTIAL_RESTING_POTENTIAL, ACTION_POTENTIAL_PEAK_POTENTIAL, ACTION_POTENTIAL_DEPOLARIZATION_TIME, ACTION_POTENTIAL_REPOLARIZATION_TIME });

		// torso mesh plot
		torso = new MeshPlot();
		load_torso_model("models/torso_model_fixed.fbx");
		color_n = { 0, 0, 1, 1 };
		color_p = { 1, 0, 0, 1 };
		color_probes = { 0, 0.25, 0, 1 };

		// wave propagation
		wave_prop.set_mesh(heart_mesh, heart_pos);
		

		// mesh plot renderer
		mpr = new MeshPlotRenderer;

		// frame buffers
		torso_fb = glFrameBuffer::create({ glTexture::create(width, height, FORMAT_RGBA, TYPE_FLOAT) }, glTexture::create(width, height, Format::FORMAT_DEPTH, Type::TYPE_FLOAT));

		// LookAtCamera
		camera = default_camera;

		// setup ImGUI
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO();
		io.IniFilename = NULL;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init("#version 330 core");

		// setup server
		initialize_socket();
		if (!server.start((server_address_select == 0) ? ADDRESS_LOCALHOST : ADDRESS_THISHOST, server_port))
		{
			printf("Failed to start the server\n");
		}

		return 0;
	}

	void run()
	{
		// enable v-sync
		glfwSwapInterval(1);

		// main loop
		timer.start();
		while (!glfwWindowShouldClose(window))
		{
			// handle window size change
			glfwGetWindowSize(window, &width, &height);
			gldev->resizeBackbuffer(width, height);
			gldev->viewport(0, 0, width, height);
			float aspect = (float)width / (float)height;
			camera.aspect = aspect;
			// resize frame buffer with window resize
			torso_fb->resize(width, height);
			Input::setWindowSize(width, height);


			// timer
			timer_dt = timer.elapsed_seconds();
			timer_time += timer_dt;
			timer.start();

			// input
			handle_input();


			// animate camera rotation
			if (camera_rotate)
			{
				camera_angle += timer_dt*camera_rotation_speed*2*PI;
				camera.eye = camera.look_at + glm::vec3(camera_eye_radius*sin(camera_angle), 0, camera_eye_radius*cos(camera_angle));
			}

			/*
			// animate dipole vector on the curve
			if (dipole_vec_source == VALUES_SOURCE_BEZIER_CURVE)
			{
				// update probes_values size
				sample_count = dipole_curve.total_duration()/dt + 1;
				probes_values.resize(probes.size(), sample_count);

				for (int step = 0; step < steps_per_frame; step++)
				{
					// next sample
					current_sample++;
					current_sample = current_sample%sample_count;

					// update dipole vector
					t = current_sample*dt;
					dipole_vec = dipole_curve.point_at(t);

					// calculate
					calculate_potentials();

					// calculate probes values at time point
					for (int i = 0; i < probes.size(); i++)
					{
						Real probe_value = evaluate_probe(*torso, probes[i]);
						probes_values(i, current_sample) = probe_value;
					}

					// clear probes graph
					if (probes_graph_clear_at_t0 && current_sample == 0)
					{
						probes_values.setZero();
					}
				}
			}
			else if (dipole_vec_source == VALUES_SOURCE_VALUES_LIST)
			{
				probes_values.resize(probes.size(), dipole_vec_values_list.size());

				if (dipole_vec_values_list.size() > 0)
				{
					dipole_vec_values_list_counter++;
					if (dipole_vec_values_list_counter > dipole_vec_values_list_change_rate)
					{
						dipole_vec_values_list_current++;
						dipole_vec_values_list_counter = 0;
					}
					dipole_vec_values_list_current = dipole_vec_values_list_current % dipole_vec_values_list.size();

					dipole_vec = dipole_vec_values_list[dipole_vec_values_list_current];
				}
				else
				{
					dipole_vec = { 0, 0, 0 };
				}

				calculate_potentials();

				// calculate probes values at time point
				for (int i = 0; i < probes.size(); i++)
				{
					Real probe_value = evaluate_probe(*torso, probes[i]);
					probes_values(i, dipole_vec_values_list_current) = probe_value;
				}

				// clear probes graph
				if (probes_graph_clear_at_t0 && dipole_vec_values_list_current == 0)
				{
					probes_values.setZero();
				}
			}
			else if (dipole_vec_source == VALUES_SOURCE_CONSTANT)
			{
				calculate_potentials();
			}
			*/


			// recalculate tmp_probes_interpolation_matrix
			if (last_heart_probes_count != heart_probes.size() || last_interpolation_power != interpolation_power)
			{
				tmp_probes_interpolation_matrix = MatrixX<Real>::Zero(M, heart_probes.size());

				for (int i = 0; i < M; i++)
				{
					Vector3<Real> vertex_pos = glm2eigen(heart_mesh->vertices[i].pos);

					// calculate each factor
					for (int j = 0; j < heart_probes.size(); j++)
					{
						Real factor = 1/pow((heart_probes[j].point-vertex_pos).norm(), interpolation_power);
						//factor = rmin(factor, 1e15);
						tmp_probes_interpolation_matrix(i, j) = factor;
					}

					// calculate the sum
					Real sum = 0;
					for (int j = 0; j < heart_probes.size(); j++)
					{
						sum += tmp_probes_interpolation_matrix(i, j);
					}

					// apply scale
					for (int j = 0; j < heart_probes.size(); j++)
					{
						tmp_probes_interpolation_matrix(i, j) /= sum;
					}
				}

				last_heart_probes_count = heart_probes.size();
				last_interpolation_power = interpolation_power;
			}

			// TMP action potential
			TMP_update_refresh_rate_counter++;
			if (TMP_update_refresh_rate_counter >= TMP_update_refresh_rate)
			{
				TMP_update_refresh_rate_counter = 0;
				// Potential calculations
				if (tmp_source == TMP_SOURCE_ACTION_POTENTIAL_PARAMETERS)
				{
					// update probes_values size
					sample_count = TMP_total_duration/TMP_dt + 1;
					probes_values.resize(probes.size(), sample_count);
					heart_probes_values.resize(heart_probes.size(), sample_count);
					// heart_probes_values_temp
					heart_probes_values_temp.resize(1, heart_probes.size());

					for (int step = 0; step < TMP_steps_per_frame; step++)
					{
						// next sample
						current_sample++;
						current_sample = current_sample%sample_count;

						// update time vector
						t = current_sample*TMP_dt;

						// update heart TMP from action potential parameters
						for (int i = 0; i < heart_mesh->vertices.size(); i++)
						{
							QH(i) = extracellular_potential(t, TMP_dt, heart_action_potential_params[i]); //action_potential_value_2
						}

						if (use_interpolation_for_action_potential)
						{	
							// update heart TMP from action potential parameters
							for (int i = 0; i < M; i++)
							{
								QH(i) = extracellular_potential(t, dt, heart_action_potential_params[i]); //action_potential_value_2
							}

							// update heart probes values
							for (int i = 0; i < heart_mesh->vertices.size(); i++)
							{
								heart_mesh->vertices[i].value = QH(i);
							}
							for (int i = 0; i < heart_probes.size(); i++)
							{
								heart_probes_values_temp(i) = evaluate_probe(*heart_mesh, heart_probes[i]);
							}

							// use interpolation for heart potentials using heart probes
							if (heart_probes.size() > 0)
							{
								QH = tmp_probes_interpolation_matrix*heart_probes_values_temp;
							}
							else
							{
								// set values to 0
								for (int i = 0; i < heart_mesh->vertices.size(); i++)
								{
									QH(i) = 0;
								}
							}
						}

						// calculate body surface potentials
						calculate_torso_potentials();

						// update probes
						for (int i = 0; i < heart_probes.size(); i++)
						{
							heart_probes_values(i, current_sample) = evaluate_probe(*heart_mesh, heart_probes[i]);
						}

						// calculate probes values at time point
						for (int i = 0; i < probes.size(); i++)
						{
							probes_values(i, current_sample) = evaluate_probe(*torso, probes[i]);
						}

						// clear probes graph
						if (probes_graph_clear_at_t0 && current_sample == 0)
						{
							heart_probes_values.setZero();
							probes_values.setZero();
						}
					}
				}
				else if (tmp_source == TMP_SOURCE_TMP_DIRECT_VALUES)
				{
					// update probes_values size
					sample_count = tmp_direct_values.rows() > 0 ? tmp_direct_values.rows() : 1;
					probes_values.resize(probes.size(), sample_count);
					heart_probes_values.resize(heart_probes.size(), sample_count);

					for (int step = 0; step < TMP_steps_per_frame; step++)
					{
						// next sample
						if (!(tmp_direct_values_one_play && current_sample == sample_count-1))
						{
							current_sample++;
						}
						current_sample = current_sample%sample_count;

						// update time vector
						t = current_sample*TMP_dt;

						// assign direct values (from probes interpolation)
						if (tmp_direct_values.rows() > 0)
						{
							QH = tmp_probes_interpolation_matrix*tmp_direct_values.row(current_sample).transpose();
						}
						else
						{
							// set values to 0
							for (int i = 0; i < heart_mesh->vertices.size(); i++)
							{
								QH(i) = 0;
							}
						}

						// calculate body surface potentials
						calculate_torso_potentials();

						// update probes
						for (int i = 0; i < heart_probes.size(); i++)
						{
							heart_probes_values(i, current_sample) = evaluate_probe(*heart_mesh, heart_probes[i]);
						}

						// calculate probes values at time point
						for (int i = 0; i < probes.size(); i++)
						{
							probes_values(i, current_sample) = evaluate_probe(*torso, probes[i]);
						}

						// clear probes graph
						if (probes_graph_clear_at_t0 && current_sample == 0)
						{
							heart_probes_values.setZero();
							probes_values.setZero();
						}
					}
				}
				else if (tmp_source == TMP_SOURCE_WAVE_PROPAGATION)
				{
					wave_prop.simulation_step();
					sample_count = wave_prop.get_sample_count();
					current_sample = wave_prop.get_current_sample();
					//wave_prop.update_mesh_values();
					QH = wave_prop.get_potentials();
					probes_values.resize(probes.size(), sample_count);
					heart_probes_values.resize(heart_probes.size(), sample_count);

					// calculate body surface potentials
					calculate_torso_potentials();

					// update probes
					for (int i = 0; i < heart_probes.size(); i++)
					{
						heart_probes_values(i, current_sample) = evaluate_probe(*heart_mesh, heart_probes[i]);
					}

					// calculate probes values at time point
					for (int i = 0; i < probes.size(); i++)
					{
						probes_values(i, current_sample) = evaluate_probe(*torso, probes[i]);
					}

					// clear probes graph
					if (probes_graph_clear_at_t0 && current_sample == 0)
					{
						heart_probes_values.setZero();
						probes_values.setZero();
					}
				}
			}


			// render
			render();

			// swap buffers
			glfwSwapBuffers(window);

			// poll events
			Input::newFrame();
			glfwPollEvents();

			// handle server requests
			handle_server_requests();
		}

		// cleanup
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
		if (torso)
		{
			delete torso;
		}
		Renderer2D::cleanup();
		Renderer3D::cleanup();
		delete axis_renderer;
		delete mpr;
		delete gldev;
		glfwDestroyWindow(window);
		glfwTerminate();
		server.stop();
	}

private:
	bool load_torso_model(const std::string& model_path)
	{
		torso_model_path = model_path;

		MeshPlot* new_torso = load_mesh_plot(model_path.c_str());
		if (!new_torso)
		{
			return false;
		}

		torso = new_torso;

		// vertices count
		N = torso->vertices.size();
		M = heart_mesh->vertices.size();

		// set matrices sizes
		A = MatrixX<Real>(M, M);
		B = MatrixX<Real>(M, 1);
		Q = MatrixX<Real>::Zero(M, 1);
		IA_inv = MatrixX<Real>(M, M);
		QH = MatrixX<Real>::Zero(M, 1);

		// DEBUG
		printf("Loading torso model: Vertex count: Troso: %d vertex  \tHeart: %d vertex\n", N, M);

		// set conductivities inside and outside each boundary
		torso_sigma_p = air_conductivity; // outside the torso
		torso_sigma_n = toso_conductivity; // inside the torso
		heart_sigma_p = toso_conductivity; // outside the heart
		heart_sigma_n = heart_conductivity; // inside the heart

		// calculate IA_inv
		Timer t;
		t.start();
		calculate_transfer_matrix();
		printf("Calculated transfer matrix in: %.3f seconds\n", t.elapsed_seconds());


		return true;
	}

	void calculate_transfer_matrix()
	{
		/*
		// BEM solver (bounded conductor) for the heart and dipole
		// Q = B - AQ
		// (I+A)Q = B
		A = MatrixX<Real>::Zero(M, M);

		// Heart Vertices
		for (int i = 0; i < heart_mesh->vertices.size(); i++)
		{
			// vertex position
			const MeshPlotVertex& vertex = heart_mesh->vertices[i];
			Vector3<Real> r = heart_pos + glm2eigen(vertex.pos);

			// A: For heart faces
			for (const MeshPlotFace& face : heart_mesh->faces)
			{
				Vector3<Real> a = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[0]].pos);
				Vector3<Real> b = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[1]].pos);
				Vector3<Real> c = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Real const_val = 1/(4*PI)*2*(heart_sigma_n-heart_sigma_p)/(heart_sigma_n+heart_sigma_p)*1/pow((r-center).norm(), 3) * (r-center).dot(face_normal)*area;

				A(i, face.idx[0]) += const_val/3;
				A(i, face.idx[1]) += const_val/3;
				A(i, face.idx[2]) += const_val/3;
			}
		}

		// (I+A)'
		IA_inv = (MatrixX<Real>::Identity(M, M) + A).inverse();
		*/


		// BEM solver (bounded conductor with defined TMP distribution)
		// Matrices derived from potentials at the torso

		// PBB (NxN)
		MatrixX<Real> PBB = MatrixX<Real>::Zero(N, N);
		for (int i = 0; i < N; i++)
		{
			// vertex position
			const MeshPlotVertex& vertex = torso->vertices[i];
			Vector3<Real> r = glm2eigen(vertex.pos);

			// A: For torso faces
			for (const MeshPlotFace& face : torso->faces)
			{
				Vector3<Real> a = glm2eigen(torso->vertices[face.idx[0]].pos);
				Vector3<Real> b = glm2eigen(torso->vertices[face.idx[1]].pos);
				Vector3<Real> c = glm2eigen(torso->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Vector3<Real> r_vec = r-center; // r-c
				//Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), 2)); // omega = (r^.n^ * ds)/(r*r)
				Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), r_power)); // omega = (r^.n^ * ds)/(r*r)
				Real const_val = 1/(4*PI)*solid_angle;

				// skip for close region triangles
				if ((center-r).norm() < close_range_threshold)
				{
					continue;
				}

				// ignore negative dot product
				if (ignore_negative_dot_product && r_vec.dot(face_normal) < 0)
				{
					continue;
				}

				PBB(i, face.idx[0]) += const_val/3;
				PBB(i, face.idx[1]) += const_val/3;
				PBB(i, face.idx[2]) += const_val/3;
			}

			PBB(i, i) = PBB(i, i) - 1; // TODO: CHECK    PBB(i, i) = -1;
			//PBB(i, i) = -1;
		}

		// PBH (NxM)
		MatrixX<Real> PBH = MatrixX<Real>::Zero(N, M);
		for (int i = 0; i < N; i++)
		{
			// vertex position
			const MeshPlotVertex& vertex = torso->vertices[i];
			Vector3<Real> r = glm2eigen(vertex.pos);

			// A: For heart faces
			for (const MeshPlotFace& face : heart_mesh->faces)
			{
				Vector3<Real> a = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[0]].pos);
				Vector3<Real> b = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[1]].pos);
				Vector3<Real> c = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Vector3<Real> r_vec = r-center; // r-c
				//Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), 2)); // omega = (r^.n^ * ds)/(r*r)
				Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), r_power)); // omega = (r^.n^ * ds)/(r*r)
				Real const_val = -1/(4*PI)*solid_angle;

				// ignore negative dot product
				if (ignore_negative_dot_product && r_vec.dot(face_normal) < 0)
				{
					continue;
				}

				PBH(i, face.idx[0]) += const_val/3;
				PBH(i, face.idx[1]) += const_val/3;
				PBH(i, face.idx[2]) += const_val/3;
			}
		}

		// GBH (NxM)
		MatrixX<Real> GBH = MatrixX<Real>::Zero(N, M);
		for (int i = 0; i < N; i++)
		{
			// vertex position
			const MeshPlotVertex& vertex = torso->vertices[i];
			Vector3<Real> r = glm2eigen(vertex.pos);

			// A: For heart faces
			for (const MeshPlotFace& face : heart_mesh->faces)
			{
				Vector3<Real> a = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[0]].pos);
				Vector3<Real> b = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[1]].pos);
				Vector3<Real> c = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Vector3<Real> r_vec = r-center; // r-c
				//Real const_val = -1/(4*PI) * area;
				Real const_val = -1/(4*PI) / r_vec.norm();

				GBH(i, face.idx[0]) += const_val/3;
				GBH(i, face.idx[1]) += const_val/3;
				GBH(i, face.idx[2]) += const_val/3;
			}
		}


		// Matrices derived from potentials at the heart

		// PHB (MxN)
		MatrixX<Real> PHB = MatrixX<Real>::Zero(M, N);
		for (int i = 0; i < M; i++)
		{
			// vertex position
			const MeshPlotVertex& vertex = heart_mesh->vertices[i];
			Vector3<Real> r = heart_pos + glm2eigen(vertex.pos);

			// A: For torso faces
			for (const MeshPlotFace& face : torso->faces)
			{
				Vector3<Real> a = glm2eigen(torso->vertices[face.idx[0]].pos);
				Vector3<Real> b = glm2eigen(torso->vertices[face.idx[1]].pos);
				Vector3<Real> c = glm2eigen(torso->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Vector3<Real> r_vec = r-center; // r-c
				//Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), 2)); // omega = (r^.n^ * ds)/(r*r)
				Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), r_power)); // omega = (r^.n^ * ds)/(r*r)
				Real const_val = 1/(4*PI)*solid_angle;

				// ignore negative dot product
				if (ignore_negative_dot_product && r_vec.dot(face_normal) < 0)
				{
					continue;
				}

				PHB(i, face.idx[0]) += const_val/3;
				PHB(i, face.idx[1]) += const_val/3;
				PHB(i, face.idx[2]) += const_val/3;
			}
		}

		// PHH (MxM)
		MatrixX<Real> PHH = MatrixX<Real>::Zero(M, M);
		for (int i = 0; i < M; i++)
		{
			// vertex position
			const MeshPlotVertex& vertex = heart_mesh->vertices[i];
			Vector3<Real> r = heart_pos + glm2eigen(vertex.pos);

			// A: For heart faces
			for (const MeshPlotFace& face : heart_mesh->faces)
			{
				Vector3<Real> a = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[0]].pos);
				Vector3<Real> b = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[1]].pos);
				Vector3<Real> c = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Vector3<Real> r_vec = r-center; // r-c
				//Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), 2)); // omega = (r^.n^ * ds)/(r*r)
				Real solid_angle = r_vec.normalized().dot(face_normal)*area / (pow(r_vec.norm(), r_power)); // omega = (r^.n^ * ds)/(r*r)
				Real const_val = -1/(4*PI)*solid_angle;

				// skip for close region triangles
				if ((center-r).norm() < close_range_threshold)
				{
					continue;
				}

				// ignore negative dot product
				if (ignore_negative_dot_product && r_vec.dot(face_normal) < 0)
				{
					continue;
				}

				PHH(i, face.idx[0]) += const_val/3;
				PHH(i, face.idx[1]) += const_val/3;
				PHH(i, face.idx[2]) += const_val/3;
			}

			PHH(i, i) = PHH(i, i) - 1; // TODO: CHECK    PHH(i, i) = -1;
			//PHH(i, i) = -1;
		}

		// GHH (MxM)
		MatrixX<Real> GHH = MatrixX<Real>::Zero(M, M);
		for (int i = 0; i < M; i++)
		{
			// vertex position
			const MeshPlotVertex& vertex = heart_mesh->vertices[i];
			Vector3<Real> r = heart_pos + glm2eigen(vertex.pos);

			// A: For heart faces
			for (const MeshPlotFace& face : heart_mesh->faces)
			{
				Vector3<Real> a = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[0]].pos);
				Vector3<Real> b = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[1]].pos);
				Vector3<Real> c = heart_pos + glm2eigen(heart_mesh->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Vector3<Real> r_vec = r-center; // r-c
				//Real const_val = -1/(4*PI) * area;
				Real const_val = -1/(4*PI) / r_vec.norm();

				// skip for close region triangles
				if ((center-r).norm() < close_range_threshold)
				{
					continue;
				}

				GHH(i, face.idx[0]) += const_val/3;
				GHH(i, face.idx[1]) += const_val/3;
				GHH(i, face.idx[2]) += const_val/3;
			}
		}


		/*
		// DEBUG
		auto print_matrix_info = [](const char* name, const MatrixX<Real>& mat)
		{
			Real min_val = 1e20;
			Real max_val = 1e-20;
			for (int i = 0; i < mat.rows()*mat.cols(); i++)
			{
				min_val = rmin(min_val, mat(i));
				max_val = rmax(max_val, mat(i));
			}

			printf("%s: %d x %d\n    min: %f    max: %f\n", name, mat.rows(), mat.cols(), min_val, max_val);
		};
		print_matrix_info("PBB", PBB);
		print_matrix_info("PBH", PBH);
		print_matrix_info("GBH", GBH);
		print_matrix_info("PHB", PHB);
		print_matrix_info("PHH", PBB);
		print_matrix_info("GHH", PBB);
		//printf("PBB: %d x %d\n", PBB.rows(), PBB.cols());
		//printf("PBH: %d x %d\n", PBH.rows(), PBH.cols());
		//printf("GBH: %d x %d\n", GBH.rows(), GBH.cols());
		//printf("PHB: %d x %d\n", PHB.rows(), PHB.cols());
		//printf("PHH: %d x %d\n", PHH.rows(), PHH.cols());
		//printf("GHH: %d x %d\n", GHH.rows(), GHH.cols());
		*/

		// ZBH (NxM) : heart potentials to torso potentials transfer matrix
		// ZBH = (PBB - GBH*GHH^-1*PHB)^-1 * (GBH*GHH^-1*PHH - PBH)
		// Q_B = ZBH * Q_H
		//ZBH = (PBB - GBH*GHH.inverse()*PHB).inverse() * (GBH*GHH.inverse()*PHH - PBH); // with potential gradient effect

		// TODO: Restore
		
		// ZBH = PBB^-1 * PBH
		ZBH = PBB.inverse() * PBH; // without potential gradient effect
		
		//ZBH = MatrixX<Real>::Zero(N, M);

		//ZBH = PBH; // without potential gradient effect

		// TODO: try to fix the first equation.

		// DEBUG:
		//printf("%d %d\n", ZBH.rows(), ZBH.cols());
		//for (int i = 0; i < ZBH.rows(); i++)
		//{
		//	for (int j = 0; j < ZBH.cols(); j++)
		//	{
		//		printf("%.3f ", ZBH(i, j));
		//	}
		//}
	}

	void calculate_torso_potentials()
	{
		// TODO: DELETE
		/*
		// heart potentials (TMP) calculation based on dipole vector
		// Heart vertices
		for (int i = 0; i < heart_mesh->vertices.size(); i++)
		{
			const MeshPlotVertex& vertex = heart_mesh->vertices[i];
			Vector3<Real> r = heart_pos + glm2eigen(vertex.pos);

			// B
			Real Q_inf = 1/(4*PI*heart_conductivity) * 1/pow((r-dipole_pos).norm(), 3) * (r-dipole_pos).dot(dipole_vec); // ATTENTION: TAKE CARE OF THE DIFFERENCE
			B(i) = 2*heart_conductivity/(heart_sigma_n+heart_sigma_p)*Q_inf;
		}

		// calculate potentials
		Q = IA_inv * B;

		// zero heart right section (x < 0)
		if (zero_heart_right_section)
		{
			for (int i = 0; i < heart_mesh->vertices.size(); i++)
			{
				const MeshPlotVertex& vertex = heart_mesh->vertices[i];
				if (vertex.pos.x < 0)
				{
					Q(i) = 0;
				}
			}
		}

		QH = Q;
		*/

		/*
		// DEBUG
		if (zero_heart_right_section)
		{
			for (int i = 0; i < heart_mesh->vertices.size(); i++)
			{
				QH(i) = 1;
			}
		}
		*/

		// TMP forward ecg
		// Q_B = ZBH * Q_H
		QB = ZBH * QH;


		//// debug
		//if (Input::isKeyDown(GLFW_KEY_I))
		//{
		//	Q = IA_inv * B;
		//}
		//else
		//{
		//	Q = B;
		//}


		// update torso potentials
		for (int i = 0; i < torso->vertices.size(); i++)
		{
			torso->vertices[i].value = QB(i);
		}

		// update heart potentials
		for (int i = 0; i < heart_mesh->vertices.size(); i++)
		{
			heart_mesh->vertices[i].value = QH(i);
		}

		/*
		// apply reference probe (to potentials in toso model only not Q)
		if (reference_probe != -1)
		{
			Real reference_value = evaluate_probe(*torso, probes[reference_probe]);
			// Torso
			for (int i = 0; i < torso->vertices.size(); i++)
			{
				torso->vertices[i].value -= reference_value;
			}
			// Heart
			for (int i = 0; i < heart_mesh->vertices.size(); i++)
			{
				heart_mesh->vertices[i].value -= reference_value;
			}
		}
		*/

	}

	void render()
	{
		// clear buffers
		gldev->clearColorBuffer(color_background.r, color_background.g, color_background.b, color_background.a);
		gldev->depthTest(STATE_ENABLED);
		gldev->depthFunc(COMPARISON_LESS);
		gldev->clearDepthBuffer(1.0);

		// set alpha mode
		gldev->setAlpha(Alpha{ true, Alpha::SRC_ALPHA, Alpha::ONE_MINUS_SRC_ALPHA });


		// calculate torso maximum and minimum values
		Real torso_potential_max = -1e12;
		Real torso_potential_min = 1e12;
		for (MeshPlotVertex& vertex : torso->vertices)
		{
			torso_potential_min = rmin(torso_potential_min, vertex.value);
			torso_potential_max = rmax(torso_potential_max, vertex.value);
		}
		// limit min and max to 0
		torso_potential_max = rmax(torso_potential_max, 0+1e-12);
		torso_potential_min = rmin(torso_potential_min, 0-1e-12);
		Real torso_potential_max_abs = rmax(rabs(torso_potential_max), rabs(torso_potential_min));
		torso_potential_max_abs = rmax(1e-12, torso_potential_max_abs);
		//torso_potential_max = torso_potential_max_abs;
		//torso_potential_min = -torso_potential_max_abs;

		Real max_abs_torso = rmax(rabs(torso_potential_max), rabs(torso_potential_min));

		// adaptive range
		if (torso_potential_adaptive_range)
		{
			if (torso_potential_adaptive_range_limit)
			{
				torso_potential_min_value = torso_potential_min;
				torso_potential_max_value = torso_potential_max;
				torso_potential_max_abs_value = max_abs_torso;
			}
			else
			{
				torso_potential_min_value = torso_potential_min;
				torso_potential_max_value = torso_potential_max;
				torso_potential_max_abs_value = max_abs_torso;
			}
		}
		else
		{
			torso_potential_min_value = rmin(torso_potential_min_value, torso_potential_min);
			torso_potential_max_value = rmax(torso_potential_max_value, torso_potential_max);
			// update torso_potential_max_abs_value
			torso_potential_max_abs_value = rmax(torso_potential_max_abs_value, rabs(max_abs_torso));
		}


		// update torso potential values at GPU
		torso->update_gpu_buffers();


		// view target channel
		if (drawing_values_enabled && drawing_view_target_channel)
		{
			// set value
			for (int i = 0; i < heart_mesh->vertices.size(); i++)
			{
				switch (drawing_values_mode)
				{
				case DRAW_REST_POTENTIAL:
					heart_mesh->vertices[i].value = heart_action_potential_params[i].resting_potential;
					break;
				case DRAW_PEAK_POTENTIAL:
					heart_mesh->vertices[i].value = heart_action_potential_params[i].peak_potential;
					break;
				case DRAW_DEPOLARIZATION_TIME:
					heart_mesh->vertices[i].value = heart_action_potential_params[i].depolarization_time;
					break;
				case DRAW_REPOLARIZATION_TIME:
					heart_mesh->vertices[i].value = heart_action_potential_params[i].repolarization_time;
					break;
				default:
					break;
				}
			}
		}

		// prepare renderer 2D
		Renderer3D::setStyle(Renderer3D::Style(true, 2, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));
		Renderer3D::setProjection(camera.calculateViewProjection());

		// render wave propagation
		if (tmp_source == TMP_SOURCE_WAVE_PROPAGATION)
		{
			wave_prop.render();
		}

		// calculate heart maximum and minimum values
		Real heart_potential_max = -1e12;
		Real heart_potential_min = 1e12;
		for (int i = 0; i < heart_mesh->vertices.size(); i++)
		{
			heart_potential_min = rmin(heart_potential_min, heart_mesh->vertices[i].value);
			heart_potential_max = rmax(heart_potential_max, heart_mesh->vertices[i].value);
		}

		heart_potential_max = rmax(heart_potential_max, 0+1e-12);
		heart_potential_min = rmin(heart_potential_min, 0-1e-12);
		// calculate maximum absolute value
		Real max_abs_heart = rmax(rabs(heart_potential_max), rabs(heart_potential_min));
		max_abs_heart = rmax(1e-12, max_abs_heart);

		// calculate heart maximum and minimum values for the scale
		if (tmp_source == TMP_SOURCE_WAVE_PROPAGATION && !wave_prop.is_mesh_in_preview())
		{
			//heart_potential_max_value = -1e12;
			//heart_potential_min_value = 1e12;
			heart_potential_min_value = rmin(heart_potential_min_value, heart_potential_min);
			heart_potential_max_value = rmax(heart_potential_max_value, heart_potential_max);

			// update heart_potential_max_abs_value
			heart_potential_max_abs_value = rmax(heart_potential_max_abs_value, rabs(max_abs_heart));
		}
		else if (tmp_source == TMP_SOURCE_TMP_DIRECT_VALUES)
		{
			//heart_potential_max_value = -1e12;
			//heart_potential_min_value = 1e12;
			heart_potential_min_value = rmin(heart_potential_min_value, heart_potential_min);
			heart_potential_max_value = rmax(heart_potential_max_value, heart_potential_max);

			// update heart_potential_max_abs_value
			heart_potential_max_abs_value = rmax(heart_potential_max_abs_value, rabs(max_abs_heart));
		}

		// adaptive range
		if (heart_potential_adaptive_range)
		{
			heart_potential_min_value = heart_potential_min;
			heart_potential_max_value = heart_potential_max;
			heart_potential_max_abs_value = max_abs_heart;
		}

		// update torso potential values at GPU
		heart_mesh->update_gpu_buffers();


		// set mesh plot ambient
		mpr->set_ambient(mesh_plot_ambient);
		mpr->set_specular(mesh_plot_specular);

		// render heart mesh plot
		mpr->set_view_projection_matrix(camera.calculateViewProjection());
		if (tmp_source == TMP_SOURCE_WAVE_PROPAGATION && wave_prop.is_mesh_in_preview())
		{
			mpr->set_values_range(wave_prop.get_mesh_in_preview_min(), wave_prop.get_mesh_in_preview_max());
		}
		else if (heart_use_separate_min_max_range)
		{
			mpr->set_values_range(heart_potential_min_value, heart_potential_max_value);
		}
		else
		{
			mpr->set_values_range(-heart_potential_max_abs_value, heart_potential_max_abs_value);
		}
		mpr->render_mesh_plot(translate(eigen2glm(heart_pos))*scale(glm::vec3(heart_render_scale)), heart_mesh);

		// render torso to torso_fb
		torso_fb->bind();
		gldev->clearColorBuffer(0, 0, 0, 0);
		gldev->clearDepthBuffer(1.0);
		const Real alpha = 1;
		mpr->set_colors(color_p, color_n);
		mpr->set_view_projection_matrix(camera.calculateViewProjection());
		if (torso_use_separate_min_max_range)
		{
			mpr->set_values_range(torso_potential_min_value, torso_potential_max_value);
		}
		else
		{
			mpr->set_values_range(-torso_potential_max_abs_value, torso_potential_max_abs_value);
		}
		mpr->render_mesh_plot(glm::mat4(1), torso);
		torso_fb->unbind();
		gldev->bindBackbuffer();
		// render torso_fb texture
		Renderer2D::setProjection(ortho(0, width, height, 0, -1, 1));
		Renderer2D::drawTexture({ width/2, height/2 }, { width, height }, torso_fb->getColorTexture(0), {1, 1, 1, torso_opacity});

		/*
		// render dipole
		gldev->depthTest(STATE_DISABLED); // disable depth testing
		Renderer3D::setStyle(Renderer3D::Style(true, dipole_vector_thickness, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));
		Renderer3D::drawLine(eigen2glm(dipole_pos), eigen2glm(dipole_pos+dipole_vec*dipole_vector_scale));

		// render dipole locus
		if (dipole_vec_source == VALUES_SOURCE_BEZIER_CURVE && render_dipole_curve)
		{
			dipole_locus.resize(sample_count);
			for (int i = 0; i < sample_count; i++)
			{
				dipole_locus[i] = eigen2glm(dipole_pos + dipole_curve.point_at(i*dt)*dipole_vector_scale);
			}
			Renderer3D::setStyle(Renderer3D::Style(true, 1, { 1, 1, 1, 1 }, false, { 0.75, 0, 0 ,1 }));
			Renderer3D::drawPolygon(&dipole_locus[0], dipole_locus.size(), false);
		}

		// render bezier curve handles (if render_dipole_curve_lines is true)
		if (dipole_vec_source == VALUES_SOURCE_BEZIER_CURVE && render_dipole_curve && render_dipole_curve_lines)
		{
			Renderer3D::setStyle(Renderer3D::Style(true, 1, { 0.7, 0.7, 0.7, 1 }, false, { 0.75, 0, 0 ,1 }));
			for (int i = 0; i < dipole_curve.segments_duratoins.size(); i++)
			{
				// handle 1
				Renderer3D::drawLine(eigen2glm(dipole_pos+dipole_curve.points[i*3+0]*dipole_vector_scale),
					eigen2glm(dipole_pos+dipole_curve.points[i*3+1]*dipole_vector_scale));
				// handle 2
				Renderer3D::drawLine(eigen2glm(dipole_pos+dipole_curve.points[i*3+2]*dipole_vector_scale),
					eigen2glm(dipole_pos+dipole_curve.points[i*3+3]*dipole_vector_scale));
			}
		}

		// render dipole vector values list
		if (dipole_vec_source == VALUES_SOURCE_VALUES_LIST && render_dipole_vec_values_point)
		{
			for (int i = 0; i < dipole_vec_values_list.size(); i++)
			{
				Renderer3D::drawPoint(eigen2glm(dipole_pos+dipole_vec_values_list[i]*dipole_vector_scale), { 0, 0, 0, 1 }, 4);
			}
		}
		if (dipole_vec_source == VALUES_SOURCE_VALUES_LIST && render_dipole_vec_values_vectors)
		{
			Renderer3D::setStyle(Renderer3D::Style(true, 1, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));
			for (int i = 0; i < dipole_vec_values_list.size(); i++)
			{
				Renderer3D::drawLine(eigen2glm(dipole_pos), eigen2glm(dipole_pos+dipole_vec_values_list[i]*dipole_vector_scale));
			}
		}
		if (dipole_vec_source == VALUES_SOURCE_VALUES_LIST && render_dipole_vec_values_locus && dipole_vec_values_list.size() > 1)
		{
			dipole_locus.resize(dipole_vec_values_list.size());
			for (int i = 0; i < dipole_vec_values_list.size(); i++)
			{
				dipole_locus[i] = eigen2glm(dipole_pos + dipole_vec_values_list[i]*dipole_vector_scale);
			}
			Renderer3D::setStyle(Renderer3D::Style(true, 1, { 1, 1, 1, 1 }, false, { 0.75, 0, 0 ,1 }));
			Renderer3D::drawPolygon(&dipole_locus[0], dipole_locus.size(), false);
		}
		*/

		gldev->depthTest(STATE_DISABLED); // disable depth testing

		// render probes
		for (int i = 0; i < probes.size(); i++)
		{
			glm::vec4 color = color_probes;
			if (i == reference_probe)
			{
				color = { 0, 0, 0, 1 };
			}
			if (i == current_selected_probe)
			{
				color = { 1, 1, 1, 1 };
			}
			Renderer3D::drawPoint(eigen2glm(probes[i].point), color, 4);
		}
		if (adding_probe && adding_probe_intersected)
		{
			Renderer3D::drawPoint(eigen2glm(adding_probe_intersection), { 0, 0, 0, 1 }, 4);
		}

		// render heart probes
		for (int i = 0; i < heart_probes.size(); i++)
		{
			glm::vec4 color = { 1, 1, 1, 1 };
			if (i == heart_current_selected_probe)
			{
				color = { 0, 0, 0, 1 };
			}
			Renderer3D::drawPoint(eigen2glm(heart_pos + heart_probes[i].point), color, 2);
		}
		if (heart_adding_probe && heart_adding_probe_intersected)
		{
			Renderer3D::drawPoint(eigen2glm(heart_pos + heart_adding_probe_intersection), { 0, 0, 0, 1 }, 2);
		}

		// render drawing preview
		if (drawing_values_enabled && drawing_values_preview.size() >= 2)
		{
			Renderer3D::setStyle(Renderer3D::Style(true, 1, { 1, 1, 1, 1 }, true, { 0.75, 0, 0, 0.2 }));
			drawing_values_preview.push_back(drawing_values_preview[0]);
			Renderer3D::drawPolygon(&drawing_values_preview[0], drawing_values_preview.size(), false);
		}

		// render axis
		gldev->depthTest(STATE_ENABLED); // enable depth testing
		axis_renderer->render(camera);
		gldev->depthTest(STATE_DISABLED); // disable depth testing
		Renderer2D::setProjection(ortho(0, width, height, 0, -1, 1));
		Renderer2D::drawTexture({ width-50, height-50 }, { 80, 80 }, axis_renderer->get_texture());
	
		render_gui();
	}

	void render_gui()
	{
		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();


		ImGui::Begin("Controls");

		// Geometry
		ImGui::Text("Geometry");
		// torso model
		ImGui::Text("Torso Model Path: %s", torso_model_path.c_str());
		ImGui::SameLine();
		if (ImGui::Button("..."))
		{
			// save file dialog
			std::string file_name = open_file_dialog("", "All\0*.*\0");

			// load torso model
			if (file_name != "")
			{
				if (load_torso_model(file_name))
				{
					printf("Loaded \"%s\" torso model\n", file_name.c_str());
				}
				else
				{
					printf("Failed to load \"%s\" torso model\n", file_name.c_str());
				}
			}
		}
		ImGui::DragVector3Eigen("Heart position", heart_pos, 0.01f);
		ImGui::InputReal("Heart scale", &heart_scale);
		// conductivities
		ImGui::InputReal("Air Conductivity", &air_conductivity, 0.01, 10);
		ImGui::InputReal("Torso Conductivity", &toso_conductivity, 0.01, 10);
		ImGui::InputReal("Heart Conductivity", &heart_conductivity, 0.01, 10);
		// transfer matrix parameters
		ImGui::InputReal("Close Range Threshold", &close_range_threshold, 0.01, 10);
		ImGui::InputReal("1/R Power", &r_power, 0.1, 20);
		ImGui::Checkbox("Ignore Faces Opposite To R Vector", &ignore_negative_dot_product);
		// recalculate coefficients matrix
		if (ImGui::Button("Recalculate Coefficients Matrix"))
		{
			calculate_transfer_matrix();
		}

		// server
		//server_address_select
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::Text("Server");
		static const char* binding_items_names[] = { "LocalHost", "ThisHost" };
		if (!server.is_running())
		{
			//ImGui::ListBox("Binding Address", &server_address_select, binding_items_names, 2);
			ImGui::Combo("Binding Address", &server_address_select, "LocalHost\0ThisHost\0", 2);
			if (ImGui::InputInt("Binding Port", &server_port))
			{
				server_port = clamp_value(server_port, 1, 0xFFFF);
			}
			if (ImGui::Button("Start Server"))
			{
				Address addr = (server_address_select == 0) ? ADDRESS_LOCALHOST : ADDRESS_THISHOST;
				if (!server.start(addr, server_port))
				{
					printf("Failed to start the server %s:%d\n", addr.to_string().c_str(), server_port);
				}
			}
		}
		else
		{
			ImGui::Text("\tBinding Address: %s", binding_items_names[server_address_select]);
			ImGui::Text("\tBinding Port: %d", server_port);
			if (ImGui::Button("Stop Server"))
			{
				if (!server.stop())
				{
					printf("Failed to stop the server\n");
				}
			}
		}


		// TMP Source
		{
			ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
			int im_tmp_source = (int)tmp_source - 1;
			ImGui::Combo("TMP Source", (int*)&im_tmp_source, "Action Potential Parameters\0Direct Values\0Wave Propagation\0", 3);
			tmp_source = (TMPValuesSource)clamp_value<int>(im_tmp_source+1, 1, 3);
		}
		if (tmp_source == TMP_SOURCE_ACTION_POTENTIAL_PARAMETERS)
		{
			ImGui::Text("Time: %.4f s", t);
			// action potential drawing drawing
			ImGui::Checkbox("draw values enable", &drawing_values_enabled);
			ImGui::Checkbox("draw values blur", &drawing_values_blur);
			ImGui::Checkbox("draw only vertices facing camera", &drawing_only_facing_camera);
			int im_drawing_values_mode_select = (int)drawing_values_mode - 1;
			ImGui::Combo("drawing target", (int*)&im_drawing_values_mode_select, "Rest Potential\0Peak Potential\0Depolarization Time\0Repolarization Time\0", 4);
			drawing_values_mode = (DrawingMode)clamp_value<int>(im_drawing_values_mode_select+1, 1, 4);
			ImGui::DragReal("drawing brush radius", &drawing_values_radius, 0.01, 0.001, 1);
			ImGui::InputReal("drawing value", &drawing_value);
			ImGui::Checkbox("view target channel", &drawing_view_target_channel);
			ImGui::Checkbox("use interpolation intermediate step", &use_interpolation_for_action_potential);
			if (ImGui::Button("Import parameters"))
			{
				// open file dialog
				std::string file_name = open_file_dialog("action_potential_parameters", "All\0*.*\0");

				// import
				if (file_name != "")
				{
					if (import_action_potential_parameters(file_name, heart_action_potential_params))
					{
						printf("Imported \"%s\" parameters\n", file_name.c_str());
					}
					else
					{
						printf("Failed to import \"%s\" parameters\n", file_name.c_str());
					}
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Export parameters"))
			{
				// save file dialog
				std::string file_name = save_file_dialog("action_potential_parameters", "All\0*.*\0");

				// export
				if (file_name != "")
				{
					if (export_action_potential_parameters(file_name, heart_action_potential_params))
					{
						printf("Exported \"%s\" parameters\n", file_name.c_str());
					}
					else
					{
						printf("Failed to export \"%s\" parameters\n", file_name.c_str());
					}
				}
			}
			if (ImGui::Button("Fix parameters dep/rep time"))
			{
				for (ActionPotentialParameters& param : heart_action_potential_params)
				{
					if (param.repolarization_time < param.depolarization_time + 0.050)
					{
						param.repolarization_time = param.depolarization_time + 0.050;
					}
				}
			}

			if (ImGui::Button("Calculate TMP direct values from action potential parameters"))
			{
				sample_count = TMP_total_duration/TMP_dt + 1;
				tmp_direct_values = MatrixX<Real>::Zero(sample_count, heart_probes.size());

				for (int sample = 0; sample < sample_count; sample++)
				{
					Real t_current = (Real)sample*TMP_dt;

					// update heart TMP from action potential parameters
					for (int i = 0; i < M; i++)
					{
						QH(i) = extracellular_potential(t_current, TMP_dt, heart_action_potential_params[i]); //action_potential_value_2
					}

					// calculate body surface potentials
					calculate_torso_potentials();

					for (int i = 0; i < heart_probes.size(); i++)
					{
						tmp_direct_values(sample, i) = evaluate_probe(*heart_mesh, heart_probes[i]);
						//tmp_direct_values(sample, i) = (Real)sample/1000.0;
						//tmp_direct_values(sample, i) = QH(i);
					}
				}

				tmp_source = TMP_SOURCE_TMP_DIRECT_VALUES;
				printf("Calculated TMP direct values from action potential parameters\n");
			}

			if (ImGui::Button("Export TMP and BSP probes values (CSV)"))
			{
				// save file dialog
				std::string file_name = save_file_dialog("tmp_bsp_values_csv", "All\0*.*\0");

				// export
				if (file_name != "")
				{
					// claculate values
					MatrixX<Real> tmp_direct_values_temporary = MatrixX<Real>::Zero(sample_count, M);
					for (int sample = 0; sample < sample_count; sample++)
					{
						Real t_current = (Real)sample * TMP_dt;

						for (int i = 0; i < M; i++)
						{
							tmp_direct_values_temporary(sample, i) = extracellular_potential(t_current, TMP_dt, heart_action_potential_params[i]); //action_potential_value_2
						}
					}

					if (export_tmp_bsp_values_csv(file_name, tmp_direct_values_temporary, probes_values))
					{
						printf("Exported \"%s\" TMP and BSP values (CSV)\n", file_name.c_str());
					}
					else
					{
						printf("Failed to export \"%s\" TMP and BSP values (CSV)\n", file_name.c_str());
					}
				}
			}

			// dt
			ImGui::InputReal("TMP Total Duration", &TMP_total_duration);
			ImGui::InputReal("TMP Time step", &TMP_dt, 0.001, 0.01, "%.5f");
			TMP_dt = clamp_value<Real>(TMP_dt, 0.000001, 5);
			ImGui::SliderInt("TMP steps per frame", &TMP_steps_per_frame, 0, 100);
			ImGui::SliderInt("TMP update refresh every FPS", &TMP_update_refresh_rate, 1, 100);
		}
		else if (tmp_source == TMP_SOURCE_TMP_DIRECT_VALUES)
		{
			ImGui::Text("Samples Count: %d", tmp_direct_values.rows());
			ImGui::Text("Current Sample: %d", current_sample);
			ImGui::DragReal("Interpolation Power", &interpolation_power, 0.1, 1, 12);
			ImGui::Checkbox("play values one time only", &tmp_direct_values_one_play);
			if (tmp_direct_values_one_play)
			{
				if (ImGui::Button("Play TMP Again"))
				{
					current_sample = 0;
				}
			}

			if (ImGui::Button("Import TMP direct values"))
			{
				// open file dialog
				std::string file_name = open_file_dialog("tmp_values", "All\0*.*\0");

				// import
				if (file_name != "")
				{
					if (import_tmp_direct_values(file_name, tmp_direct_values, N))
					{
						printf("Imported \"%s\" tmp_values\n", file_name.c_str());
					}
					else
					{
						printf("Failed to import \"%s\" tmp_values\n", file_name.c_str());
					}
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Export TMP direct values"))
			{
				// save file dialog
				std::string file_name = save_file_dialog("tmp_values", "All\0*.*\0");

				// export
				if (file_name != "")
				{
					if (export_tmp_direct_values(file_name, tmp_direct_values))
					{
						printf("Exported \"%s\" tmp_values\n", file_name.c_str());
					}
					else
					{
						printf("Failed to export \"%s\" tmp_values\n", file_name.c_str());
					}
				}
			}

			if (ImGui::Button("Export TMP and BSP probes values (CSV)"))
			{
				// save file dialog
				std::string file_name = save_file_dialog("tmp_bsp_values_csv", "All\0*.*\0");

				// export
				if (file_name != "")
				{
					if (export_tmp_bsp_values_csv(file_name, tmp_direct_values, probes_values))
					{
						printf("Exported \"%s\" TMP and BSP values (CSV)\n", file_name.c_str());
					}
					else
					{
						printf("Failed to export \"%s\" TMP and BSP values (CSV)\n", file_name.c_str());
					}
				}
			}

			ImGui::SliderInt("TMP steps per frame", &TMP_steps_per_frame, 0, 100);
			ImGui::SliderInt("TMP update refresh every FPS", &TMP_update_refresh_rate, 1, 100);
		}
		else if (tmp_source == TMP_SOURCE_WAVE_PROPAGATION)
		{
			ImGui::SliderInt("TMP steps per frame", &TMP_steps_per_frame, 0, 100);
			ImGui::SliderInt("TMP update refresh every FPS", &TMP_update_refresh_rate, 1, 100);
			wave_prop.render_gui();
		}


		/*
		// dipole position and vector
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::Text("Dipole");
		ImGui::DragVector3Eigen("Dipole position", dipole_pos, 0.01f);
		ImGui::DragVector3Eigen("Dipole Vector", dipole_vec, 0.01f);
		ImGui::Checkbox("Zero right heart section", &zero_heart_right_section);

		// dipole curve
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		int im_dipole_vec_source = (int)dipole_vec_source - 1;
		ImGui::Combo("Dipole Vector Values Source", (int*)&im_dipole_vec_source, "Constant Value\0Bezier Curve\0Values List\0", 3);
		dipole_vec_source = (ValuesSource)clamp_value<int>(im_dipole_vec_source+1, 1, 3);
		if (dipole_vec_source == VALUES_SOURCE_BEZIER_CURVE)
		{
			ImGui::Text("Time: %.4f s", t);

			// dt
			ImGui::InputReal("time step", &dt, 0.001, 0.01, "%.5f");
			dt = clamp_value<Real>(dt, 0.00001, 5);

			// steps per frame
			ImGui::SliderInt("steps per frame", &steps_per_frame, 0, 100);

			if (ImGui::ListBoxHeader("dipole curve", { 0, 200 }))
			{
				// curve points
				for (int i = 0; i < dipole_curve.points.size(); i++)
				{
					// point position
					std::string point_name = "p" + std::to_string(i);
					ImGui::DragVector3Eigen(point_name.c_str(), dipole_curve.points[i], 0.01f);
					// segment duration
					if (i != 0 && i%3 == 0)
					{
						int segment_idx = i/3-1;
						ImGui::SameLine();
						std::string duration_name = "d" + std::to_string(segment_idx);
						ImGui::InputReal(duration_name.c_str(), &dipole_curve.segments_duratoins[segment_idx]);
					}
					// tangent point mirror
					if (i != 0 && i != 1 && i%3 != 0)
					{
						ImGui::SameLine();
						std::string point_mirror_name = "Mirror " + std::to_string(i);
						if (ImGui::Button(point_mirror_name.c_str()))
						{
							int pivot_idx = ((i+1)/3)*3;
							if (i == pivot_idx-1)
							{
								Eigen::Vector3<Real> new_point = dipole_curve.points[pivot_idx] - (dipole_curve.points[pivot_idx+1]-dipole_curve.points[pivot_idx]);
								dipole_curve.points[i] = new_point;
							}
							else if (i == pivot_idx+1)
							{
								Eigen::Vector3<Real> new_point = dipole_curve.points[pivot_idx] - (dipole_curve.points[pivot_idx-1]-dipole_curve.points[pivot_idx]);
								dipole_curve.points[i] = new_point;
							}
							else
							{
								printf("Invalid index for curve point mirror operation");
							}
						}
					}

				}
				
				ImGui::ListBoxFooter();
			}

			// add and remove point
			if (ImGui::Button("Add point"))
			{
				dipole_curve.add_point({ 0, 0, 0 });
			}
			ImGui::SameLine();
			if (ImGui::Button("remove point"))
			{
				dipole_curve.remove_point();
			}

			// Import curve locations
			if (ImGui::Button("Import curve"))
			{
				// open file dialog
				std::string file_name = open_file_dialog("dipole_vector.curve", "All\0*.*\0probes locations file (.probes)\0*.probes\0");

				// import
				if (file_name != "")
				{
					if (import_bezier_curve(file_name, dipole_curve))
					{
						printf("Imported \"%s\" curve\n", file_name.c_str());
					}
					else
					{
						printf("Failed to import \"%s\" curve\n", file_name.c_str());
					}
				}
			}
			// Export probes locations
			ImGui::SameLine();
			if (ImGui::Button("Export curve"))
			{
				// save file dialog
				std::string file_name = save_file_dialog("dipole_vector.curve", "All\0*.*\0probes locations file (.probes)\0*.probes\0");

				// export
				if (file_name != "")
				{
					if (export_bezier_curve(file_name, dipole_curve))
					{
						printf("Exported \"%s\" curve\n", file_name.c_str());
					}
					else
					{
						printf("Failed to export \"%s\" curve\n", file_name.c_str());
					}
				}
			}
		}
		// dipole vector values list
		if (dipole_vec_source == VALUES_SOURCE_VALUES_LIST)
		{
			ImGui::SliderInt("Change rate (frames)", &dipole_vec_values_list_change_rate, 1, 100);
			ImGui::Text("Counter: %d", dipole_vec_values_list_counter);
			ImGui::Text("Current dipole vector (%d) value: (%.3f, %.3f, %.3f)", dipole_vec_values_list_current, dipole_vec.x(), dipole_vec.y(), dipole_vec.z());

			if (ImGui::ListBoxHeader("dipole vector values list", { 0, 200 }))
			{
				// values
				for (int i = 0; i < dipole_vec_values_list.size(); i++)
				{
					// vector value
					std::string point_name = "vec" + std::to_string(i);
					ImGui::DragVector3Eigen(point_name.c_str(), dipole_vec_values_list[i], 0.01f);
				}

				ImGui::ListBoxFooter();
			}

			// add and remove value
			if (ImGui::Button("Add value"))
			{
				dipole_vec_values_list.push_back({ 0, 0, 0 });
			}
			ImGui::SameLine();
			if (ImGui::Button("Remove value"))
			{
				if (dipole_vec_values_list.size() > 0)
				{
					dipole_vec_values_list.erase(dipole_vec_values_list.begin() + dipole_vec_values_list.size() - 1);
				}
			}
		}
		*/

		// rendering options
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::Text("Rendering Options");
		ImGui::SliderFloat("Dipole Vector Thickness", &dipole_vector_thickness, 1, 20);
		ImGui::SliderFloat("Heart Render Scale", &heart_render_scale, 0.1, 5);
		if (ImGui::Checkbox("Rotate Camera", &camera_rotate))
		{
			camera_eye_radius = glm::length(camera.eye-camera.look_at);
		}
		ImGui::SliderFloat("Rotation Speed (Hz)", &camera_rotation_speed, -2, 2);
		ImGui::SliderFloat("Torso Opacity", &torso_opacity, 0, 1);
		ImGui::SliderFloat("Mesh Plot Ambient", &mesh_plot_ambient, 0, 1);
		ImGui::SliderFloat("Mesh Plot Specular", &mesh_plot_specular, 1, 10);
		ImGui::ColorEdit4("Background Color", (float*)&color_background);
		ImGui::ColorEdit4("Negative Color", (float*)&color_n);
		ImGui::ColorEdit4("Positive Color", (float*)&color_p);
		ImGui::ColorEdit4("Probe Color", (float*)&color_probes);
		ImGui::SliderFloat("Dipole Vector Rendering Scale", &dipole_vector_scale, 0.1, 2);
		if (dipole_vec_source == VALUES_SOURCE_BEZIER_CURVE)
		{
			ImGui::Checkbox("Render Dipole Curve", &render_dipole_curve);
			if (render_dipole_curve)
			{
				ImGui::Checkbox("Render Dipole Curve Lines", &render_dipole_curve_lines);
			}
		}
		if (dipole_vec_source == VALUES_SOURCE_VALUES_LIST)
		{
			ImGui::Checkbox("Render Dipole Vector Values Points", &render_dipole_vec_values_point);
			ImGui::Checkbox("Render Dipole Vector Values Vectors", &render_dipole_vec_values_vectors);
			ImGui::Checkbox("Render Dipole Vector Values Locus", &render_dipole_vec_values_locus);
		}
		// Torso Potential Range
		ImGui::Checkbox("Torso Use Separate Min Max For The Range", &torso_use_separate_min_max_range);
		ImGui::Checkbox("Torso Potential Adaptive Range (Each Frame Separate)", &torso_potential_adaptive_range);
		ImGui::Checkbox("Torso Potential Adaptive Range Limit", &torso_potential_adaptive_range_limit);
		ImGui::DragReal("Torso Plot Scale (max absolute value)", &torso_potential_max_abs_value, 0.001);
		ImGui::DragReal("Torso Plot max value", &torso_potential_max_value, 0.001);
		ImGui::DragReal("Torso Plot min value", &torso_potential_min_value, 0.001);
		if (ImGui::Button("Reset Torso Plot Scale"))
		{
			torso_potential_max_abs_value = 0;
			torso_potential_max_value = -1e12;
			torso_potential_min_value = 1e12;
		}
		// Heart Potential Range
		ImGui::Checkbox("Heart Use Separate Min Max For The Range", &heart_use_separate_min_max_range);
		ImGui::Checkbox("Heart Potential Adaptive Range (Each Frame Separate)", &heart_potential_adaptive_range);
		ImGui::Checkbox("Heart Potential Adaptive Range Limit", &heart_potential_adaptive_range_limit);
		ImGui::DragReal("Heart Plot Scale (max absolute value)", &heart_potential_max_abs_value, 0.001);
		ImGui::DragReal("Heart Plot max value", &heart_potential_max_value, 0.001);
		ImGui::DragReal("Heart Plot min value", &heart_potential_min_value, 0.001);
		if (ImGui::Button("Reset Heart Plot Scale"))
		{
			heart_potential_max_abs_value = 0;
			heart_potential_max_value = -1e12;
			heart_potential_min_value = 1e12;
		}

		// probes
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		if (ImGui::ListBoxHeader("Probes", {0, 120}))
		{
			for (int i = 0; i < probes.size(); i++)
			{
				bool is_selected = i==current_selected_probe;
				ImGui::Selectable(probes[i].name.c_str(), &is_selected);
				if (is_selected)
				{
					current_selected_probe = i;
				}

				// value
				ImGui::SameLine();
				Real probe_value = evaluate_probe(*torso, probes[i]);
				std::string item_name = "  " + std::to_string(probe_value);
				ImGui::Text(item_name.c_str());
			}
			ImGui::ListBoxFooter();
		}
		// probe info
		if (current_selected_probe != -1 && current_selected_probe < probes.size())
		{
			probe_info = true;
			ImGui::Begin("Probe info", &probe_info);

			const Probe& probe = probes[current_selected_probe];

			char probe_name_buffer[256];
			strncpy(probe_name_buffer, probes[current_selected_probe].name.c_str(), sizeof(probe_name_buffer)-1);
			ImGui::InputText("Name", (char*)&probe_name_buffer, sizeof(probe_name_buffer));
			probes[current_selected_probe].name = probe_name_buffer;

			ImGui::Text("\tTriangle: %d", probe.triangle_idx);
			ImGui::Text("\tPoint: {%.3lf, %.3lf, %.3lf}", probe.point.x(), probe.point.y(), probe.point.z());
			ImGui::Text("\tValue: %.3lf", evaluate_probe(*torso, probe));

			ImGui::End();

			if (!probe_info)
			{
				current_selected_probe = -1;
			}
		}
		// Add probe, Remove probe, move up, move down
		if (ImGui::Button("Add Probe"))
		{
			adding_probe = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Remove Probe"))
		{
			if (current_selected_probe != -1 && current_selected_probe < probes.size())
			{
				probes.erase(probes.begin() + current_selected_probe);
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("U")) // up
		{
			if (current_selected_probe > 0)
			{
				swap(probes[current_selected_probe-1], probes[current_selected_probe]);
				current_selected_probe--;
				if (reference_probe == current_selected_probe)
				{
					reference_probe++;
				}
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("D")) // down
		{
			if (current_selected_probe < probes.size()-1)
			{
				swap(probes[current_selected_probe], probes[current_selected_probe+1]);
				current_selected_probe++;
				if (reference_probe == current_selected_probe)
				{
					reference_probe--;
				}
			}
		}
		if (adding_probe)
		{
			ImGui::SameLine();
			ImGui::Text("Click to add a probe");
		}
		// Clear all torso probes
		if (ImGui::Button("Clear Torso Probes"))
		{
			probes.resize(0);
		}
		// reset reference probe value
		if (reference_probe >= probes.size())
		{
			reference_probe = -1;
		}
		// Reference probe
		if (ImGui::BeginCombo("Reference probe", reference_probe != -1 ? probes[reference_probe].name.c_str() : "NON"))
		{
			// NON
			if (ImGui::Selectable("NON", reference_probe == -1))
			{
				reference_probe = -1;
			}
			// probes
			for (int i = 0; i < probes.size(); i++)
			{
				bool is_selected = i == reference_probe; // You can store your selection however you want, outside or inside your objects
				if (ImGui::Selectable(probes[i].name.c_str(), is_selected))
					reference_probe = i;
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		// cast probes rays
		ImGui::InputInt("Torso Probes Rows", &torso_cast_probes_rows);
		torso_cast_probes_rows = clamp_value<int>(torso_cast_probes_rows, 0, 1000000);
		ImGui::InputInt("Torso Probes Columns", &torso_cast_probes_cols);
		torso_cast_probes_cols = clamp_value<int>(torso_cast_probes_cols, 0, 1000000);
		if (ImGui::Button("Cast torso probes in sphere"))
		{
			probes = cast_probes_in_sphere("B", *torso, torso_cast_probes_rows, torso_cast_probes_cols);
		}
		// cast probes rays
		if (ImGui::Button("OLD cast probes in sphere"))
		{
			probes.clear();
			// spherical coordinates to cartisian
			for (int theta_i = 0; theta_i < 12; theta_i++)
			{
				Real theta = -PI/2 + PI*((Real)theta_i/12);
				for (int phi_i = 0; phi_i < 12; phi_i++)
				{
					// calculate ray
					Real phi = 0 + 2*PI*((Real)phi_i/12);
					Vector3<Real> ray_direction = { cos(theta)*cos(phi), sin(theta), cos(theta)*sin(phi) };
					ray_direction.normalize();
					Ray cast_ray = { {0, 0, 0}, ray_direction };

					// intersect ray with mesh
					Real t;
					int tri_idx;
					if (ray_mesh_intersect(*torso, Vector3<Real>(0, 0, 0), cast_ray, t, tri_idx))
					{
						Vector3<Real> intersection_point = cast_ray.point_at_dir(t);
						probes.push_back(Probe{ tri_idx, intersection_point, std::string("B_")+std::to_string(theta_i)+"_"+std::to_string(phi_i) });
					}
				}
			}
		}
		// cast probes rays
		if (ImGui::Button("9x9 FIXED cast probes in sphere"))
		{
			probes.clear();
			// spherical coordinates to cartisian
			for (int theta_i = 0; theta_i < 9; theta_i++)
			{
				Real theta = -PI/2 + PI*((Real)theta_i/9);
				for (int phi_i = 0; phi_i < 9; phi_i++)
				{
					// calculate ray
					Real phi = 0 + 2*PI*((Real)phi_i/9);
					Vector3<Real> ray_direction = { cos(theta)*cos(phi), sin(theta), cos(theta)*sin(phi) };
					ray_direction.normalize();
					Ray cast_ray = { {0, 0, 0}, ray_direction };

					// intersect ray with mesh
					Real t;
					int tri_idx;
					if (ray_mesh_intersect(*torso, Vector3<Real>(0, 0, 0), cast_ray, t, tri_idx))
					{
						Vector3<Real> intersection_point = cast_ray.point_at_dir(t);
						probes.push_back(Probe{ tri_idx, intersection_point, std::string("B_")+std::to_string(theta_i)+"_"+std::to_string(phi_i) });
					}
				}
			}
		}
		// Plane probes cast (front and back)
		ImGui::InputReal("Torso Probes x min", &torso_cast_probes_x_min);
		ImGui::InputReal("Torso Probes x max", &torso_cast_probes_x_max);
		ImGui::InputReal("Torso Probes y min", &torso_cast_probes_y_min);
		ImGui::InputReal("Torso Probes y max", &torso_cast_probes_y_max);
		if (ImGui::Button("Cast Torso Probes In Plane (Front and Back)"))
		{
			std::vector<Probe> front_probes = cast_probes_in_plane("B_F", *torso, torso_cast_probes_rows, torso_cast_probes_cols, 1, -1, torso_cast_probes_x_min, torso_cast_probes_x_max, torso_cast_probes_y_min, torso_cast_probes_y_max);
			std::vector<Probe> back_probes = cast_probes_in_plane("B_B", *torso, torso_cast_probes_rows, torso_cast_probes_cols, -1, 1, torso_cast_probes_x_min, torso_cast_probes_x_max, torso_cast_probes_y_min, torso_cast_probes_y_max);
			probes = front_probes;
			probes.insert(probes.end(), back_probes.begin(), back_probes.end());
		}
		if (ImGui::Button("Cast Torso Probes In Plane (Front Only)"))
		{
			probes = cast_probes_in_plane("B_F", *torso, torso_cast_probes_rows, torso_cast_probes_cols, 1, -1, torso_cast_probes_x_min, torso_cast_probes_x_max, torso_cast_probes_y_min, torso_cast_probes_y_max);
		}
		if (ImGui::Button("Cast Torso Probes In Plane (Back Only)"))
		{
			probes = cast_probes_in_plane("B_B", *torso, torso_cast_probes_rows, torso_cast_probes_cols, -1, 1, torso_cast_probes_x_min, torso_cast_probes_x_max, torso_cast_probes_y_min, torso_cast_probes_y_max);
		}
		// Import probes locations
		if (ImGui::Button("Import probes"))
		{
			// open file dialog
			std::string file_name = open_file_dialog("locations.probes", "All\0*.*\0probes locations file (.probes)\0*.probes\0");

			// import
			if (file_name != "")
			{
				if (import_probes(file_name, probes))
				{
					printf("Imported \"%s\" probes\n", file_name.c_str());
				}
				else
				{
					printf("Failed to import \"%s\" probes\n", file_name.c_str());
				}
			}
		}
		// Export probes locations
		ImGui::SameLine();
		if (ImGui::Button("Export probes"))
		{
			// save file dialog
			std::string file_name = save_file_dialog("locations.probes", "All\0*.*\0probes locations file (.probes)\0*.probes\0");

			// export
			if (file_name != "")
			{
				if (export_probes(file_name, probes))
				{
					printf("Exported \"%s\" probes\n", file_name.c_str());
				}
				else
				{
					printf("Failed to export \"%s\" probes\n", file_name.c_str());
				}
			}
		}

		ImGui::Checkbox("probes differentiation", &probes_differentiation);


		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		// view probes graph
		if (ImGui::Button("Torso Probes graph"))
		{
			probes_graph = true;
		}
		if (ImGui::Button("Heart Probes graph"))
		{
			heart_probes_graph = true;
		}
		ImGui::Checkbox("Clear graph at (t = 0)", &probes_graph_clear_at_t0);
		// dump to csv
		if (ImGui::Button("Dump to csv"))
		{
			// save file dialog
			std::string file_name = save_file_dialog("probes.csv", "CSV File (.csv)\0*.csv\0All Files\0*.*\0\0");

			// dump
			if (file_name != "")
			{
				bool res = dump_probes_values_to_csv(file_name.c_str(), dipole_pos, dipole_curve, dt, sample_count, probes, probes_values);
				if (res)
				{
					printf("Successfuly dumped probes to \"%s\"\n", file_name.c_str());
				}
				else
				{
					printf("Failed to dump probes to \"%s\"\n", file_name.c_str());
				}
			}
		}
		if (ImGui::Button("Dump TMP BSP Probes to CSV"))
		{
			Timer generating_timer;
			generating_timer.start();

			// calculate BSP probes values
			std::vector<std::string> names(heart_probes.size()+probes.size(), "");
			MatrixX<Real> TMP_BSP_values = MatrixX<Real>::Zero(sample_count, heart_probes.size()+probes.size());
			for (int sample = 0; sample < sample_count; sample++)
			{
				Real t_current = sample*TMP_dt;

				// update heart potentials
				if (tmp_source == TMP_SOURCE_ACTION_POTENTIAL_PARAMETERS)
				{
					// update heart TMP from action potential parameters
					for (int i = 0; i < M; i++)
					{
						QH(i) = extracellular_potential(t_current, TMP_dt, heart_action_potential_params[i]); //action_potential_value_2
					}
				}
				else /*TMP_SOURCE_WAVE_PROPAGATION*/
				{
					// wave propagation
					if (sample == 0)
					{
						wave_prop.reset();
					}
					wave_prop.simulation_step();
					QH = wave_prop.get_potentials();
				}

				// calculate body surface potentials
				calculate_torso_potentials();

				for (int i = 0; i < heart_probes.size(); i++)
				{
					names[i] = heart_probes[i].name;
					TMP_BSP_values(sample, i) = evaluate_probe(*heart_mesh, heart_probes[i]);
				}

				for (int i = 0; i < probes.size(); i++)
				{
					names[heart_probes.size()+i] = probes[i].name;
					TMP_BSP_values(sample, heart_probes.size()+i) = evaluate_probe(*torso, probes[i]);
				}
			}

			printf("Generated TMP BSP probes values in: %.3f seconds\n", generating_timer.elapsed_seconds());

			// save to file
			std::string file_name = save_file_dialog("TMP_BSP_values.csv", "All\0*.*\0CSV File (.csv)\0*.csv\0");

			// dump
			if (file_name != "")
			{
				bool res = dump_matrix_to_csv(file_name, names, TMP_BSP_values);
				if (res)
				{
					printf("Successfuly dumped probes to \"%s\"\n", file_name.c_str());
				}
				else
				{
					printf("Failed to dump probes to \"%s\"\n", file_name.c_str());
				}
			}
		}

		// heart probes
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		if (ImGui::ListBoxHeader("Heart Probes", { 0, 120 }))
		{
			for (int i = 0; i < heart_probes.size(); i++)
			{
				bool is_selected = i==heart_current_selected_probe;
				ImGui::Selectable(heart_probes[i].name.c_str(), &is_selected);
				if (is_selected)
				{
					heart_current_selected_probe = i;
				}

				// value
				ImGui::SameLine();
				Real probe_value = evaluate_probe(*heart_mesh, heart_probes[i]);

				if (tmp_source == TMP_SOURCE_TMP_DIRECT_VALUES)
				{
					probe_value = tmp_direct_values(current_sample, i);
				}

				std::string item_name = "  " + std::to_string(probe_value);
				ImGui::Text(item_name.c_str());
			}
			ImGui::ListBoxFooter();
		}
		// probe info
		if (heart_current_selected_probe != -1 && heart_current_selected_probe < heart_probes.size())
		{
			probe_info = true;
			ImGui::Begin("Heart Probe info", &probe_info);

			const Probe& heart_probe = heart_probes[heart_current_selected_probe];

			char probe_name_buffer[256];
			strncpy(probe_name_buffer, heart_probes[heart_current_selected_probe].name.c_str(), sizeof(probe_name_buffer)-1);
			ImGui::InputText("Name", (char*)&probe_name_buffer, sizeof(probe_name_buffer));
			heart_probes[heart_current_selected_probe].name = probe_name_buffer;

			ImGui::Text("\tTriangle: %d", heart_probe.triangle_idx);
			ImGui::Text("\tPoint: {%.3lf, %.3lf, %.3lf}", heart_probe.point.x(), heart_probe.point.y(), heart_probe.point.z());
			ImGui::Text("\tValue: %.3lf", evaluate_probe(*heart_mesh, heart_probe));

			ImGui::End();

			if (!probe_info)
			{
				heart_current_selected_probe = -1;
			}
		}
		// Add probe, Remove probe, move up, move down
		if (ImGui::Button("Add heart Probe"))
		{
			heart_adding_probe = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Remove Heart Probe"))
		{
			if (heart_current_selected_probe != -1 && heart_current_selected_probe < heart_probes.size())
			{
				heart_probes.erase(heart_probes.begin() + heart_current_selected_probe);
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("U H")) // up
		{
			if (heart_current_selected_probe > 0)
			{
				swap(heart_probes[heart_current_selected_probe-1], heart_probes[heart_current_selected_probe]);
				heart_current_selected_probe--;
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("D H")) // down
		{
			if (heart_current_selected_probe < heart_probes.size()-1)
			{
				swap(heart_probes[heart_current_selected_probe], heart_probes[heart_current_selected_probe+1]);
				heart_current_selected_probe++;
			}
		}
		if (heart_adding_probe)
		{
			ImGui::SameLine();
			ImGui::Text("Click to add a heart probe");
		}
		// Clear all heart probes
		if (ImGui::Button("Clear heart Probes"))
		{
			heart_probes.resize(0);
		}
		// cast heart probes rays
		ImGui::InputInt("Heart Probes Rows", &heart_cast_probes_rows);
		heart_cast_probes_rows = clamp_value<int>(heart_cast_probes_rows, 0, 1000000);
		ImGui::InputInt("Heart Probes Columns", &heart_cast_probes_cols);
		heart_cast_probes_cols = clamp_value<int>(heart_cast_probes_cols, 0, 1000000);
		ImGui::InputReal("Heart Probes Z-rotation (degree)", &heart_cast_probes_z_rot);
		if (ImGui::Button("Cast heart probes in sphere"))
		{
			heart_probes = cast_probes_in_sphere("H", *heart_mesh, heart_cast_probes_rows, heart_cast_probes_cols, heart_cast_probes_z_rot*PI/180);
		}
		// cast heart probes rays
		if (ImGui::Button("OLD cast heart probes in sphere"))
		{
			heart_probes.clear();
			// spherical coordinates to cartisian
			for (int theta_i = 0; theta_i < 10; theta_i++)
			{
				Real theta = -PI/2 + PI*((Real)theta_i/10);
				for (int phi_i = 0; phi_i < 10; phi_i++)
				{
					// calculate ray
					Real phi = 0 + 2*PI*((Real)phi_i/10);
					Vector3<Real> ray_direction = { cos(theta)*cos(phi), sin(theta), cos(theta)*sin(phi) };
					ray_direction.normalize();
					Ray cast_ray = { {0, 0, 0}, ray_direction };

					// intersect ray with mesh
					Real t;
					int tri_idx;
					if (ray_mesh_intersect(*heart_mesh, Vector3<Real>(0, 0, 0), cast_ray, t, tri_idx))
					{
						Vector3<Real> intersection_point = cast_ray.point_at_dir(t);
						heart_probes.push_back(Probe{ tri_idx, intersection_point, std::string("H_")+std::to_string(theta_i)+"_"+std::to_string(phi_i) });
					}
				}
			}
		}
		// cast probes rays
		if (ImGui::Button("15x5 FIXED cast probes in sphere"))
		{
			heart_probes.clear();
			// spherical coordinates to cartisian
			for (int theta_i = 0; theta_i < 5; theta_i++)
			{
				Real theta = -PI/2 + PI*((Real)theta_i/5);
				for (int phi_i = 0; phi_i < 15; phi_i++)
				{
					// calculate ray
					Real phi = 0 + 2*PI*((Real)phi_i/15);
					Vector3<Real> ray_direction = { cos(theta)*cos(phi), sin(theta), cos(theta)*sin(phi) };
					ray_direction.normalize();
					Ray cast_ray = { {0, 0, 0}, ray_direction };

					// intersect ray with mesh
					Real t;
					int tri_idx;
					if (ray_mesh_intersect(*heart_mesh, Vector3<Real>(0, 0, 0), cast_ray, t, tri_idx))
					{
						Vector3<Real> intersection_point = cast_ray.point_at_dir(t);
						heart_probes.push_back(Probe{ tri_idx, intersection_point, std::string("H_")+std::to_string(theta_i)+"_"+std::to_string(phi_i) });
					}
				}
			}
		}
		// Import probes locations
		if (ImGui::Button("Import heart probes"))
		{
			// open file dialog
			std::string file_name = open_file_dialog("locations.probes", "All\0*.*\0probes locations file (.probes)\0*.probes\0");

			// import
			if (file_name != "")
			{
				if (import_probes(file_name, heart_probes))
				{
					printf("Imported \"%s\" heart probes\n", file_name.c_str());
				}
				else
				{
					printf("Failed to import \"%s\" probes\n", file_name.c_str());
				}
			}
		}
		// Export probes locations
		ImGui::SameLine();
		if (ImGui::Button("Export heart probes"))
		{
			// save file dialog
			std::string file_name = save_file_dialog("locations.probes", "All\0*.*\0probes locations file (.probes)\0*.probes\0");

			// export
			if (file_name != "")
			{
				if (export_probes(file_name, heart_probes))
				{
					printf("Exported \"%s\" probes\n", file_name.c_str());
				}
				else
				{
					printf("Failed to export \"%s\" probes\n", file_name.c_str());
				}
			}
		}


		// stats
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		Real torso_max_val = -INFINITY;
		Real torso_min_val = INFINITY;
		for (MeshPlotVertex& vertex : torso->vertices)
		{
			torso_max_val = rmax(vertex.value, torso_max_val);
			torso_min_val = rmin(vertex.value, torso_min_val);
		}
		Real heart_max_val = -INFINITY;
		Real heart_min_val = INFINITY;
		for (MeshPlotVertex& vertex : heart_mesh->vertices)
		{
			heart_max_val = rmax(vertex.value, heart_max_val);
			heart_min_val = rmin(vertex.value, heart_min_val);
		}
		ImGui::Text("Stats:");
		ImGui::Text("\tTorso Potential Max: %f", torso_max_val);
		ImGui::Text("\tTorso Potential Min: %f", torso_min_val);
		ImGui::Text("\tTorso Potential Delta: %f", torso_max_val-torso_min_val);
		ImGui::Text("\t ");
		ImGui::Text("\tHeart Potential Max: %f", heart_max_val);
		ImGui::Text("\tHeart Potential Min: %f", heart_min_val);
		ImGui::Text("\tHeart Potential Delta: %f", heart_max_val-heart_min_val);

		// frame rate and frame time
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::Text("Frame Rate: %.1f FPS (%.3f ms), elapsed: %.2f s", 1/timer_dt, 1000*timer_dt, timer_time);

		ImGui::End();


		// probes graph
		if (probes_graph)
		{
			ImGui::Begin("Probes Graph", &probes_graph);

			// graph height
			ImGui::SliderFloat("Height", &probes_graph_height, 10, 200);
			ImGui::SliderFloat("Width", &probes_graph_width, 20, 400);

			// graphs
			static std::vector<float> values;
			static std::vector<float> values_diff;
			values.resize(probes_values.cols());
			values_diff.resize(probes_values.cols());
			for (int i = 0; i < probes.size(); i++)
			{
				for (int j = 0; j < probes_values.cols(); j++)
				{
					values[j] = probes_values(i, j);
				}
				for (int j = 1; j < probes_values.cols(); j++)
				{
					values_diff[j-1] = (values[j] - values[j-1])/TMP_dt;
				}
				values_diff[values_diff.size()-1] = values_diff[values_diff.size()-2];
				if (probes_differentiation)
				{
					ImGui::PlotLines((probes[i].name + "_differential").c_str(), &values_diff[0], values_diff.size(), 0, NULL, FLT_MAX, FLT_MAX, { probes_graph_width, probes_graph_height });

				}
				else
				{
					ImGui::PlotLines(probes[i].name.c_str(), &values[0], values.size(), 0, NULL, FLT_MAX, FLT_MAX, { probes_graph_width, probes_graph_height });
				}
			}

			ImGui::End();
		}

		// heart probes graph
		if (heart_probes_graph)
		{
			ImGui::Begin("Heart Probes Graph", &heart_probes_graph);

			// graph height
			ImGui::SliderFloat("Height", &heart_probes_graph_height, 10, 200);
			ImGui::SliderFloat("Width", &heart_probes_graph_width, 20, 400);

			// graphs
			static std::vector<float> values;
			values.resize(heart_probes_values.cols());
			for (int i = 0; i < heart_probes.size(); i++)
			{
				for (int j = 0; j < heart_probes_values.cols(); j++)
				{
					values[j] = heart_probes_values(i, j);
				}
				ImGui::PlotLines(heart_probes[i].name.c_str(), &values[0], values.size(), 0, NULL, FLT_MAX, FLT_MAX, { heart_probes_graph_width, heart_probes_graph_height });
			}

			ImGui::End();
		}


		// render ImGui
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}

	void handle_input()
	{
		// camera forward, up and right
		Vector3<Real> forward = glm2eigen(camera.look_at-camera.eye).normalized();
		Vector3<Real> up = glm2eigen(camera.up);
		Vector3<Real> right = forward.cross(up).normalized();
		up = right.cross(forward).normalized(); // recalculate the up vector


		// cursor (mouse and keyboard arrows)
		Vector2<Real> cursor_delta = { 0, 0 };
		// middle mouse + movement
		if (Input::isButtonDown(GLFW_MOUSE_BUTTON_MIDDLE))
		{
			cursor_delta = { Input::getCursorXDelta(), -Input::getCursorYDelta() };
		}
		const Real translation_scaler = glm2eigen(camera.eye-camera.look_at).norm()/width;// 0.01;
		const Real rotation_scaler = translation_scaler*10;
		// keyboard translation (shift + arrows)
		const Real keyboard_cursor_speed = 10;
		if (Input::isKeyDown(GLFW_KEY_LEFT))
		{
			cursor_delta.x() += keyboard_cursor_speed;
		}
		if (Input::isKeyDown(GLFW_KEY_RIGHT))
		{
			cursor_delta.x() -= keyboard_cursor_speed;
		}
		if (Input::isKeyDown(GLFW_KEY_DOWN))
		{
			cursor_delta.y() += keyboard_cursor_speed;
		}
		if (Input::isKeyDown(GLFW_KEY_UP))
		{
			cursor_delta.y() -= keyboard_cursor_speed;
		}

		// handle camera control (ctrl + mouse middle button)
		if (Input::isKeyDown(GLFW_KEY_LEFT_CONTROL) || Input::isKeyDown(GLFW_KEY_RIGHT_CONTROL))
		{
			// apply rotation
			Vector3<Real> new_location_rel = glm2eigen(camera.eye-camera.look_at) - right*rotation_scaler*cursor_delta.x() - up*rotation_scaler*cursor_delta.y();
			new_location_rel = new_location_rel.normalized() * glm2eigen(camera.eye-camera.look_at).norm();
			camera.eye = camera.look_at + eigen2glm(new_location_rel);
			//up = glm2eigen(camera.up)
		}

		// translation
		if (Input::isKeyDown(GLFW_KEY_LEFT_SHIFT) || Input::isKeyDown(GLFW_KEY_RIGHT_SHIFT))
		{
			// camera translation (shift + middle mouse button)
			Vector3<Real> translation = right*translation_scaler*cursor_delta.x() + up*translation_scaler*cursor_delta.y();

			// apply translation
			camera.look_at += -eigen2glm(translation);
			camera.eye += -eigen2glm(translation);
		}

		
		const Real zoom_scaler = 0.05;
		const Real roll_scaler = 0.05;
		// mouse scroll wheel
		Real scroll_delta = Input::getScrollDelta();
		// keyboard rotation (- + +)
		if (Input::isKeyDown(GLFW_KEY_KP_SUBTRACT))
		{
			scroll_delta -= 0.1;
		}
		if (Input::isKeyDown(GLFW_KEY_KP_ADD))
		{
			scroll_delta += 0.1;
		}
		// camera roll (ctrl + scroll wheel) or (ctrl + +/-)
		if (Input::isKeyDown(GLFW_KEY_LEFT_CONTROL) || Input::isKeyDown(GLFW_KEY_RIGHT_CONTROL))
		{
			Vector3<Real> new_up = (up + right*scroll_delta*roll_scaler).normalized();
			up = new_up;
		}
		// camera zoom (shift + scroll wheel) or (shift + +/-)
		if (Input::isKeyDown(GLFW_KEY_LEFT_SHIFT) || Input::isKeyDown(GLFW_KEY_RIGHT_SHIFT))
		{
			camera.eye = eigen2glm(glm2eigen(camera.look_at) + (1-scroll_delta*zoom_scaler)*glm2eigen(camera.eye-camera.look_at));
		}

		// update camera up
		camera.up = eigen2glm(up);

		// camera reset (R key)
		if (Input::isKeyPressed(GLFW_KEY_R))
		{
			camera = default_camera;
			camera.aspect = (float)width / (float)height;
		}

		// cancel adding probes (Esc)
		if (Input::isKeyPressed(GLFW_KEY_ESCAPE))
		{
			adding_probe = false;
		}

		// adding_probes
		adding_probe_intersected = false;
		if (adding_probe)
		{
			// normalized screen coordinates
			Real x = (Real)Input::getCursorXPos()/width*2 - 1;
			Real y = -((Real)Input::getCursorYPos()/height*2 - 1);

			// camera axis
			Vector3<Real> forward = glm2eigen(camera.look_at-camera.eye).normalized();
			Vector3<Real> up = glm2eigen(camera.up).normalized();
			Vector3<Real> right = forward.cross(up).normalized();
			// calculate the pointer direction
			Vector3<Real> direction = forward + up*tan(0.5*y*camera.fov) + right*tan(0.5*x*camera.aspect*camera.fov);
			direction = direction.normalized();

			Ray ray = { glm2eigen(camera.eye), direction };

			Real t;
			int tri_idx;
			if (ray_mesh_intersect(*torso, Vector3<Real>(0, 0, 0), ray, t, tri_idx))
			{
				if (Input::isButtonDown(GLFW_MOUSE_BUTTON_LEFT))
				{
					probes.push_back({ tri_idx, ray.point_at_dir(t), "probe" + std::to_string(probe_name_counter++)});
					adding_probe = false;
				}
				adding_probe_intersected = true;
				adding_probe_intersection = ray.point_at_dir(t);
			}
		}

		// heart_adding_probes
		heart_adding_probe_intersected = false;
		if (heart_adding_probe)
		{
			// normalized screen coordinates
			Real x = (Real)Input::getCursorXPos()/width*2 - 1;
			Real y = -((Real)Input::getCursorYPos()/height*2 - 1);

			// camera axis
			Vector3<Real> forward = glm2eigen(camera.look_at-camera.eye).normalized();
			Vector3<Real> up = glm2eigen(camera.up).normalized();
			Vector3<Real> right = forward.cross(up).normalized();
			// calculate the pointer direction
			Vector3<Real> direction = forward + up*tan(0.5*y*camera.fov) + right*tan(0.5*x*camera.aspect*camera.fov);
			direction = direction.normalized();

			Ray ray = { glm2eigen(camera.eye), direction };

			Real t;
			int tri_idx;
			if (ray_mesh_intersect(*heart_mesh, heart_pos, ray, t, tri_idx))
			{
				if (Input::isButtonDown(GLFW_MOUSE_BUTTON_LEFT))
				{
					heart_probes.push_back({ tri_idx, ray.point_at_dir(t)-heart_pos, "probe" + std::to_string(heart_probe_name_counter++) });
					heart_adding_probe = false;
				}
				heart_adding_probe_intersected = true;
				heart_adding_probe_intersection = ray.point_at_dir(t)-heart_pos;
			}
		}

		// drawing
		if (drawing_values_enabled)
		{
			// normalized screen coordinates
			Real x = (Real)Input::getCursorXPos()/width*2 - 1;
			Real y = -((Real)Input::getCursorYPos()/height*2 - 1);

			// camera axis
			Vector3<Real> forward = glm2eigen(camera.look_at-camera.eye).normalized();
			Vector3<Real> up = glm2eigen(camera.up).normalized();
			Vector3<Real> right = forward.cross(up).normalized();
			// calculate the pointer direction
			Vector3<Real> direction = forward + up*tan(0.5*y*camera.fov) + right*tan(0.5*x*camera.aspect*camera.fov);
			direction = direction.normalized();

			std::vector<Vector3<Real>> origin_points(drawing_values_points_count, Vector3<Real>(0, 0, 0));

			// generate origin points
			for (int i = 0; i < drawing_values_points_count; i++)
			{
				Real theta = ((Real)i/(Real)drawing_values_points_count)*2*PI;
				origin_points[i] = glm2eigen(camera.eye) + drawing_values_radius*sin(theta)*up + drawing_values_radius*cos(theta)*right;
			}

			//// intersect preview
			//drawing_values_preview.resize(drawing_values_points_count);
			//for (int i = 0; i < drawing_values_points_count; i++)
			//{
			//	Vector3<Real> new_point = origin_points[i] + direction*10;
			//	drawing_values_preview[i] = eigen2glm(new_point);
			//}

			// intersect preview
			drawing_values_preview.resize(0);
			Ray ray = { glm2eigen(camera.eye), direction };
			// calculate average t
			Real t_average = 0;
			for (int i = 0; i < drawing_values_points_count; i++)
			{
				Real t;
				int tri_idx;
				ray.origin = origin_points[i];
				if (ray_mesh_intersect(*heart_mesh, heart_pos, ray, t, tri_idx))
				{
					t_average = t;
				}
			}
			// actual intersection
			Real t;
			int tri_idx;
			for (int i = 0; i < drawing_values_points_count; i++)
			{
				ray.origin = origin_points[i];
				if (ray_mesh_intersect(*heart_mesh, heart_pos, ray, t, tri_idx))
				{
					drawing_values_preview.push_back(eigen2glm(ray.point_at_dir(t)));
				}
				else
				{
					t = t_average;
					drawing_values_preview.push_back(eigen2glm(ray.point_at_dir(t)));
				}
			}
			
			// apply drawing value to mesh vertices
			if (Input::isButtonDown(GLFW_MOUSE_BUTTON_LEFT))
			{
				// blur
				std::vector<int> intersected_values;
				for (int i = 0; i < heart_mesh->vertices.size(); i++)
				{
					if (perpendicular_distance(glm2eigen(camera.eye), glm2eigen(camera.eye)+direction, heart_pos+glm2eigen(heart_mesh->vertices[i].pos)) < drawing_values_radius)
					{
						if (drawing_only_facing_camera && glm2eigen(heart_mesh->vertices[i].normal).dot(-forward) > 0)
						{
							intersected_values.push_back(i);
						}
						else if (!drawing_only_facing_camera)
						{
							intersected_values.push_back(i);
						}
					}
				}
				// blur or set
				if (drawing_values_blur)
				{
					// calculate average value
					Real average_value = 0;
					for (int i = 0; i < intersected_values.size(); i++)
					{
						switch (drawing_values_mode)
						{
						case DRAW_REST_POTENTIAL:
							average_value += heart_action_potential_params[intersected_values[i]].resting_potential;
							break;
						case DRAW_PEAK_POTENTIAL:
							average_value += heart_action_potential_params[intersected_values[i]].peak_potential;
							break;
						case DRAW_DEPOLARIZATION_TIME:
							average_value += heart_action_potential_params[intersected_values[i]].depolarization_time;
							break;
						case DRAW_REPOLARIZATION_TIME:
							average_value += heart_action_potential_params[intersected_values[i]].repolarization_time;
							break;
						default:
							break;
						}
					}
					average_value /= (Real)intersected_values.size();

					// apply the blur effect
					for (int i = 0; i < intersected_values.size(); i++)
					{
						switch (drawing_values_mode)
						{
						case DRAW_REST_POTENTIAL:
							heart_action_potential_params[intersected_values[i]].resting_potential = 0.99*heart_action_potential_params[intersected_values[i]].resting_potential + 0.01*average_value;
							break;
						case DRAW_PEAK_POTENTIAL:
							heart_action_potential_params[intersected_values[i]].peak_potential = 0.99*heart_action_potential_params[intersected_values[i]].peak_potential + 0.01*average_value;
							break;
						case DRAW_DEPOLARIZATION_TIME:
							heart_action_potential_params[intersected_values[i]].depolarization_time = 0.99*heart_action_potential_params[intersected_values[i]].depolarization_time + 0.01*average_value;
							break;
						case DRAW_REPOLARIZATION_TIME:
							heart_action_potential_params[intersected_values[i]].repolarization_time = 0.99*heart_action_potential_params[intersected_values[i]].repolarization_time + 0.01*average_value;
							break;
						default:
							break;
						}
					}
				}
				else
				{
					// set value
					for (int i = 0; i < intersected_values.size(); i++)
					{
						switch (drawing_values_mode)
						{
						case DRAW_REST_POTENTIAL:
							heart_action_potential_params[intersected_values[i]].resting_potential = drawing_value;
							break;
						case DRAW_PEAK_POTENTIAL:
							heart_action_potential_params[intersected_values[i]].peak_potential = drawing_value;
							break;
						case DRAW_DEPOLARIZATION_TIME:
							heart_action_potential_params[intersected_values[i]].depolarization_time = drawing_value;
							break;
						case DRAW_REPOLARIZATION_TIME:
							heart_action_potential_params[intersected_values[i]].repolarization_time = drawing_value;
							break;
						}
					}
				}
			}
		}

		wave_prop.handle_input(camera);
	}

	void handle_server_requests()
	{
		Address request_addr; Port request_port;
		std::vector<uint8_t> request_bytes;

		if (server.poll_request(request_bytes, &request_addr, &request_port))
		{
			// handle request
			Deserializer des(request_bytes);
			Serializer ser;

			// handle message
			uint32_t request_type = des.parse_u32();
			if (request_type == REQUEST_GET_VALUES)
			{
				// row and columns count
				ser.push_u32(sample_count + 1); // row_count = sample_count + 1 row for the names
				ser.push_u32(probes.size() + 2 + 6); // col_count = sample idx + time + dipole_posx, dipole_posy, dipole_posz + dipole_vecx, dipole_vecy, dipole_vecz + probes_values

				// names row
				ser.push_string("sample");
				ser.push_string("time");
				ser.push_string("dipole_posx");
				ser.push_string("dipole_posy");
				ser.push_string("dipole_posz");
				ser.push_string("dipole_vecx");
				ser.push_string("dipole_vecy");
				ser.push_string("dipole_vecz");
				for (int i = 0; i < probes_values.rows(); i++)
				{
					ser.push_string(probes[i].name);
				}

				//// debug
				//for (int j = 0; j < sample_count; j++)
				//{
				//	for (int i = 0; i < probes.size() + 2 + 6; i++)
				//	{
				//		ser.push_double(j*sample_count + i);
				//	}
				//}

				// values (row major)
				for (int i = 0; i < sample_count; i++)
				{
					Real time = i * dt;
					ser.push_double(i); // sample
					ser.push_double(time); // time
					// dipole_pos
					ser.push_double(dipole_pos.x());
					ser.push_double(dipole_pos.y());
					ser.push_double(dipole_pos.z());
					// dipole_vec
					Eigen::Vector3<Real> dipole_vec_current = dipole_curve.point_at(time);
					ser.push_double(dipole_vec_current.x());
					ser.push_double(dipole_vec_current.y());
					ser.push_double(dipole_vec_current.z());

					// probes values
					for (int j = 0; j < probes.size(); j++)
					{
						ser.push_double(probes_values(j, i));
					}
				}
			}
			else if (request_type == REQUEST_SET_DIPOLE_VECTOR)
			{
				Eigen::Vector3<Real> new_dipole_vec;
				new_dipole_vec.x() = des.parse_double();
				new_dipole_vec.y() = des.parse_double();
				new_dipole_vec.z() = des.parse_double();
				dipole_vec = new_dipole_vec;
				dipole_vec_source = VALUES_SOURCE_CONSTANT;
				ser.push_u8(1); // return true acknowledgement
			}
			else if (request_type == REQUEST_GET_PROBES_NAMES)
			{
				ser.push_u32(probes.size()); // count of probes
				for (int i = 0; i < probes.size(); i++)
				{
					ser.push_string(probes[i].name);
				}
			}
			else if (request_type == REQUEST_CALCULATE_VALUES_FOR_VECTOR)
			{
				Eigen::Vector3<Real> new_dipole_vec;
				new_dipole_vec.x() = des.parse_double();
				new_dipole_vec.y() = des.parse_double();
				new_dipole_vec.z() = des.parse_double();
				dipole_vec = new_dipole_vec;
				calculate_torso_potentials();

				ser.push_u32(3 + probes.size()); // vector x, y, z and count of probes
				// dipole vector
				ser.push_double(dipole_vec.x());
				ser.push_double(dipole_vec.y());
				ser.push_double(dipole_vec.z());
				// probes values
				for (int i = 0; i < probes.size(); i++)
				{
					ser.push_double(evaluate_probe(*torso, probes[i]));
				}
			}
			else if (request_type == REQUEST_CALCULATE_VALUES_FOR_RANDOM_VECTORS)
			{
				uint32_t random_samples_count = des.parse_u32(); // random samples count
				Real maximum_radius = des.parse_double();

				ser.push_u32(3 + probes.size()); // vector x, y, z and count of probes
				Timer generating_timer;
				generating_timer.start();
				Random rnd;
				for (uint32_t i = 0; i < random_samples_count; i++)
				{
					dipole_vec = rnd.next_vector3(maximum_radius);
					calculate_torso_potentials();

					// dipole vector
					ser.push_double(dipole_vec.x());
					ser.push_double(dipole_vec.y());
					ser.push_double(dipole_vec.z());

					// probes values
					for (int i = 0; i < probes.size(); i++)
					{
						ser.push_double(evaluate_probe(*torso, probes[i]));
					}
				}
				printf("Generated %u random vector values in %.3f ms\n", random_samples_count, 1000*generating_timer.elapsed_seconds());
			}
			else if (request_type == REQUEST_SET_DIPOLE_VECTOR_VALUES)
			{
				uint32_t values_count = des.parse_u32();

				dipole_vec_values_list.clear();

				for (uint32_t i = 0; i < values_count; i++)
				{
					Eigen::Vector3<Real> new_value;
					new_value.x() = des.parse_double();
					new_value.y() = des.parse_double();
					new_value.z() = des.parse_double();
					dipole_vec_values_list.push_back(new_value);
				}
				
				dipole_vec_source = VALUES_SOURCE_VALUES_LIST;
				ser.push_u8(1); // return true acknowledgement
			}
			else if (request_type == REQUEST_GET_TMP_BSP_VALUES)
			{
				Timer generating_timer;
				generating_timer.start();
				
				// calculate BSP values
				MatrixX<Real> TMP_BSP_values = MatrixX<Real>::Zero(sample_count, M+N);
				for (int sample = 0; sample < sample_count; sample++)
				{
					Real t_current = sample*TMP_dt;

					// update heart potentials
					if (tmp_source == TMP_SOURCE_ACTION_POTENTIAL_PARAMETERS)
					{
						// update heart TMP from action potential parameters
						for (int i = 0; i < M; i++)
						{
							QH(i) = extracellular_potential(t_current, TMP_dt, heart_action_potential_params[i]); //action_potential_value_2
						}
					}
					else /*TMP_SOURCE_WAVE_PROPAGATION*/
					{
						// wave propagation
						if (sample == 0)
						{
							wave_prop.reset();
						}
						wave_prop.simulation_step();
						QH = wave_prop.get_potentials();
					}

					// calculate body surface potentials
					calculate_torso_potentials();

					for (int i = 0; i < M; i++)
					{
						TMP_BSP_values(sample, i) = QH(i);
					}

					for (int i = 0; i < N; i++)
					{
						TMP_BSP_values(sample, M+i) = QB(i);
					}
				}

				printf("Generated BSP probes values in: %.3f seconds\n", generating_timer.elapsed_seconds());

				// serialize data
				ser.push_u32(sample_count); // sample count
				ser.push_u32(M); // TMP values count
				ser.push_u32(N); // BSP values count

				// serialize matrix SAMPLE_COUNTx(M+PROBES_COUNT)
				for (int i = 0; i < TMP_BSP_values.rows(); i++)
				{
					for (int j = 0; j < TMP_BSP_values.cols(); j++)
					{
						ser.push_double(TMP_BSP_values(i, j));
					}
				}
			}
			else if (request_type == REQUEST_SET_TMP_VALUES)
			{
				// deserialize data
				int rows_count = des.parse_u32();
				int cols_count = des.parse_u32();

				if (cols_count == heart_probes.size())
				{
					// deserialize matrix SAMPLE_COUNTxPROBES_COUNT
					MatrixX<Real> new_tmp_direct_values = MatrixX<Real>::Zero(rows_count, cols_count);
					for (int i = 0; i < rows_count; i++)
					{
						for (int j = 0; j < cols_count; j++)
						{
							new_tmp_direct_values(i, j) = des.parse_double();
						}
					}

					// set new values
					tmp_direct_values = new_tmp_direct_values;
					tmp_source = TMP_SOURCE_TMP_DIRECT_VALUES;
					tmp_direct_values_one_play = true;
					current_sample = 0;

					ser.push_u8(1); // return true acknowledgement
				}
				else
				{
					printf("TMP values count doesn't match\n");

					ser.push_u8(0); // return false acknowledgement
				}
			}
			else if (request_type == REQUEST_GET_TMP_BSP_VALUES_PROBES)
			{
				Timer generating_timer;
				generating_timer.start();

				// calculate BSP probes values
				MatrixX<Real> TMP_BSP_values = MatrixX<Real>::Zero(sample_count, heart_probes.size()+probes.size());
				for (int sample = 0; sample < sample_count; sample++)
				{
					Real t_current = sample*TMP_dt;

					// update heart potentials
					if (tmp_source == TMP_SOURCE_ACTION_POTENTIAL_PARAMETERS)
					{
						// update heart TMP from action potential parameters
						for (int i = 0; i < M; i++)
						{
							QH(i) = extracellular_potential(t_current, TMP_dt, heart_action_potential_params[i]); //action_potential_value_2
						}
					}
					else /*TMP_SOURCE_WAVE_PROPAGATION*/
					{
						// wave propagation
						if (sample == 0)
						{
							wave_prop.reset();
						}
						wave_prop.simulation_step();
						QH = wave_prop.get_potentials();
					}

					// calculate body surface potentials
					calculate_torso_potentials();

					for (int i = 0; i < heart_probes.size(); i++)
					{
						TMP_BSP_values(sample, i) = evaluate_probe(*heart_mesh, heart_probes[i]);
					}

					for (int i = 0; i < probes.size(); i++)
					{
						TMP_BSP_values(sample, heart_probes.size()+i) = evaluate_probe(*torso, probes[i]);
					}
				}

				printf("Generated BSP probes values in: %.3f seconds\n", generating_timer.elapsed_seconds());

				// serialize data
				ser.push_u32(sample_count); // sample count
				ser.push_u32(heart_probes.size()); // heart probes count
				ser.push_u32(probes.size()); // probes count

				// serialize matrix SAMPLE_COUNTx(HEART_PROBES_COUNT+PROBES_COUNT)
				for (int i = 0; i < TMP_BSP_values.rows(); i++)
				{
					for (int j = 0; j < TMP_BSP_values.cols(); j++)
					{
						ser.push_double(TMP_BSP_values(i, j));
					}
				}
			}
			else if (request_type == REQUEST_GET_TMP_BSP_VALUES_PROBES_2)
			{
				Timer generating_timer;
				generating_timer.start();

				// progress bar
				print_progress_bar(0);

				// calculate BSP probes values
				MatrixX<Real> TMP_BSP_values = MatrixX<Real>::Zero(sample_count, heart_probes.size()+probes.size());
				heart_probes_values_temp.resize(1, heart_probes.size());
				for (int sample = 0; sample < sample_count; sample++)
				{
					Real t_current = sample*TMP_dt;

					// update heart potentials
					if (tmp_source == TMP_SOURCE_ACTION_POTENTIAL_PARAMETERS)
					{
						// update heart TMP from action potential parameters
						for (int i = 0; i < M; i++)
						{
							QH(i) = extracellular_potential(t_current, TMP_dt, heart_action_potential_params[i]); //action_potential_value_2
						}
					}
					else /*TMP_SOURCE_WAVE_PROPAGATION*/
					{
						// wave propagation
						if (sample == 0)
						{
							wave_prop.reset();
						}
						wave_prop.simulation_step();
						QH = wave_prop.get_potentials();
					}

					// update heart probes values
					for (int i = 0; i < heart_mesh->vertices.size(); i++)
					{
						heart_mesh->vertices[i].value = QH(i);
					}
					for (int i = 0; i < heart_probes.size(); i++)
					{
						heart_probes_values_temp(i) = evaluate_probe(*heart_mesh, heart_probes[i]);
					}

					// use interpolation for heart potentials using heart probes
					if (heart_probes.size() > 0)
					{
						QH = tmp_probes_interpolation_matrix*heart_probes_values_temp;
					}
					else
					{
						// set values to 0
						for (int i = 0; i < heart_mesh->vertices.size(); i++)
						{
							QH(i) = 0;
						}
					}

					// calculate body surface potentials
					calculate_torso_potentials();

					// fill the matrix
					for (int i = 0; i < heart_probes.size(); i++)
					{
						TMP_BSP_values(sample, i) = heart_probes_values_temp(i);
					}

					for (int i = 0; i < probes.size(); i++)
					{
						TMP_BSP_values(sample, heart_probes.size()+i) = evaluate_probe(*torso, probes[i]);
					}

					print_progress_bar((sample*100)/sample_count);
				}

				print_progress_bar(100);
				printf("\n");

				printf("Generated BSP probes values in: %.3f seconds\n", generating_timer.elapsed_seconds());

				// serialize data
				ser.push_u32(sample_count); // sample count
				ser.push_u32(heart_probes.size()); // heart probes count
				ser.push_u32(probes.size()); // probes count

				// serialize matrix SAMPLE_COUNTx(HEART_PROBES_COUNT+PROBES_COUNT)
				for (int i = 0; i < TMP_BSP_values.rows(); i++)
				{
					for (int j = 0; j < TMP_BSP_values.cols(); j++)
					{
						ser.push_double(TMP_BSP_values(i, j));
					}
				}
			}
			else if (request_type == REQUEST_GET_TMP_BSP_VALUES_PROBES_TRAIN)
			{
				Timer generating_timer;
				generating_timer.start();
				Random rnd;

				// samples count
				uint32_t request_sample_count = des.parse_u32();

				// calculate BSP probes values
				MatrixX<Real> TMP_BSP_values = MatrixX<Real>::Zero(request_sample_count, heart_probes.size()+probes.size());
				for (int sample = 0; sample < request_sample_count; sample++)
				{
					Real t_current = sample*TMP_dt;

					// update heart potentials
					for (int i = 0; i < M; i++)
					{
						QH(i) = rnd.next_real()*2 - 1;
					}

					// calculate body surface potentials
					calculate_torso_potentials();

					for (int i = 0; i < heart_probes.size(); i++)
					{
						TMP_BSP_values(sample, i) = evaluate_probe(*heart_mesh, heart_probes[i]);
					}

					for (int i = 0; i < probes.size(); i++)
					{
						TMP_BSP_values(sample, heart_probes.size()+i) = evaluate_probe(*torso, probes[i]);
					}
				}

				printf("Generated BSP probes values in: %.3f seconds\n", generating_timer.elapsed_seconds());

				// serialize data
				ser.push_u32(request_sample_count); // sample count
				ser.push_u32(heart_probes.size()); // heart probes count
				ser.push_u32(probes.size()); // probes count

				// serialize matrix SAMPLE_COUNTx(HEART_PROBES_COUNT+PROBES_COUNT)
				for (int i = 0; i < TMP_BSP_values.rows(); i++)
				{
					for (int j = 0; j < TMP_BSP_values.cols(); j++)
					{
						ser.push_double(TMP_BSP_values(i, j));
					}
				}
			}
			else
			{
				printf("Unknown request\n");
			}

			// send response
			if (!server.push_response(ser.get_data()))
			{
				printf("Failed to push response\n");
			}

			// log request
			printf("Request from (%s:%d):\n \tIndex: %d\n \tRequest ID: %s (%d)\n \tRequest size: %u bytes\n \tResponse size: %u bytes\n\n",
				request_addr.to_string().c_str(), request_port,
				server_request_counter,
				request_type_to_string((RequestType)request_type).c_str(), request_type, 
				request_bytes.size(), ser.get_data().size());

			server_request_counter++;
		}
	}

private:
	GLFWwindow* window;
	int width, height;
	glGraphicsDevice* gldev;
	std::string torso_model_path;
	MeshPlot* torso = NULL;
	MeshPlot* heart_mesh = NULL;
	unsigned int N; // torso vertex count
	unsigned int M; // heart vertex count
	AxisRenderer* axis_renderer;
	MeshPlotRenderer* mpr;
	glFrameBuffer* torso_fb;
	float torso_opacity = 0.5;
	float mesh_plot_ambient = 0.6;
	float mesh_plot_specular = 2;
	float heart_render_scale = 1;
	glm::vec4 color_background = { 0.1, 0.05, 0.1, 1 };
	glm::vec4 color_n, color_p;
	glm::vec4 color_probes;
	LookAtCamera camera;

	Real air_conductivity;
	Real toso_conductivity;
	Real heart_conductivity;
	Real torso_sigma_p;
	Real torso_sigma_n;
	Real heart_sigma_p;
	Real heart_sigma_n;
	Vector3<Real> heart_pos;
	Real heart_scale = 1;
	Vector3<Real> dipole_pos;
	Vector3<Real> dipole_vec;
	Real t;
	bool zero_heart_right_section = false;
	MatrixX<Real> A;
	MatrixX<Real> IA_inv;
	MatrixX<Real> B;
	MatrixX<Real> Q;

	// new approach
	MatrixX<Real> QH; // Heart potentials
	MatrixX<Real> QB; // Body potentials
	MatrixX<Real> ZBH; // transfer matrix
	//MatrixX<Real> A_heart;
	//MatrixX<Real> IA_inv_heart;
	//MatrixX<Real> B_heart;
	//MatrixX<Real> Q_heart;

	// dipole vector source
	ValuesSource dipole_vec_source = VALUES_SOURCE_CONSTANT;
	// dipole vector curve
	BezierCurve dipole_curve = { {{0, 0, 0}, {-1, 0, 0}, {-1, -1, 0}, {1, -1, 0}}, {1} };
	Real dt = 0.004; // time step
	int steps_per_frame = 1;
	int sample_count;
	int current_sample = 0;
	Eigen::MatrixX<Real> probes_values; // PROBES_COUNTxSAMPLE_COUNT
	std::vector<glm::vec3> dipole_locus;
	bool render_dipole_curve = true;
	bool render_dipole_curve_lines = true;
	float dipole_vector_thickness = 2;
	// dipole vector values list
	std::vector<Eigen::Vector3<Real>> dipole_vec_values_list;
	int dipole_vec_values_list_current = 0;
	int dipole_vec_values_list_counter = 0;
	int dipole_vec_values_list_change_rate = 60;
	bool render_dipole_vec_values_point = false;
	bool render_dipole_vec_values_vectors = false;
	bool render_dipole_vec_values_locus = true;
	// rendering scale
	float dipole_vector_scale = 0.5;
	// Torso potentials range
	bool torso_use_separate_min_max_range = false;
	bool torso_potential_adaptive_range = false;
	bool torso_potential_adaptive_range_limit = true;
	Real torso_potential_max_abs_value = 0;
	Real torso_potential_max_value = -1e12;
	Real torso_potential_min_value = 1e12;
	// Heart potentials range
	bool heart_use_separate_min_max_range = false;
	bool heart_potential_adaptive_range = false;
	bool heart_potential_adaptive_range_limit = true;
	Real heart_potential_max_abs_value = 0;
	Real heart_potential_max_value = -1e12;
	Real heart_potential_min_value = 1e12;

	// probes
	bool adding_probe = false;
	bool adding_probe_intersected = false;
	Eigen::Vector3<Real> adding_probe_intersection;
	bool probe_info;
	int current_selected_probe = -1;
	int probe_name_counter = 1;
	bool probes_graph = false;
	bool probes_graph_clear_at_t0 = false;
	float probes_graph_height = 60;
	float probes_graph_width = 120;
	std::vector<Probe> probes;
	int reference_probe = -1;
	bool probes_differentiation = false;

	// server
	Server server;
	int server_address_select = 1;
	int server_port = 1234;
	int server_request_counter = 0;

	// animation
	Timer timer;
	Real timer_time = 0;
	Real timer_dt = 0;
	// rotation
	bool camera_rotate = false;
	Real camera_eye_radius = 1;
	Real camera_angle = 0;
	float camera_rotation_speed = 1; // RPS

	// action potential drawing
	Real TMP_total_duration = 0.5;
	Real TMP_dt = 0.0005; // old value: 0.0001
	int TMP_steps_per_frame = 1; // old value: 20
	std::vector<ActionPotentialParameters> heart_action_potential_params;
	bool drawing_values_enabled = false;
	bool drawing_values_blur = false;
	bool drawing_only_facing_camera = true;
	DrawingMode drawing_values_mode = DRAW_DEPOLARIZATION_TIME;
	Real drawing_values_radius = 0.1;
	Real drawing_value = 15e-3;
	std::vector<glm::vec3> drawing_values_preview;
	int drawing_values_points_count = 32;
	bool drawing_view_target_channel = true;
	// TMP direct values source
	TMPValuesSource tmp_source = TMP_SOURCE_WAVE_PROPAGATION;
	MatrixX<Real> tmp_direct_values; // SAMPLE_COUNTxPROBES_COUNT matrix
	bool use_interpolation_for_action_potential = false;
	MatrixX<Real> heart_probes_values_temp;
	bool tmp_direct_values_one_play = true;

	// update rate
	int TMP_update_refresh_rate = 1; // updates per x frames
	int TMP_update_refresh_rate_counter = 0;

	// heart probes
	bool heart_adding_probe = false;
	bool heart_adding_probe_intersected = false;
	Eigen::Vector3<Real> heart_adding_probe_intersection;
	bool heart_probe_info;
	int heart_current_selected_probe = -1;
	int heart_probe_name_counter = 1;
	bool heart_probes_graph = false;
	bool heart_probes_graph_clear_at_t0 = false;
	float heart_probes_graph_height = 60;
	float heart_probes_graph_width = 120;
	std::vector<Probe> heart_probes;
	MatrixX<Real> heart_probes_values;

	// heart probes interpolation
	Real interpolation_power = 3;
	MatrixX<Real> tmp_probes_interpolation_matrix; // MxPROBES_COUNT
	int last_heart_probes_count = 0;
	Real last_interpolation_power = 3;

	// cast probes
	int torso_cast_probes_rows = 10;
	int torso_cast_probes_cols = 10;
	int heart_cast_probes_rows = 12;
	int heart_cast_probes_cols = 12;
	Real heart_cast_probes_z_rot = 45;
	Real torso_cast_probes_x_min = -0.38;
	Real torso_cast_probes_x_max = 0.38;
	Real torso_cast_probes_y_min = 0;
	Real torso_cast_probes_y_max = 0.6;

	// transfer matrix parameters
	Real close_range_threshold = 0;
	Real r_power = 2;
	bool ignore_negative_dot_product = true;

	// wave propagation
	WavePropagationSimulation wave_prop;

};


int main()
{
	ForwardECGApp app;

	int result = app.setup();
	if (result != 0)
	{
		printf("Failed to setup ForwardECGApp\n");
		return result;
	}

	try
	{
		app.run();
	}
	catch (std::exception e)
	{
		printf("Caught exception: %s\n", e.what());
	}

    return 0;
}
