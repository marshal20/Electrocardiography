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
#include <Eigen/Dense>
#include "math.h"
#include "axis_renderer.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "geometry.h"
#include "bezier_curve.h"
#include "filedialog.h"
#include "network/server.h"
#include "network/serializer.h"
#include "timer.h"
#include "random.h"


using namespace Eigen;

const LookAtCamera default_camera({ 0, 0, 5 }, { 0, 0, 0 }, { 0, 1, 0 }, (float)(45*PI/180), 1.0f, 0.1f, 1000.0f);


struct Probe
{
	int triangle_idx;
	Eigen::Vector3<Real> point;

	std::string name;
};

static Real evaluate_probe(const MeshPlot& mesh, const Probe& probe)
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


struct ProbeSerialized
{
	int tri;
	double px, py, pz;
	char name[16];
};

static bool import_probes(const std::string& file_name, std::vector<Probe>& probes)
{
	FILE* file = fopen(file_name.c_str(), "r");
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

static bool export_probes(const std::string& file_name, const std::vector<Probe>& probes)
{
	FILE* file = fopen(file_name.c_str(), "w");
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


struct PointSerialized
{
	double x, y, z;
};

static bool import_curve(const std::string& file_name, BezierCurve& curve)
{
	FILE* file = fopen(file_name.c_str(), "r");
	if (!file)
	{
		return false;
	}

	BezierCurve new_curve;

	// read points count
	int points_count = 0;
	fread(&points_count, sizeof(points_count), 1, file);
	int durations_count = 0;
	fread(&durations_count, sizeof(durations_count), 1, file);

	// read points
	PointSerialized serialized_point;
	new_curve.points.reserve(points_count);
	for (int i = 0; i < points_count; i++)
	{
		fread(&serialized_point, sizeof(serialized_point), 1, file);
		new_curve.points.push_back({ serialized_point.x, serialized_point.y, serialized_point.z });
	}

	// read durations
	double temp;
	new_curve.segments_duratoins.reserve(durations_count);
	for (int i = 0; i < durations_count; i++)
	{
		fread(&temp, sizeof(temp), 1, file);
		new_curve.segments_duratoins.push_back(temp);
	}

	// assign imported probes
	curve = new_curve;

	fclose(file);
	return true;
}

static bool export_curve(const std::string& file_name, const BezierCurve& curve)
{
	FILE* file = fopen(file_name.c_str(), "w");
	if (!file)
	{
		return false;
	}

	// read points count
	const int points_count = curve.points.size();
	fwrite(&points_count, sizeof(points_count), 1, file);
	const int durations_count = curve.segments_duratoins.size();
	fwrite(&durations_count, sizeof(durations_count), 1, file);

	// write points
	PointSerialized serialized_point;
	for (int i = 0; i < points_count; i++)
	{
		serialized_point = { curve.points[i].x(), curve.points[i].y(), curve.points[i].z() };
		fwrite(&serialized_point, sizeof(serialized_point), 1, file);
	}

	// read durations
	double temp;
	for (int i = 0; i < durations_count; i++)
	{
		temp = curve.segments_duratoins[i];
		fwrite(&temp, sizeof(temp), 1, file);
	}

	fclose(file);
	return true;
}

template<typename T>
static void swap(T& a, T& b)
{
	T temp = a;
	a = b;
	b = temp;
}


enum RequestType
{
	REQUEST_GET_VALUES = 1,
	REQUEST_SET_DIPOLE_VECTOR = 2,
	REQUEST_GET_PROBES_NAMES = 3,
	REQUEST_CALCULATE_VALUES_FOR_VECTOR = 4,
	REQUEST_CALCULATE_VALUES_FOR_RANDOM_VECTORS = 5,
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
		dipole_pos = { 0.07, 0.5, 0.1 };
		dipole_vec = { 1, 0, 0 };
		conductivity = 1;
		sigma_p = 0;
		sigma_n = conductivity;

		// torso mesh plot
		torso = new MeshPlot();
		load_torso_model("models/torso_model_3.fbx");
		color_n = { 0, 0, 1, 1 };
		color_p = { 1, 0, 0, 1 };
		color_probes = { 0, 0.25, 0, 1 };
		
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


			// timer
			timer_dt = timer.elapsed_seconds();
			timer_time += timer_dt;
			timer.start();

			// input
			handle_input();

			// set values
			sigma_p = 0;
			sigma_n = conductivity;
			// animate dipole vector
			//t += 0.01;
			//dipole_vec = { cos(-t), sin(-t), 0 };
			//dipole_vec = 10*dipole_vec;


			// animate camera rotation
			if (camera_rotate)
			{
				camera_angle += timer_dt*camera_rotation_speed*2*PI;
				camera.eye = camera.look_at + glm::vec3(camera_eye_radius*sin(camera_angle), 0, camera_eye_radius*cos(camera_angle));
			}

			// animate dipole vector on the curve
			if (dipole_vec_source == VALUES_SOURCE_BEZIER_CURVE)
			{
				// update dipoles_values size
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

		// set matrices sizes
		N = torso->vertices.size();
		A = MatrixX<Real>(N, N);
		B = MatrixX<Real>(N, 1);
		Q = MatrixX<Real>(N, 1);
		IA_inv = MatrixX<Real>(N, N);

		// calculate IA_inv
		calculate_coefficients_matrix();

		return true;
	}

	void calculate_coefficients_matrix()
	{
		// conductivity inside and outside torso
		sigma_p = 0;
		sigma_n = conductivity;

		// BEM solver (bounded conductor)
		// Q = B - AQ
		// (I+A)Q = B
		A = MatrixX<Real>::Zero(N, N);
		for (int i = 0; i < torso->vertices.size(); i++)
		{
			const MeshPlotVertex& vertex = torso->vertices[i];
			Vector3<Real> r = glm2eigen(vertex.pos);

			// A
			for (const MeshPlotFace& face : torso->faces)
			{
				Vector3<Real> a = glm2eigen(torso->vertices[face.idx[0]].pos);
				Vector3<Real> b = glm2eigen(torso->vertices[face.idx[1]].pos);
				Vector3<Real> c = glm2eigen(torso->vertices[face.idx[2]].pos);
				//Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;
				Vector3<Real> face_normal = (b-a).cross(c-a).normalized();

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Real const_val = 1/(4*PI)*2*(sigma_n-sigma_p)/(sigma_n+sigma_p)*1/pow((r-center).norm(), 3) * (r-center).dot(face_normal)*area;

				A(i, face.idx[0]) += const_val/3;
				A(i, face.idx[1]) += const_val/3;
				A(i, face.idx[2]) += const_val/3;
			}
		}

		// (I+A)'
		IA_inv = (MatrixX<Real>::Identity(N, N) + A).inverse();
	}

	void calculate_potentials()
	{
		// BEM solver (bounded conductor)
		// Q = B - AQ
		// (I+A)Q = B
		// (I+A) is constant for the same geometry
		for (int i = 0; i < torso->vertices.size(); i++)
		{
			const MeshPlotVertex& vertex = torso->vertices[i];
			Vector3<Real> r = glm2eigen(vertex.pos);

			// B
			Real Q_inf = 1/(4*PI*conductivity) * 1/pow((r-dipole_pos).norm(), 3) * (r-dipole_pos).dot(dipole_vec);
			B(i) = 2*conductivity/(sigma_n+sigma_p)*Q_inf;
		}

		// calculate potentials
		Q = IA_inv * B;
		
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
			torso->vertices[i].value = Q(i);
		}

		// apply reference probe (to potentials in toso model only not Q)
		if (reference_probe != -1)
		{
			Real reference_value = evaluate_probe(*torso, probes[reference_probe]);
			for (int i = 0; i < torso->vertices.size(); i++)
			{
				torso->vertices[i].value -= reference_value;
			}
		}
	}

	void render()
	{
		// calculate maximum value
		Real max_abs = 1e-6;
		for (MeshPlotVertex& vertex : torso->vertices)
		{
			max_abs = rmax(rabs(vertex.value), max_abs);
		}
		//// debug
		//printf("max_abs : %f\n", max_abs);

		// update torso potential values at GPU
		torso->update_gpu_buffers();


		// clear buffers
		gldev->clearColorBuffer(color_background.r, color_background.g, color_background.b, color_background.a);
		gldev->depthTest(STATE_ENABLED);
		gldev->depthFunc(COMPARISON_LESS);
		gldev->clearDepthBuffer(1.0);

		// set alpha mode
		gldev->setAlpha(Alpha{ true, Alpha::SRC_ALPHA, Alpha::ONE_MINUS_SRC_ALPHA });


		// render torso to torso_fb
		torso_fb->bind();
		gldev->clearColorBuffer(0, 0, 0, 0);
		gldev->clearDepthBuffer(1.0);
		const Real alpha = 1;
		mpr->set_colors(color_p, color_n);
		mpr->set_view_projection_matrix(camera.calculateViewProjection());
		mpr->set_max_val(max_abs);
		mpr->render_mesh_plot(glm::mat4(1), torso);
		torso_fb->unbind();
		gldev->bindBackbuffer();
		// render torso_fb texture
		Renderer2D::setProjection(ortho(0, width, height, 0, -1, 1));
		Renderer2D::drawTexture({ width/2, height/2 }, { width, height }, torso_fb->getColorTexture(0), {1, 1, 1, torso_opacity});


		// render dipole
		gldev->depthTest(STATE_DISABLED); // disable depth testing
		Renderer3D::setStyle(Renderer3D::Style(true, dipole_vector_thickness, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));
		Renderer3D::setProjection(camera.calculateViewProjection());
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

		// render axis
		axis_renderer->render(camera);
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
		// torso conductivity
		float im_conductivity = conductivity;
		ImGui::InputFloat("Torso Conductivity", &im_conductivity, 0.01, 10);
		conductivity = im_conductivity;
		// recalculate coefficients matrix
		if (ImGui::Button("Recalculate Coefficients Matrix"))
		{
			calculate_coefficients_matrix();
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


		// dipole position and vector
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::Text("Dipole");
		glm::vec3 im_dipole_pos = eigen2glm(dipole_pos);
		glm::vec3 im_dipole_vec = eigen2glm(dipole_vec);
		ImGui::DragFloat3("Dipole position", (float*)&im_dipole_pos, 0.01f);
		ImGui::DragFloat3("Dipole Vector", (float*)&im_dipole_vec, 0.01f);
		dipole_pos = glm2eigen(im_dipole_pos);
		dipole_vec = glm2eigen(im_dipole_vec);

		// dipole curve
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		int im_dipole_vec_source = (int)dipole_vec_source - 1;
		ImGui::Combo("Dipole Vector Values Source", (int*)&im_dipole_vec_source, "Constant Value\0Bezier Curve\0Values List\0", 3);
		dipole_vec_source = (ValuesSource)clamp_value<int>(im_dipole_vec_source+1, 1, 3);
		if (dipole_vec_source == VALUES_SOURCE_BEZIER_CURVE)
		{
			ImGui::Text("Time: %.4f s", t);

			// dt
			float im_dt = dt;
			ImGui::InputFloat("time step", &im_dt, 0.001, 0.01, "%.5f");
			dt = clamp_value<float>(im_dt, 0.00001, 5);

			// steps per frame
			ImGui::SliderInt("steps per frame", &steps_per_frame, 0, 100);

			if (ImGui::ListBoxHeader("dipole curve", { 0, 200 }))
			{
				// curve points
				glm::vec3 im_curve_point_pos;
				float im_curve_duration;
				for (int i = 0; i < dipole_curve.points.size(); i++)
				{
					// point position
					im_curve_point_pos = eigen2glm(dipole_curve.points[i]);
					std::string point_name = "p" + std::to_string(i);
					ImGui::DragFloat3(point_name.c_str(), (float*)&im_curve_point_pos, 0.01f);
					dipole_curve.points[i] = glm2eigen(im_curve_point_pos);
					// segment duration
					if (i != 0 && i%3 == 0)
					{
						int segment_idx = i/3-1;
						im_curve_duration = dipole_curve.segments_duratoins[segment_idx];
						ImGui::SameLine();
						std::string duration_name = "d" + std::to_string(segment_idx);
						ImGui::InputFloat(duration_name.c_str(), &im_curve_duration);
						dipole_curve.segments_duratoins[segment_idx] = im_curve_duration;
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
					if (import_curve(file_name, dipole_curve))
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
					if (export_curve(file_name, dipole_curve))
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
				glm::vec3 im_dipole_vec_value;
				float im_curve_duration;
				for (int i = 0; i < dipole_vec_values_list.size(); i++)
				{
					// vector value
					im_dipole_vec_value = eigen2glm(dipole_vec_values_list[i]);
					std::string point_name = "vec" + std::to_string(i);
					ImGui::DragFloat3(point_name.c_str(), (float*)&im_dipole_vec_value, 0.01f);
					dipole_vec_values_list[i] = glm2eigen(im_dipole_vec_value);
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

		// rendering options
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::Text("Rendering Options");
		ImGui::SliderFloat("Dipole Vector Thickness", &dipole_vector_thickness, 1, 20);
		if (ImGui::Checkbox("Rotate Camera", &camera_rotate))
		{
			camera_eye_radius = glm::length(camera.eye-camera.look_at);
		}
		ImGui::SliderFloat("Rotation Speed (Hz)", &camera_rotation_speed, -2, 2);
		ImGui::SliderFloat("Torso Opacity", &torso_opacity, 0, 1);
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


		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		// view probes graph
		if (ImGui::Button("Probes graph"))
		{
			probes_graph = true;
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

		// stats
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		Real max_val = -INFINITY;
		Real min_val = INFINITY;
		for (MeshPlotVertex& vertex : torso->vertices)
		{
			max_val = rmax(vertex.value, max_val);
			min_val = rmin(vertex.value, min_val);
		}
		ImGui::Text("Stats:");
		ImGui::Text("\tMin: %.3f, Max: %.3f", min_val, max_val);

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

			// graphs
			static std::vector<float> values;
			values.resize(probes_values.cols());
			for (int i = 0; i < probes.size(); i++)
			{
				for (int j = 0; j < probes_values.cols(); j++)
				{
					values[j] = probes_values(i, j);
				}
				ImGui::PlotLines(probes[i].name.c_str(), &values[0], probes_values.cols(), 0, NULL, FLT_MAX, FLT_MAX, {0, probes_graph_height});
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

		// handle camera control (mouse middle button)
		if (Input::isButtonDown(GLFW_MOUSE_BUTTON_MIDDLE))
		{
			Vector2<Real> cursor_delta = { Input::getCursorXDelta(), -Input::getCursorYDelta() };
			const Real translation_scaler = glm2eigen(camera.eye-camera.look_at).norm()/width;// 0.01;
			const Real rotation_scaler = translation_scaler*10;

			if (Input::isKeyDown(GLFW_KEY_LEFT_SHIFT) || Input::isKeyDown(GLFW_KEY_RIGHT_SHIFT))
			{
				// camera translation (shift + middle mouse button)
				Vector3<Real> translation = right*translation_scaler*cursor_delta.x() + up*translation_scaler*cursor_delta.y();
				camera.look_at += -eigen2glm(translation);
				camera.eye += -eigen2glm(translation);
			}
			else
			{
				// camera rotation (middle mouse button)
				Vector3<Real> new_location_rel = glm2eigen(camera.eye-camera.look_at) - right*rotation_scaler*cursor_delta.x() - up*rotation_scaler*cursor_delta.y();
				new_location_rel = new_location_rel.normalized() * glm2eigen(camera.eye-camera.look_at).norm();
				camera.eye = camera.look_at + eigen2glm(new_location_rel);
				//up = glm2eigen(camera.up);
			}
		}
		
		const Real zoom_scaler = 0.05;
		const Real roll_scaler = 0.05;
		Real scroll_delta = Input::getScrollDelta();
		if (Input::isKeyDown(GLFW_KEY_LEFT_CONTROL) || Input::isKeyDown(GLFW_KEY_RIGHT_CONTROL))
		{
			// camera roll (ctrl + scroll wheel)
			Vector3<Real> new_up = (up + right*scroll_delta*roll_scaler).normalized();
			up = new_up;
		}
		if (Input::isKeyDown(GLFW_KEY_LEFT_SHIFT) || Input::isKeyDown(GLFW_KEY_RIGHT_SHIFT))
		{
			// camera zoom (Shift + scroll wheel)
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
			Vector3<Real> direction = forward;
			direction = rodrigues_rotate(direction, up, -x*0.5*camera.fov*camera.aspect);
			direction = rodrigues_rotate(direction, right, y*0.5*camera.fov);
			direction = direction.normalized();

			Ray ray = { glm2eigen(camera.eye), direction };

			Real t;
			int tri_idx;
			if (ray_mesh_intersect(*torso, ray, t, tri_idx))
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
				calculate_potentials();

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
					calculate_potentials();

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
	unsigned int N;
	AxisRenderer* axis_renderer;
	MeshPlotRenderer* mpr;
	glFrameBuffer* torso_fb;
	float torso_opacity = 1;
	glm::vec4 color_background = { 0.1, 0.05, 0.1, 1 };
	glm::vec4 color_n, color_p;
	glm::vec4 color_probes;
	LookAtCamera camera;

	Real conductivity;
	Real sigma_p;
	Real sigma_n;
	Vector3<Real> dipole_pos;
	Vector3<Real> dipole_vec;
	Real t;
	MatrixX<Real> A;
	MatrixX<Real> IA_inv;
	MatrixX<Real> B;
	MatrixX<Real> Q;

	// dipole vector source
	ValuesSource dipole_vec_source = VALUES_SOURCE_BEZIER_CURVE;
	// dipole vector curve
	BezierCurve dipole_curve = { {{0, 0, 0}, {-1, 0, 0}, {-1, -1, 0}, {1, -1, 0}}, {1} };
	Real dt = 0.004; // time step
	int steps_per_frame = 1;
	int sample_count;
	int current_sample = 0;
	Eigen::MatrixX<Real> probes_values;
	std::vector<glm::vec3> dipole_locus;
	bool render_dipole_curve = true;
	bool render_dipole_curve_lines = true;
	float dipole_vector_thickness = 2;
	// dipole vector values list
	std::vector<Eigen::Vector3<Real>> dipole_vec_values_list;
	int dipole_vec_values_list_current = 0;
	int dipole_vec_values_list_counter = 0;
	int dipole_vec_values_list_change_rate = 60;
	bool render_dipole_vec_values_point = true;
	bool render_dipole_vec_values_vectors = true;
	bool render_dipole_vec_values_locus = true;
	// rendering scale
	float dipole_vector_scale = 0.5;

	// probes
	bool adding_probe = false;
	bool adding_probe_intersected = false;
	Eigen::Vector3<Real> adding_probe_intersection;
	bool probe_info;
	int current_selected_probe = -1;
	int probe_name_counter = 1;
	bool probes_graph = false;
	bool probes_graph_clear_at_t0 = true;
	float probes_graph_height = 70;
	std::vector<Probe> probes;
	int reference_probe = -1;

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

	app.run();

    return 0;
}
