#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <thread>
#include <glad/glad.h>
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

using namespace Eigen;

const LookAtCamera default_camera({ 0, 0, 5 }, { 0, 0, 0 }, { 0, 1, 0 }, (float)(45*PI/180), 1.0f, 0.1f, 1000.0f);


struct Probe
{
	int triangle_idx;
	Eigen::Vector3<Real> point;
};

static Real evaluate_probe(const MeshPlot& mesh, const Probe& probe)
{
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
		window = createOpenglWindow(800, 600, "ForwardECG");
		if (!window)
		{
			glfwTerminate();
			return -1;
		}

		// main device and input
		hookInputCallbacks(window);
		gldev = createOpenglDevice(window);
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

		// torso mesh plot
		torso = load_mesh_plot("models/torso_model_3.fbx");
		color_n = { 0, 0, 1, 1 };
		color_p = { 1, 0, 0, 1 };

		// initialize matrices
		N = torso.vertices.size();
		A = MatrixX<Real>(N, N);
		B = MatrixX<Real>(N, 1);
		Q = MatrixX<Real>(N, 1);
		IA_inv = MatrixX<Real>(N, N);

		// set parameters
		t = 0;
		dipole_pos = { 0.2, 0.4, 0.1 };
		dipole_vec = { 1, 0, 0 };
		conductivity = 1;
		sigma_p = 0;
		sigma_n = conductivity;

		//// Debug
		//Real max_val_x = 0.0;
		//Real max_val_y = 0.0;
		//Real max_val_z = 0.0;
		//for (MeshPlotVertex& vertex : torso.vertices)
		//{
		//	max_val_x = rmax(rabs(vertex.pos.x), max_val_x);
		//	max_val_y = rmax(rabs(vertex.pos.y), max_val_y);
		//	max_val_z = rmax(rabs(vertex.pos.z), max_val_z);
		//}
		//printf("max vertex values: {%f, %f, %f}\n", max_val_x, max_val_y, max_val_z);
		//printf("vertex count: %d\n", torso.vertices.size());

		// mesh plot renderer
		mpr = new MeshPlotRenderer;

		// LookAtCamera
		camera = default_camera;

		// calculate IA_inv
		calculate_coefficients_matrix();

		// setup ImGUI
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO();
		io.IniFilename = NULL;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init("#version 130");

		return 0;
	}

	void run()
	{
		// main loop
		while (!glfwWindowShouldClose(window))
		{
			// poll events
			Input::newFrame();
			glfwPollEvents();

			// handle window size change
			glfwGetWindowSize(window, &width, &height);
			gldev->resizeBackbuffer(width, height);
			gldev->viewport(0, 0, width, height);
			float aspect = (float)width / (float)height;
			camera.aspect = aspect;

			// input
			handle_input();

			// set values
			sigma_p = 0;
			sigma_n = conductivity;
			// animate dipole vector
			t += 0.01;
			//dipole_vec = { cos(-t), sin(-t), 0 };
			//dipole_vec = 10*dipole_vec;

			// update and render
			calculate_potentials();
			render();

			// update window
			glfwSwapBuffers(window);
			glfwSwapInterval(1);
			//std::this_thread::sleep_for(std::chrono::milliseconds(16));
		}

		// cleanup
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
		free_mesh_plot(torso);
		Renderer2D::cleanup();
		Renderer3D::cleanup();
		delete axis_renderer;
		delete mpr;
		delete gldev;
		glfwDestroyWindow(window);
		glfwTerminate();
	}

private:
	void calculate_coefficients_matrix()
	{
		// BEM solver (bounded conductor)
		// Q = B - AQ
		// (I+A)Q = B
		A = MatrixX<Real>::Zero(N, N);
		for (int i = 0; i < torso.vertices.size(); i++)
		{
			const MeshPlotVertex& vertex = torso.vertices[i];
			Vector3<Real> r = glm2eigen(vertex.pos);

			// A
			for (const MeshPlotFace& face : torso.faces)
			{
				Vector3<Real> a = glm2eigen(torso.vertices[face.idx[0]].pos);
				Vector3<Real> b = glm2eigen(torso.vertices[face.idx[1]].pos);
				Vector3<Real> c = glm2eigen(torso.vertices[face.idx[2]].pos);
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
		for (int i = 0; i < torso.vertices.size(); i++)
		{
			const MeshPlotVertex& vertex = torso.vertices[i];
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
	}

	void render()
	{
		// update mesh plot
		for (int i = 0; i < torso.vertices.size(); i++)
		{
			torso.vertices[i].value = Q(i);
		}

		// calculate maximum value
		Real max_abs = 0.0;
		for (MeshPlotVertex& vertex : torso.vertices)
		{
			max_abs = rmax(rabs(vertex.value), max_abs);
		}
		//// debug
		//printf("max_abs : %f\n", max_abs);

		torso.update_gpu_buffers();


		// clear buffers
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		gldev->depthTest(STATE_ENABLED);
		gldev->depthFunc(COMPARISON_LESS);
		gldev->clearDepthBuffer(1.0);

		// set alpha mode
		gldev->setAlpha(Alpha{ true, Alpha::SRC_ALPHA, Alpha::ONE_MINUS_SRC_ALPHA });

		// render torso
		const Real alpha = 1;
		mpr->set_colors(color_p, color_n);
		mpr->set_view_projection_matrix(camera.calculateViewProjection());
		mpr->set_max_val(max_abs);
		mpr->render_mesh_plot(glm::mat4(1), &torso);

		// render dipole
		gldev->depthTest(STATE_DISABLED); // disable depth testing
		Renderer3D::setStyle(Renderer3D::Style(true, 2, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));
		Renderer3D::setProjection(camera.calculateViewProjection());
		Renderer3D::drawLine(eigen2glm(dipole_pos), eigen2glm(dipole_pos+dipole_vec*0.5));

		// render probes
		for (int i = 0; i < probes.size(); i++)
		{
			glm::vec4 color = { 0, 1, 0, 1 };
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

		// dipole position and vector
		glm::vec3 im_dipole_pos = eigen2glm(dipole_pos);
		glm::vec3 im_dipole_vec = eigen2glm(dipole_vec);
		ImGui::DragFloat3("Dipole position", (float*)&im_dipole_pos, 0.01f);
		ImGui::DragFloat3("Dipole Vector", (float*)&im_dipole_vec, 0.01f);
		dipole_pos = glm2eigen(im_dipole_pos);
		dipole_vec = glm2eigen(im_dipole_vec);

		//// conductivity
		// ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		//float im_conductivity = conductivity;
		//ImGui::InputFloat("Conductivity", &im_conductivity, 0.01, 10);
		//conductivity = im_conductivity;

		// mesh plot colors
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::ColorEdit4("Negative color", (float*)&color_n);
		ImGui::ColorEdit4("Positive color", (float*)&color_p);

		// probes
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		probes_str.clear();
		for (const Probe& probe : probes)
		{
			Real probe_value = evaluate_probe(torso, probe);
			probes_str.push_back(std::to_string(probe_value));
		}
		probes_str_ptr.clear();
		for (const std::string& str : probes_str)
		{
			probes_str_ptr.push_back(&str[0]);
		}
		ImGui::ListBox("Probes", &current_selected_probe, &probes_str_ptr[0], probes_str_ptr.size());
		if (current_selected_probe != -1)
		{
			// probe info
			const Probe& probe = probes[current_selected_probe];
			ImGui::Text("\tTri: %d", probe.triangle_idx);
			ImGui::Text("\tPoint: {%.3lf, %.3lf, %.3lf}", probe.point.x(), probe.point.y(), probe.point.z());
			ImGui::Text("\tValue: %.3lf", evaluate_probe(torso, probe));
		}
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
		if (adding_probe)
		{
			ImGui::Text("\tClick to add a probe");
		}

		// stats
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		Real max_val = -INFINITY;
		Real min_val = INFINITY;
		for (MeshPlotVertex& vertex : torso.vertices)
		{
			max_val = rmax(vertex.value, max_val);
			min_val = rmin(vertex.value, min_val);
		}
		ImGui::Text("Stats:");
		ImGui::Text("\tMin: %.3f, Max: %.3f", min_val, max_val);

		// frame rate and frame time
		ImGui::Dummy(ImVec2(0.0f, 20.0f)); // spacer
		ImGui::Text("Frame Rate: %.1f FPS (%.3f ms)", ImGui::GetIO().Framerate, 1000.0f/ImGui::GetIO().Framerate);
		
		ImGui::End();

		// Rendering
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

		// camera reset (0 key)
		if (Input::isKeyDown(GLFW_KEY_0))
		{
			camera = default_camera;
		}

		// cancel adding probes
		if (Input::isKeyDown(GLFW_KEY_ESCAPE))
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
			if (ray_mesh_intersect(torso, ray, t, tri_idx))
			{
				if (Input::isButtonDown(GLFW_MOUSE_BUTTON_LEFT))
				{
					probes.push_back({ tri_idx, ray.point_at_dir(t) });
					adding_probe = false;
				}
				adding_probe_intersected = true;
				adding_probe_intersection = ray.point_at_dir(t);
			}
		}
	}

private:
	GLFWwindow* window;
	int width, height;
	glGraphicsDevice* gldev;
	MeshPlot torso;
	unsigned int N;
	AxisRenderer* axis_renderer;
	MeshPlotRenderer* mpr;
	glm::vec4 color_n, color_p;
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

	bool adding_probe = false;
	bool adding_probe_intersected = false;
	Eigen::Vector3<Real> adding_probe_intersection;
	int current_selected_probe = -1;
	std::vector<Probe> probes;
	std::vector<std::string> probes_str;
	std::vector<const char*> probes_str_ptr;
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
