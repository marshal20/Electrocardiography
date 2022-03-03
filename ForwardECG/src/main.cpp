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
#include "model.h"
#include "camera.h"
#include "forward_renderer.h"
#include "mesh_plot.h"
#include <Eigen/Dense>
#include "math.h"


using namespace Eigen;

const LookAtCamera default_camera({ 0, 0, 5 }, { 0, 0, 0 }, { 0, 1, 0 }, (float)(45*PI/180), 1.0f, 0.1f, 1000.0f);


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

		// setup 2D renderer
		Renderer2D::init();
		const float view_size = 2;
		Renderer2D::setProjection(ortho(-view_size, view_size, view_size, -view_size, 1, -1));
		Renderer2D::setStyle(Renderer2D::Style(true, 2, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));

		// torso mesh plot
		torso = load_mesh_plot("models/torso_model_3.fbx");

		// initialize matrices
		N = torso.vertices.size();
		A = MatrixX<Real>(N, N);
		B = MatrixX<Real>(N, 1);
		Q = MatrixX<Real>(N, 1);
		IA_inv = MatrixX<Real>(N, N);

		// set parameters
		t = 0;
		dipole_pos = { 0.2, 0.4, 0.1 };
		dipole_vec = { 0, 1, -1 };
		conductivity = 1;
		sigma_p = 0;
		sigma_n = conductivity;

		// Debug
		Real max_val_x = 0.0;
		Real max_val_y = 0.0;
		Real max_val_z = 0.0;
		for (MeshPlotVertex& vertex : torso.vertices)
		{
			max_val_x = rmax(rabs(vertex.pos.x), max_val_x);
			max_val_y = rmax(rabs(vertex.pos.y), max_val_y);
			max_val_z = rmax(rabs(vertex.pos.z), max_val_z);
		}
		printf("max vertex values: {%f, %f, %f}\n", max_val_x, max_val_y, max_val_z);
		printf("vertex count: %d\n", torso.vertices.size());

		// mesh plot renderer
		mpr = new MeshPlotRenderer;

		// LookAtCamera
		camera = default_camera;

		// calculate IA_inv
		calculate_coefficients_matrix();

		return 0;
	}

	void run()
	{
		// main loop
		while (!glfwWindowShouldClose(window))
		{
			// handle window size change
			glfwGetWindowSize(window, &width, &height);
			gldev->resizeBackbuffer(width, height);
			gldev->viewport(0, 0, width, height);
			float aspect = (float)width / (float)height;
			camera.aspect = aspect;

			// input
			handle_input();

			// animate dipole vector
			t += 0.01;
			dipole_vec = { cos(-t), sin(-t), 0 };
			//dipole_vec = 10*dipole_vec;

			// update and render
			calculate_potentials();
			render();

			// update window
			glfwSwapBuffers(window);
			glfwPollEvents();
			glfwSwapInterval(1);
			//std::this_thread::sleep_for(std::chrono::milliseconds(16));
		}

		// cleanup
		free_mesh_plot(torso);
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
		mpr->set_view_projection_matrix(camera.calculateViewProjection());
		mpr->set_max_val(max_abs);
		mpr->render_mesh_plot(glm::mat4(1), &torso);

		// render dipole
		Renderer2D::setProjection(ortho(-2, 2, 2, -2, 1, -1));
		glm::vec2 p1 = { dipole_pos.x(), dipole_pos.y() };
		glm::vec2 p2 = { (dipole_pos+dipole_vec).x(), (dipole_pos+dipole_vec).y() };
		Renderer2D::drawLine(p1, p2);
	}

	void handle_input()
	{
		Input::newFrame();

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
			}

			//// debug
			//printf("Cursor pos: (%.2lf, %.2lf), delta: (%.2lf, %.2lf)          \r", 
			//	Input::getCursorXPos(), Input::getCursorYPos(),
			//	Input::getCursorXDelta(), Input::getCursorYDelta());

		}
		
		const Real scroll_scaler = 0.1;
		Real scroll_delta = scroll_scaler*Input::getScrollDelta();
		if (Input::isKeyDown(GLFW_KEY_LEFT_CONTROL) || Input::isKeyDown(GLFW_KEY_RIGHT_CONTROL))
		{
			// camera roll (ctrl + scroll wheel)
			Vector3<Real> new_up = (up + right*scroll_delta).normalized();
			up = new_up;
		}
		else
		{
			// camera zoom (scroll wheel)
			camera.eye = eigen2glm(glm2eigen(camera.look_at) + (1-scroll_delta)*glm2eigen(camera.eye-camera.look_at));
		}

		// update camera up
		camera.up = eigen2glm(up);

		// camera reset (zero key)
		if (Input::isButtonPressed(GLFW_KEY_0))
		{
			camera = default_camera;
		}

		//// debug
		//printf("Scroll delta: %.2lf                       \r", Input::getScrollOffset());
	}

private:
	GLFWwindow* window;
	int width, height;
	glGraphicsDevice* gldev;
	MeshPlot torso;
	unsigned int N;
	MeshPlotRenderer* mpr;
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
