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

		//// convert from CM to M
		//for (MeshPlotVertex& vertex : torso.vertices)
		//{
		//	vertex.pos = glm::vec3(vertex.pos.x*100, vertex.pos.y*100, vertex.pos.z*100);
		//}

		// initialize matrices
		N = torso.vertices.size();
		A = MatrixX<Real>(N, N);
		B = MatrixX<Real>(N, 1);
		Q = MatrixX<Real>(N, 1);

		// set parameters
		dipole_pos = { 0.2, 0.4, 0 };
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
		camera.eye = { 0, 0, 5 };
		camera.look_at = { 0, 0, 0 };
		camera.up = { 0, 1, 0 };
		camera.fov = (float)(45*PI/180);
		camera.aspect = 1.0f;
		camera.near = 0.1f;
		camera.far = 1000.0f;

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
			Input::newFrame();

			// animate dipole vector
			static Real t = 0;
			t += 0.01;
			dipole_vec = { cos(-t), sin(-t), 0 };
			//dipole_vec = dipole_vec;

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
		glfwTerminate();
	}

private:
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
		printf("max_abs : %f\n", max_abs);

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
				Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;

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
