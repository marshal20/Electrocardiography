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


#define PI 3.14159265359


using namespace Eigen;
typedef double Real;

static Real rclamp(Real val, Real min, Real max)
{
	if (val > max)
	{
		return max;
	}
	else if (val < min)
	{
		return min;
	}

	return val;
}

static Real rmax(Real a, Real b)
{
	if (a > b)
	{
		return a;
	}
	
	return b;
}

static Real rabs(Real val)
{
	if (val < 0)
	{
		return -val;
	}

	return val;
}

static Vector3<Real> glm2eigen(const glm::vec3& v3)
{
	return { v3.x, v3.y, v3.z };
}


int main()
{
	int width, height;

	// create the window
    GLFWwindow* window = createOpenglWindow(800, 600, "ForwardECG");
	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	// main device and input
	hookInputCallbacks(window);
    glGraphicsDevice* gldev = createOpenglDevice(window);
	gdevSet(gldev);

	// setup 2D renderer
	Renderer2D::init();
	float view_size = 2;
	Renderer2D::setProjection(ortho(-view_size, view_size, view_size, -view_size, 1, -1));
	Renderer2D::setStyle(Renderer2D::Style(true, 2, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));

	// torso mesh plot
	MeshPlot torso = load_mesh_plot("models/torso_model_3.fbx");

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
	printf("max vertex positions: {%f, %f, %f}\n", max_val_x, max_val_y, max_val_z);
	printf("vertex count: %d\n", torso.vertices.size());





	//// set torso values
	//for (int i = 0; i < torso.vertices.size(); i++)
	//{
	//	torso.vertices[i].value = 1.0;
	//}
	//torso.update_gpu_buffers();

	// mesh plot renderer
	MeshPlotRenderer* mpr = new MeshPlotRenderer;

	// LookAtCamera
	LookAtCamera camera;
	camera.eye = { 0, 0, 20 };
	camera.look_at = { 0, 0, 0 };
	camera.up = { 0, 1, 0 };
	camera.fov = (float)(90*PI/180);
	camera.aspect = 1.0f;
	camera.near = 0.1f;
	camera.far = 1000.0f;


	// main loop
	while (!glfwWindowShouldClose(window))
	{
		// handle window size change
		glfwGetWindowSize(window, &width, &height);
		gldev->resizeBackbuffer(width, height);
		gldev->viewport(0, 0, width, height);
		float aspect = (float)width / (float)height;
		camera.aspect = aspect;



		// calculating potentials

		Vector3<Real> dipole_pos = { 0.2, 0.4, 0 };
		Vector3<Real> dipole_vec = { 0, 1, -1 };
		Real conductivity = 1;

		// animate dipole vector
		static Real t = 0;
		t += 0.1;
		dipole_vec = { cos(-t), sin(-t), 0 };
		//dipole_vec *= 0.1;
		dipole_pos *= 100;

		Real sigma_p = 0;
		Real sigma_n = conductivity;
		// BEM solver (bounded conductor)
		// Q = B - AQ
		// (I+A)Q = B
		unsigned int N = torso.vertices.size();
		MatrixX<Real> A(N, N);
		MatrixX<Real> B(N, 1);
		for (int i = 0; i < torso.vertices.size(); i++)
		{
			const MeshPlotVertex& vertex = torso.vertices[i];

			// B
			Vector3<Real> r = glm2eigen(vertex.pos);
			Real Q_inf = 1/(4*PI*conductivity) * 1/pow((r-dipole_pos).norm(), 3) * (r-dipole_pos).dot(dipole_vec);
			B(i) = 2*conductivity/(sigma_n+sigma_p)*Q_inf;

			// A
			for (const MeshPlotFace& face : torso.faces)
			{
				Vector3<Real> a = glm2eigen(torso.vertices[face.idx[0]].pos);
				Vector3<Real> b = glm2eigen(torso.vertices[face.idx[1]].pos);
				Vector3<Real> c = glm2eigen(torso.vertices[face.idx[2]].pos);
				Vector3<Real> face_normal = (glm2eigen(torso.vertices[face.idx[0]].normal)+glm2eigen(torso.vertices[face.idx[1]].normal)+glm2eigen(torso.vertices[face.idx[2]].normal))/3;

				Real area = ((b-a).cross(c-a)).norm()/2;
				Vector3<Real> center = (a+b+c)/3; // triangle center
				Real const_val = 1/(4*PI)*2*(sigma_n-sigma_p)/(sigma_n+sigma_p)*1/pow((r-center).norm(), 3) * (r-center).dot(face_normal);

				A(i, face.idx[0]) += const_val/3;
				A(i, face.idx[1]) += const_val/3;
				A(i, face.idx[2]) += const_val/3;
			}
		}
		MatrixX<Real> Q = (MatrixX<Real>::Identity(N, N) + A).inverse() * B;
		//MatrixX<Real> Q = B;

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

		torso.update_gpu_buffers();


		// set alpha mode
		gldev->setAlpha({ true, Alpha::Factor::SRC_ALPHA, Alpha::Factor::ONE_MINUS_SRC_ALPHA });

		// render here
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		Renderer2D::drawCircle({ 0, 0 }, 1);

		// render torso
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		gldev->depthTest(STATE_ENABLED);
		gldev->depthFunc(COMPARISON_LESS);
		gldev->clearDepthBuffer(1.0);
		mpr->set_view_projection_matrix(camera.calculateViewProjection());
		mpr->set_max_val(max_abs);
		mpr->render_mesh_plot(scale({ 0.1, 0.1, 0.1 }), &torso);

		// render dipole
		Renderer2D::setProjection(ortho(-200, 200, 200, -200, 1, -1));
		glm::vec2 p1 = {dipole_pos.x(), dipole_pos.y()};
		glm::vec2 p2 = {(dipole_pos+dipole_vec*100).x(), (dipole_pos+dipole_vec*100).y()};
		Renderer2D::drawLine(p1, p2);

		// update window
		glfwSwapBuffers(window);
		glfwPollEvents();
		glfwSwapInterval(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(16));
	}

	// cleanup
	free_mesh_plot(torso);
	delete mpr;
	glfwTerminate();

    return 0;
}
