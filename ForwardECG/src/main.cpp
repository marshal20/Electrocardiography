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
	MeshPlot torso = load_mesh_plot("models/torso_model_2.fbx");








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
	camera.eye = { 0, 20, 5 };
	camera.look_at = { 0, 0, 0 };
	camera.up = { 0, 0, 1 };
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

		Vector3<Real> dipole_pos = { 0, 0.3, 0.5 };
		Vector3<Real> dipole_vec = { 0, 1, -1 };
		Real conductivity = 1;

		// animate dipole vector
		static Real t = 0;
		t += 0.1;
		dipole_vec = { 0, cos(-t), sin(-t) };

		for (MeshPlotVertex& vertex : torso.vertices)
		{
			Vector3<Real> r = Vector3<Real>(vertex.pos.x, vertex.pos.y, vertex.pos.z) - dipole_pos;
			Real potential = 1/(4*PI*conductivity) * 1/r.norm() * r.dot(dipole_vec);
			vertex.value = potential;
		}

		// normalize values
		Real max_abs = 0.0;
		for (MeshPlotVertex& vertex : torso.vertices)
		{
			max_abs = rmax(rabs(vertex.value), max_abs);
		}
		for (MeshPlotVertex& vertex : torso.vertices)
		{
			vertex.value = vertex.value/max_abs;
		}

		torso.update_gpu_buffers();



		// render here
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		Renderer2D::drawCircle({ 0, 0 }, 1);

		// render torso
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		gldev->depthTest(STATE_ENABLED);
		gldev->depthFunc(COMPARISON_LESS);
		gldev->clearDepthBuffer(1.0);
		mpr->set_view_projection_matrix(camera.calculateViewProjection());
		//fr->view_pos = camera.eye;
		mpr->render_mesh_plot(scale({ 0.1, 0.1, 0.1 }), &torso);

		// render dipole
		glm::vec2 p1 = {dipole_pos.y(), dipole_pos.z()};
		glm::vec2 p2 = {(dipole_pos+dipole_vec).y(), (dipole_pos+dipole_vec).z()};
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
