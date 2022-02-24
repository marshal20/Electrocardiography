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


#define PI 3.14159265359


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

	// torso model
	Model torso = loadModelFile("models/torso_model_2.fbx");
	Material& mat = torso.mesh_list[0].material;
	mat.specular = 1;
	mat.ambient = 0.2;
	mat.diffuse_color = { 0.8, 0.5, 0.5 };

	// LookAtCamera
	LookAtCamera camera;
	camera.eye = { 0, 20, 0 };
	camera.look_at = { 0, 0, 0 };
	camera.up = { 0, 0, 1 };
	camera.fov = (float)(90*PI/180);
	camera.aspect = 1.0f;
	camera.near = 0.1f;
	camera.far = 1000.0f;

	// forward renderer
	ForwardRenderer* fr = new ForwardRenderer;


	// main loop
	while (!glfwWindowShouldClose(window))
	{
		// handle window size change
		glfwGetWindowSize(window, &width, &height);
		gldev->resizeBackbuffer(width, height);
		gldev->viewport(0, 0, width, height);
		float aspect = (float)width / (float)height;
		camera.aspect = aspect;

		// render here
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		Renderer2D::drawCircle({ 0, 0 }, 1);

		// render torso
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		gldev->depthTest(STATE_ENABLED);
		gldev->depthFunc(COMPARISON_LESS);
		gldev->clearDepthBuffer(1.0);
		fr->setViewProjectionMatrix(camera.calculateViewProjection());
		//fr->view_pos = camera.eye;
		fr->renderMesh(scale({0.1, 0.1, 0.1}), &torso.mesh_list[0]);

		// update window
		glfwSwapBuffers(window);
		glfwPollEvents();
		glfwSwapInterval(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(16));
	}

	// cleanup
	freeModel(torso);
	delete fr;
	glfwTerminate();

    return 0;
}
