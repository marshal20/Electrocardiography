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
	Renderer2D r2d;
	r2d.init();
	float view_size = 2;
	r2d.setProjection(ortho(-view_size, view_size, view_size, -view_size, 1, -1));
	r2d.setStyle(Renderer2D::Style(true, 2, { 0, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));

	// main loop
	while (!glfwWindowShouldClose(window))
	{
		// handle window size change
		glfwGetWindowSize(window, &width, &height);
		gldev->viewport(0, 0, width, height);

		// render here
		gldev->clearColorBuffer(0.1, 0.05, 0.1, 1);
		r2d.drawCircle({ 0, 0 }, 1);

		// update window
		glfwSwapBuffers(window);
		glfwPollEvents();
		glfwSwapInterval(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(16));
	}

	// cleanup
	glfwTerminate();
    return 0;
}
