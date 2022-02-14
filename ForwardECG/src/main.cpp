#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <thread>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "window.h"
#include "input.h"
#include "opengl/gl_graphics_device.h"

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

	hookInputCallbacks(window);
    glGraphicsDevice* gldev = createOpenglDevice(window);

	// main loop
	while (!glfwWindowShouldClose(window))
	{
		glfwGetWindowSize(window, &width, &height);

		// render here

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
