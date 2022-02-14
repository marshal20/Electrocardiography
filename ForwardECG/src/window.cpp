#include "window.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "input.h"
#include "opengl/gl_graphics_device.h"

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
static void mouseCallback(GLFWwindow* window, int key, int scancode, int action);

GLFWwindow* createOpenglWindow(int width, int height, const char* title)
{
	GLFWwindow* window;
	// Initialize glfw.
	if (!glfwInit())
		return NULL;
	glfwWindowHint(GLFW_OPENGL_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	window = glfwCreateWindow(width, height, title, NULL, NULL);
	if (!window)
		return NULL;
	return window;
}

glGraphicsDevice* createOpenglDevice(GLFWwindow* window)
{
	return glGraphicsDevice::create(window);
}

void hookInputCallbacks(GLFWwindow* window)
{
	glfwSetKeyCallback(window, keyCallback);
	glfwSetMouseButtonCallback(window, mouseCallback);
}

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS)
		Input::handleKeyEvent(key, EVENT_PRESS);
	else if (action == GLFW_RELEASE)
		Input::handleKeyEvent(key, EVENT_RELEASE);
}

static void mouseCallback(GLFWwindow * window, int button, int action, int mods)
{
	if (action == GLFW_PRESS)
		Input::handleButtonEvent(button, EVENT_PRESS);
	else if (action == GLFW_RELEASE)
		Input::handleButtonEvent(button, EVENT_RELEASE);
}
