#include "window.h"
#include "opengl/gl_headers.h"
#include <GLFW/glfw3.h>
#include "input.h"
#include "opengl/gl_graphics_device.h"

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
static void mouseCallback(GLFWwindow* window, int key, int scancode, int action);
static void cursorPosCallback(GLFWwindow * window, double xpos, double ypos);
static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
static void glfw_error_handler(int error, const char * description);

GLFWwindow* createOpenglWindow(int width, int height, const char* title)
{
	// set error callback
	glfwSetErrorCallback(glfw_error_handler);

	// Initialize glfw.
	if (!glfwInit())
		return NULL;

	//glfwWindowHint(GLFW_OPENGL_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	return glfwCreateWindow(width, height, title, NULL, NULL);
}

glGraphicsDevice* createOpenglDevice(GLFWwindow* window)
{
	return glGraphicsDevice::create(window);
}

void hookInputCallbacks(GLFWwindow* window)
{
	glfwSetKeyCallback(window, keyCallback);
	glfwSetMouseButtonCallback(window, mouseCallback);
	glfwSetCursorPosCallback(window, cursorPosCallback);
	glfwSetScrollCallback(window, scrollCallback);
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

static void cursorPosCallback(GLFWwindow * window, double xpos, double ypos)
{
	Input::cursorPositionCallback(xpos, ypos);
}

static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
	Input::scrollCallback(xoffset, yoffset);
}

void glfw_error_handler(int error, const char* description)
{
	printf("GLFW error 0x%08X: %s\n", error, description);
}
