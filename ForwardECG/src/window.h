#pragma once

struct GLFWwindow;

class glGraphicsDevice;

GLFWwindow* createOpenglWindow(int width, int height, const char* title);
glGraphicsDevice* createOpenglDevice(GLFWwindow* window);
void hookInputCallbacks(GLFWwindow* window);
