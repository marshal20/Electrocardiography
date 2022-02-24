#pragma once
#include <glad/glad.h>
#include "definitions.h"
#include "gl_shader.h"
#include "gl_texture.h"
#include "gl_vertex_buffer.h"
#include "gl_index_buffer.h"
#include "gl_vertex_layout.h"
#include "gl_framebuffer.h"
#include "gl_uniform_buffer.h"

struct GLFWwindow;

//class glShader;
//class glTexture;
//class glVertexBuffer;
//class glIndexBuffer;
//class glUniformBuffer;
//class glVertexLayout;
//class glFrameBuffer;

class glGraphicsDevice
{
public:
	static glGraphicsDevice* create(GLFWwindow* window);

	~glGraphicsDevice();

	void begin();
	void end();

	void resizeBackbuffer(int width, int height);
	int getBackbufferWidth() const;
	int getBackbufferHeight() const;

	void viewport(unsigned x, unsigned y, unsigned width, unsigned height);
	void bindBackbuffer();

	void clearColorBuffer(float r, float g, float b, float a);
	void clearDepthBuffer(float value);
	void clearStencilBuffer(int value);
	void depthTest(State state);
	void stencilTest(State state);
	void depthFunc(Comparison comp);
	void stencilFunc(Comparison comp, int ref, unsigned mask);
	void setAlpha(const Alpha& alpha);

	void drawArrays(Topology topology, unsigned first, unsigned count);
	void drawElements(Topology topology, unsigned first, unsigned count, IndexType i_type);

	glShader* createShader(const char* vertex_src, const char* fragment_src);
	glTexture* createTexture(unsigned int width, unsigned int height, Format format, Type type);
	glVertexBuffer* createVertexBuffer(unsigned int size, Usage usage, const void* data = nullptr);
	glIndexBuffer* createIndexBuffer(unsigned int size, Usage usage, const void* data = nullptr);
	glUniformBuffer* createUniformBuffer(unsigned int size, Usage usage, const void* data = nullptr);
	glVertexLayout* createVertexLayout(const VertexLayoutElementList& element_list);
	glFrameBuffer* createFramebuffer(const std::vector<glTexture*>& color_tex, glTexture* depth_tex);

private:
	unsigned int m_dummy_vao;
	int m_width, m_height;
};
