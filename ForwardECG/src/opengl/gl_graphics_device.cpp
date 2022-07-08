#include "gl_headers.h"
#include <GLFW/glfw3.h>
#include <assert.h>
#include "gl_graphics_device.h"
//#include "gl_shader.h"
//#include "gl_texture.h"
//#include "gl_vertex_buffer.h"
//#include "gl_index_buffer.h"
//#include "gl_vertex_layout.h"
//#include "gl_framebuffer.h"
//#include "gl_uniform_buffer.h"


static GLenum comparison_to_glenum(Comparison comp)
{
	switch (comp)
	{
	case COMPARISON_NEVER:         return GL_NEVER;
	case COMPARISON_ALWAYS:        return GL_ALWAYS;
	case COMPARISON_EQUAL:         return GL_EQUAL;
	case COMPARISON_NOT_EQUAL:     return GL_NOTEQUAL;
	case COMPARISON_LESS:          return GL_LESS;
	case COMPARISON_LESS_EQUAL:    return GL_LEQUAL;
	case COMPARISON_GREATER:       return GL_GREATER;
	case COMPARISON_GREATER_EQUAL: return GL_GEQUAL;
	default: assert(0);
	}
	return NULL;
}

static GLenum factor_to_glenum(Alpha::Factor factor)
{
	switch (factor)
	{
	case Alpha::ZERO:                return GL_ZERO;
	case Alpha::ONE:                 return GL_ONE;
	case Alpha::SRC_ALPHA:           return GL_SRC_ALPHA;
	case Alpha::ONE_MINUS_SRC_ALPHA: return GL_ONE_MINUS_SRC_ALPHA;
	case Alpha::DST_ALPHA:           return GL_DST_ALPHA;
	case Alpha::ONE_MINUS_DST_ALPHA: return GL_ONE_MINUS_DST_ALPHA;
	default: assert(0);
	}
	return NULL;
}

static GLenum topology_to_glenum(Topology topology)
{
	switch (topology)
	{
	case TOPOLOGY_POINT_LIST:     return GL_POINTS;
	case TOPOLOGY_LINE_LIST:      return GL_LINES;
	case TOPOLOGY_LINE_STRIP:     return GL_LINE_STRIP;
	case TOPOLOGY_LINE_LOOP:      return GL_LINE_LOOP;
	case TOPOLOGY_TRIANGLE_LIST:  return GL_TRIANGLES;
	case TOPOLOGY_TRIANGLE_STRIP: return GL_TRIANGLE_STRIP;
	case TOPOLOGY_TRIANGLE_FAN:   return GL_TRIANGLE_FAN;
	default: assert(0);
	}
	return NULL;
}

static GLenum index_type_to_glenum(IndexType i_type)
{
	switch (i_type)
	{
	case INDEX_UNSIGNED_BYTE:     return GL_UNSIGNED_BYTE;
	case INDEX_UNSIGNED_SHORT:    return GL_UNSIGNED_SHORT;
	case INDEX_UNSIGNED_INT:      return GL_UNSIGNED_INT;
	default: assert(0);
	}
	return NULL;
}

glGraphicsDevice* glGraphicsDevice::create(GLFWwindow* window)
{
	glfwMakeContextCurrent(window);

	// Initialize gl3w
	if (gl3wInit2(glfwGetProcAddress))
	{
		printf("Error: Failed to initialize gl3w\n");
		return NULL;
	}

	// check OpenGL version
	if (!gl3wIsSupported(3, 3))
	{
		printf("Error: OpenGL 3.3 isn't supported on this system\n");
		return NULL;
	}

	// create new graphics device
	glGraphicsDevice* created = new glGraphicsDevice();
	created->m_width = 0;
	created->m_height = 0;

	// Create a dummy VAO.
	glGenVertexArrays(1, &created->m_dummy_vao);
	glBindVertexArray(created->m_dummy_vao);

	// Width and height.
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	created->m_width = width;
	created->m_height = height;

	return created;
}

glGraphicsDevice::~glGraphicsDevice()
{
	glDeleteVertexArrays(1, &m_dummy_vao);
}

void glGraphicsDevice::begin()
{

}

void glGraphicsDevice::end()
{

}

void glGraphicsDevice::resizeBackbuffer(int width, int height)
{
	// As the window handling is done by the window library, 
	// we don't have to do any back-buffer resizing.
	m_width = width;
	m_height = height;
}

int glGraphicsDevice::getBackbufferWidth() const
{
	return m_width;
}

int glGraphicsDevice::getBackbufferHeight() const
{
	return m_height;
}

void glGraphicsDevice::viewport(unsigned x, unsigned y, unsigned width, unsigned height)
{
	glViewport(x, y, width, height);
}

void glGraphicsDevice::bindBackbuffer()
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, m_width, m_height);
}

void glGraphicsDevice::clearColorBuffer(float r, float g, float b, float a)
{
	glClearColor(r, g, b, a);
	glClear(GL_COLOR_BUFFER_BIT);
}

void glGraphicsDevice::clearDepthBuffer(float value)
{
	glClearDepth(value);
	glClear(GL_DEPTH_BUFFER_BIT);
}

void glGraphicsDevice::clearStencilBuffer(int value)
{
	glClearStencil(value);
	glClear(GL_STENCIL_BUFFER_BIT);
}

void glGraphicsDevice::depthTest(State state)
{
	if (state == STATE_DISABLED)
		glDisable(GL_DEPTH_TEST);
	else if (state == STATE_ENABLED)
		glEnable(GL_DEPTH_TEST);
}

void glGraphicsDevice::stencilTest(State state)
{
	if (state == STATE_DISABLED)
		glDisable(GL_STENCIL_TEST);
	else if (state == STATE_ENABLED)
		glEnable(GL_STENCIL_TEST);
}

void glGraphicsDevice::depthFunc(Comparison comp)
{
	glDepthFunc(comparison_to_glenum(comp));
}

void glGraphicsDevice::stencilFunc(Comparison comp, int ref, unsigned mask)
{
	glStencilFunc(comparison_to_glenum(comp), ref, mask);
}

void glGraphicsDevice::setAlpha(const Alpha& alpha)
{
	if (alpha.blend)
		glEnable(GL_BLEND);
	else
		glDisable(GL_BLEND);
	glBlendFunc(factor_to_glenum(alpha.sfactor), factor_to_glenum(alpha.dfactor));
}

void glGraphicsDevice::setPolygonMode(const FaceDirection& apply_to_face_dir, const PolygonMode& poly_mode)
{
	// select face
	GLenum face = 0;
	switch (apply_to_face_dir)
	{
	case FACE_BACK:
		face = GL_BACK;
		break;
	case FACE_FRONT:
		face = GL_FRONT;
		break;
	case FACE_FRONT_AND_BACK:
		face = GL_FRONT_AND_BACK;
		break;
	default:
		face = GL_FRONT_AND_BACK;
		break;
	}

	// select polygon mode
	GLenum mode = 0;
	switch (poly_mode)
	{
	case POLYGON_MODE_FILL:
		mode = GL_FILL;
		break;
	case POLYGON_MODE_LINE:
		mode = GL_LINE;
		break;
	case POLYGON_MODE_POINT:
		mode = GL_POINT;
		break;
	default:
		face = GL_FILL;
		break;
	}

	glPolygonMode(face, mode);

}

void glGraphicsDevice::setLineWidth(float line_width)
{
	glLineWidth(line_width);
}

void glGraphicsDevice::setPointSize(float point_size)
{
	setPointSize(point_size);
}

void glGraphicsDevice::drawArrays(Topology topology, unsigned first, unsigned count)
{
	glDrawArrays(topology_to_glenum(topology), first, count);
}

void glGraphicsDevice::drawElements(Topology topology, unsigned first, unsigned count, IndexType i_type)
{
	glDrawElements(topology_to_glenum(topology), count, index_type_to_glenum(i_type), (const void*)first);
}

glShader* glGraphicsDevice::createShader(const char* vertex_src, const char* fragment_src)
{
	return glShader::create(vertex_src, fragment_src);
}

glTexture* glGraphicsDevice::createTexture(unsigned int width, unsigned int height, Format format, Type type)
{
	return glTexture::create(width, height, format, type);
}

glVertexBuffer* glGraphicsDevice::createVertexBuffer(unsigned int size, Usage usage, const void* data)
{
	return glVertexBuffer::create(size, usage, data);
}

glIndexBuffer* glGraphicsDevice::createIndexBuffer(unsigned int size, Usage usage, const void* data)
{
	return glIndexBuffer::create(size, usage, data);
}

glUniformBuffer* glGraphicsDevice::createUniformBuffer(unsigned int size, Usage usage, const void* data)
{
	return glUniformBuffer::create(size, usage, data);
}

glVertexLayout* glGraphicsDevice::createVertexLayout(const VertexLayoutElementList& element_list)
{
	return glVertexLayout::create(element_list);
}

glFrameBuffer* glGraphicsDevice::createFramebuffer(const std::vector<glTexture*>& color_tex, glTexture* depth_tex)
{
	return glFrameBuffer::create(color_tex, depth_tex);
}
