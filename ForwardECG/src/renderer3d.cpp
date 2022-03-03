#include <stdlib.h>
#include "renderer3d.h"
#include "transform.h"
#include "main_dev.h"
#include "opengl/gl_shader.h"
#include "opengl/gl_texture.h"
#include "opengl/gl_vertex_buffer.h"
#include "opengl/gl_vertex_layout.h"


#define PI 3.14159265359
#define MAX_VERTICES 256

// Simple shader.
static const char* simple_vert = R"(
#version 450 core
layout (location = 0) in vec3 pos;

layout (location = 0) uniform mat4 projection;
layout (location = 1) uniform mat4 model;

void main()
{
	gl_Position = projection*model*vec4(pos, 1.0); // w = 1 for points, w = 0 for vectors.
}
)";
static const char* simple_frag = R"(
#version 450 core

layout(location = 2) uniform vec4 color;

out vec4 FragColor;

void main()
{
	FragColor = color;
}
)";

static glShader* simple_shader;
static glVertexLayout* simple_layout;
static glm::mat4 projection = glm::mat4(1);
static Renderer3D::Style style = { true, 1.0f, {0, 0, 0, 1}, true, {1, 1, 1, 1} };

struct Vertex
{
	glm::vec3 pos;
};

static glVertexBuffer* vertex_buffer;
static Vertex vertices[MAX_VERTICES];

void Renderer3D::init()
{
	// Initialize vertex buffer.
	vertex_buffer = gdevGet()->createVertexBuffer(MAX_VERTICES * sizeof(Vertex), USAGE_DYNAMIC);
	// Initialize shader.
	simple_shader = gdevGet()->createShader(simple_vert, simple_frag);
	simple_layout = gdevGet()->createVertexLayout({
		{VertexLayoutElement::VEC3, "pos" }
		});
	simple_shader->bind();
	simple_shader->setMat4(0, glm::mat4(1));
	simple_shader->setMat4(1, glm::mat4(1));
}

void Renderer3D::cleanup()
{
	delete vertex_buffer;
	delete simple_shader;
	delete simple_layout;
}

void Renderer3D::setProjection(const glm::mat4 & proj)
{
	projection = proj;
}

void Renderer3D::setStyle(const Style & _style)
{
	style = _style;
}

void Renderer3D::fill(const glm::vec4 & color)
{
	gdevGet()->clearColorBuffer(color.x, color.y, color.z, color.w);
}

static void bind_global_buffers()
{
	// Bind shader.
	simple_shader->bind();
	// Projection.
	simple_shader->setMat4(0, projection);
	// Bind buffer.
	vertex_buffer->bind();
	simple_layout->bind();
}

void Renderer3D::drawLine(const glm::vec3 & p1, const glm::vec3 & p2)
{
	Vertex line[2] = {
		{ p1 },
		{ p2 }
	};
	bind_global_buffers();
	// Update buffer.
	vertex_buffer->update(0, sizeof(line), line);
	// Draw command.
	if (style.stroke)
	{
		simple_shader->setVec4(2, style.stroke_color);
		glLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LOOP, 0, 2);
	}
}

void Renderer3D::drawPolygon(const glm::vec3* points, int count)
{
	count = (count <= MAX_VERTICES) ? count : MAX_VERTICES;
	for (int i = 0; i < count; i++)
		vertices[i] = { points[i] };
	bind_global_buffers();
	// Update buffer.
	vertex_buffer->update(0, count * sizeof(Vertex), vertices);
	// Draw command.
	if (style.fill)
	{
		simple_shader->setVec4(2, style.fill_color);
		gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_FAN, 0, count);
	}
	if (style.stroke)
	{
		simple_shader->setVec4(2, style.stroke_color);
		glLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LOOP, 0, count);
	}

	// render the rest of points
	if (count > MAX_VERTICES)
	{
		drawPolygon(&points[MAX_VERTICES-1], count-MAX_VERTICES);
	}
}

