#include <stdlib.h>
#include "renderer3d.h"
#include "transform.h"
#include "main_dev.h"
#include "opengl/gl_shader.h"
#include "opengl/gl_texture.h"
#include "opengl/gl_vertex_buffer.h"
#include "opengl/gl_vertex_layout.h"


#define PI 3.14159265359
#define MAX_VERTICES 1024

// Simple shader.
static const char* simple_vert = R"(
#version 330 core
layout (location = 0) in vec3 pos;

uniform mat4 projection;
uniform mat4 model;

void main()
{
	gl_Position = projection*model*vec4(pos, 1.0); // w = 1 for points, w = 0 for vectors.
}
)";
static const char* simple_frag = R"(
#version 330 core

uniform vec4 color;

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
static bool renderer3d_is_initialized = false;

struct Vertex
{
	glm::vec3 pos;
};

static glVertexBuffer* vertex_buffer;
static Vertex vertices[MAX_VERTICES];

void Renderer3D::init()
{
	// check for double initialization
	if (renderer3d_is_initialized)
	{
		return;
	}
	renderer3d_is_initialized = true;

	// Initialize vertex buffer.
	vertex_buffer = gdevGet()->createVertexBuffer(MAX_VERTICES * sizeof(Vertex), USAGE_DYNAMIC);
	// Initialize shader.
	simple_shader = gdevGet()->createShader(simple_vert, simple_frag);
	simple_layout = gdevGet()->createVertexLayout({
		{VertexLayoutElement::VEC3, "pos" }
		});
	simple_shader->bind();
	simple_shader->setMat4("projection", glm::mat4(1));
	simple_shader->setMat4("model", glm::mat4(1));
}

void Renderer3D::cleanup()
{
	if (renderer3d_is_initialized)
	{
		delete vertex_buffer;
		delete simple_shader;
		delete simple_layout;
		renderer3d_is_initialized = false;
	}
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
	simple_shader->setMat4("projection", projection);
	// Bind buffer.
	vertex_buffer->bind();
	simple_layout->bind();
}

void Renderer3D::drawLine(const glm::vec3 & p1, const glm::vec3 & p2)
{
	// Update buffer.
	Vertex line[2] = {
		{ p1 },
		{ p2 }
	};
	vertex_buffer->update(0, sizeof(line), line);

	bind_global_buffers();
	simple_shader->setMat4("model", glm::mat4(1));
	// Draw command.
	if (style.stroke)
	{
		simple_shader->setVec4("color", style.stroke_color);
		glLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LIST, 0, 2);
	}
}

void Renderer3D::drawPolygon(const glm::vec3* points, int count, bool loop)
{
	int new_count = (count <= MAX_VERTICES) ? count : MAX_VERTICES;

	for (int i = 0; i < new_count; i++)
		vertices[i] = { points[i] };
	bind_global_buffers();
	simple_shader->setMat4("model", glm::mat4(1));
	// Update buffer.
	vertex_buffer->update(0, new_count * sizeof(Vertex), vertices);
	// Draw command.
	if (style.fill)
	{
		simple_shader->setVec4("color", style.fill_color);
		gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_FAN, 0, new_count);
	}
	if (style.stroke)
	{
		simple_shader->setVec4("color", style.stroke_color);
		glLineWidth(style.stroke_width);
		Topology topology = loop ? TOPOLOGY_LINE_LOOP : TOPOLOGY_LINE_STRIP;
		gdevGet()->drawArrays(topology, 0, new_count);
	}

	// render the rest of points
	if (count > MAX_VERTICES)
	{
		drawPolygon(&points[MAX_VERTICES-1], count-MAX_VERTICES, loop);
	}
}

void Renderer3D::drawPoint(const glm::vec3& point, const glm::vec4& color, float size)
{
	// Update buffer.
	Vertex line[1] = {
	{ point }
	};
	vertex_buffer->update(0, sizeof(line), line);

	bind_global_buffers();
	simple_shader->setMat4("model", glm::mat4(1));
	// Draw command.
	simple_shader->setVec4("color", color);
	glPointSize(size);
	gdevGet()->drawArrays(TOPOLOGY_POINT_LIST, 0, 1);
}

