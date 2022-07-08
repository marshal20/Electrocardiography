#include <stdlib.h>
#include "renderer2d.h"
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
#version 330 core
layout (location = 0) in vec2 pos;

uniform mat4 projection;
uniform mat4 model;

void main()
{
	gl_Position = projection*model*vec4(pos, 0.0, 1.0); // w = 1 for points, w = 0 for vectors.
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

// Sprite shader.
static const char* sprite_vert = R"(
#version 330 core
layout (location = 0) in vec2 pos;
layout (location = 1) in vec2 uv_in;

uniform mat4 projection;
uniform mat4 model;

layout(location = 0) out vec2 uv;

void main()
{
	uv = uv_in;
	gl_Position = projection*model*vec4(pos, 0.0, 1.0); // w = 1 for points, w = 0 for vectors.
}
)";
static const char* sprite_frag = R"(
#version 330 core
layout(location = 0) in vec2 uv;

uniform vec4 color;
uniform sampler2D texture;

out vec4 FragColor;

void main()
{
	FragColor = color*texture2D(texture, uv);
}
)";

static glShader* simple_shader;
static glVertexLayout* simple_layout;
static glm::mat4 projection = glm::mat4(1);
static Renderer2D::Style style = { true, 1.0f, {0, 0, 0, 1}, true, {1, 1, 1, 1} };

struct Vertex
{
	glm::vec2 pos;
};

struct SpriteVertex
{
	glm::vec2 pos;
	glm::vec2 uv;
};

static glVertexBuffer* vertex_buffer;
static Vertex vertices[MAX_VERTICES];
static const int circle_segments = 64;
static glVertexBuffer* circle_vertex_buffer;
static glVertexBuffer* sprite_vertex_buffer;
static glShader* sprite_shader;
static glVertexLayout* sprite_layout;
static bool renderer2d_is_initialized = false;

static void createCircle(int segments, Vertex* vertices)
{
	const float increment = 2 * PI / segments;
	for (int i = 0; i < segments; i++)
	{
		float theta = i * increment;
		vertices[i] = { glm::vec2(cos(theta), sin(theta)) };
	}
}

void Renderer2D::init()
{
	// check for double initialization
	if (renderer2d_is_initialized)
	{
		return;
	}
	renderer2d_is_initialized = true;

	// Initialize vertex buffer.
	vertex_buffer = gdevGet()->createVertexBuffer(MAX_VERTICES * sizeof(Vertex), USAGE_DYNAMIC);
	// Initialize circle.
	{
		Vertex temp[circle_segments];
		createCircle(circle_segments, temp);
		circle_vertex_buffer = gdevGet()->createVertexBuffer(circle_segments * sizeof(Vertex), USAGE_STATIC, temp);
	}
	// Initialize shader.
	simple_shader = gdevGet()->createShader(simple_vert, simple_frag);
	simple_layout = gdevGet()->createVertexLayout({
		{VertexLayoutElement::VEC2, "pos" }
		});
	simple_shader->bind();
	simple_shader->setMat4("projection", glm::mat4(1));
	simple_shader->setMat4("model", glm::mat4(1));
	// Sprite variables.
	SpriteVertex v[6] =
	{
		// First triangle.
		{{-1, -1}, {0, 0}},
		{{ 1, -1}, {1, 0}},
		{{ 1,  1}, {1, 1}},
		// Second triangle.
		{{-1, -1}, {0, 0}},
		{{ 1,  1}, {1, 1}},
		{{-1,  1}, {0, 1}}
	};
	sprite_vertex_buffer = gdevGet()->createVertexBuffer(sizeof(v), USAGE_STATIC, v);
	sprite_shader = gdevGet()->createShader(sprite_vert, sprite_frag);
	sprite_layout = gdevGet()->createVertexLayout({
		{VertexLayoutElement::VEC2, "pos" },
		{VertexLayoutElement::VEC2, "uv" }
		});
}

void Renderer2D::cleanup()
{
	if (renderer2d_is_initialized)
	{
		delete vertex_buffer;
		delete circle_vertex_buffer;
		delete simple_shader;
		delete simple_layout;
		delete sprite_vertex_buffer;
		delete sprite_shader;
		delete sprite_layout;
		renderer2d_is_initialized = false;
	}
}

void Renderer2D::setProjection(const glm::mat4 & proj)
{
	projection = proj;
}

void Renderer2D::setStyle(const Style & _style)
{
	style = _style;
}

void Renderer2D::fill(const glm::vec4 & color)
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

void Renderer2D::drawLine(const glm::vec2 & p1, const glm::vec2 & p2)
{
	Vertex line[2] = {
		{ p1 },
		{ p2 }
	};
	// Update buffer.
	vertex_buffer->update(0, sizeof(line), line);

	bind_global_buffers();
	// Draw command.
	if (style.stroke)
	{
		simple_shader->setVec4("color", style.stroke_color);
		gdevGet()->setLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LOOP, 0, 2);
	}
}

void Renderer2D::drawTriangle(const glm::vec2 & p1, const glm::vec2 & p2, const glm::vec2 & p3)
{
	Vertex triangle[3] = {
	{ p1 },
	{ p2 },
	{ p3 }
	};
	// Update buffer.
	vertex_buffer->update(0, sizeof(triangle), triangle);

	bind_global_buffers();
	// Draw command.
	if (style.fill)
	{
		simple_shader->setVec4("color", style.fill_color);
		gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_FAN, 0, 3);
	}
	if (style.stroke)
	{
		simple_shader->setVec4("color", style.stroke_color);
		gdevGet()->setLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LOOP, 0, 3);
	}
}

void Renderer2D::drawRect(const glm::vec2 & p1, const glm::vec2 & p2)
{
	drawQuad({ p1.x, p1.y }, { p1.x, p2.y }, { p2.x, p2.y }, { p2.x, p1.y });
}

void Renderer2D::drawQuad(const glm::vec2 & p1, const glm::vec2 & p2, const glm::vec2 & p3, const glm::vec2 & p4)
{
	Vertex quad[4] = {
		{ p1 }, { p2 }, { p3 }, {p4}
	};
	// Update buffer.
	vertex_buffer->update(0, sizeof(quad), quad);

	bind_global_buffers();
	// Draw command.
	if (style.fill)
	{
		simple_shader->setVec4("color", style.fill_color);
		gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_FAN, 0, 4);
	}
	if (style.stroke)
	{
		simple_shader->setVec4("color", style.stroke_color);
		gdevGet()->setLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LOOP, 0, 4);
	}
}

void Renderer2D::drawPolygon(const glm::vec2 * points, int count)
{
	count = (count <= MAX_VERTICES) ? count : MAX_VERTICES;
	for (int i = 0; i < count; i++)
		vertices[i] = { points[i] };
	// Update buffer.
	vertex_buffer->update(0, count * sizeof(Vertex), vertices);

	bind_global_buffers();
	// Draw command.
	if (style.fill)
	{
		simple_shader->setVec4("color", style.fill_color);
		gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_FAN, 0, count);
	}
	if (style.stroke)
	{
		simple_shader->setVec4("color", style.stroke_color);
		gdevGet()->setLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LOOP, 0, count);
	}
}

void Renderer2D::drawCircle(const glm::vec2 & p, float r)
{
	drawEllipse(p, r, r);
}

void Renderer2D::drawEllipse(const glm::vec2 & p, float rx, float ry)
{
	simple_shader->bind();
	simple_shader->setMat4("projection", projection);
	glm::mat4 model = translate({ p, 0 }) * scale({ rx, ry, 1 });
	simple_shader->setMat4("model", model);
	circle_vertex_buffer->bind();
	simple_layout->bind();
	// Draw command.
	if (style.fill)
	{
		simple_shader->setVec4("color", style.fill_color);
		gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_FAN, 0, circle_segments);
	}
	if (style.stroke)
	{
		simple_shader->setVec4("color", style.stroke_color);
		gdevGet()->setLineWidth(style.stroke_width);
		gdevGet()->drawArrays(TOPOLOGY_LINE_LOOP, 0, circle_segments);
	}
	// Reset model matrix.
	simple_shader->setMat4("model", glm::mat4(1));
}

void Renderer2D::drawSprite(const Sprite & sprite)
{
	glm::mat4 model = translate({ sprite.position, 0 }) * scale({ sprite.size / 2.0f, 1 });
	// Bind shader.
	sprite_shader->bind();
	sprite_shader->setMat4("projection", projection);
	sprite_shader->setMat4("model", model);
	sprite_shader->setVec4("color", sprite.color);
	// Texture.
	sprite_shader->setInt("texture", 0);
	sprite.texture->bind(0);
	// Bind buffers.
	sprite_vertex_buffer->bind();
	sprite_layout->bind();
	// Draw command.
	gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_LIST, 0, 6);
}

void Renderer2D::drawTexture(const glm::vec2& position, const glm::vec2& size, glTexture* texture, const glm::vec4& color)
{
	glm::mat4 model = translate({ position, 0 }) * scale({ size / 2.0f, 1 });
	// Bind shader.
	sprite_shader->bind();
	sprite_shader->setMat4("projection", projection);
	sprite_shader->setMat4("model", model);
	sprite_shader->setVec4("color", color);
	// Texture.
	sprite_shader->setInt("texture", 0);
	texture->bind(0);
	// Bind buffers.
	sprite_vertex_buffer->bind();
	sprite_layout->bind();
	// Draw command.
	gdevGet()->drawArrays(TOPOLOGY_TRIANGLE_LIST, 0, 6);
}
