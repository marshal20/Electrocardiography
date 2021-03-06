#include "forward_renderer.h"
#include "main_dev.h"
#include "opengl/gl_texture.h"
#include "opengl/gl_shader.h"
#include "opengl/gl_vertex_buffer.h"
#include "opengl/gl_index_buffer.h"
#include "opengl/gl_vertex_layout.h"
#include "model.h"

// TODO: Move material variables to a struct (uniform buffer).
// TODO: Move light variables to a struct and use multiple (maybe 8) with light count.
// TODO: Add directional light.


static const char* diffuse_vert = R"(
#version 330 core
layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex_coords;

uniform mat4 view_proj;
uniform mat4 model;

layout (location = 0) out vec3 frag_position;
layout (location = 1) out vec3 normal_out;
layout (location = 2) out vec2 tex_coords_out;

void main()
{
	normal_out = normalize((model*vec4(normal, 0)).xyz);
	tex_coords_out = vec2(tex_coords.x, 1-tex_coords.y);
	frag_position = (model*vec4(pos, 1.0)).xyz;
	gl_Position = view_proj*model*vec4(pos, 1.0); // w = 1 for points, w = 0 for vectors.
}
)";
static const char* diffuse_frag = R"(
#version 330 core
layout (location = 0) in vec3 frag_position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex_coords;

// Material.
uniform vec3 diffuse_color;
uniform int sample_diffuse;
uniform sampler2D diffuse_tex;
uniform float ambient_value;
uniform float specular_strength;
uniform float shininess;

// Point light.
uniform vec3 light_position;
uniform vec3 light_color;
uniform vec3 view_pos;

out vec4 FragColor;

void main()
{
	vec3 light_dir = normalize(light_position - frag_position);
	vec3 view_dir = normalize(view_pos - frag_position);
	vec3 reflect_dir = normalize(reflect(-light_dir, normal));	

	vec3 diff_color = diffuse_color;
	if(sample_diffuse == 1)
		diff_color = texture2D(diffuse_tex, tex_coords).xyz;
	
	// Ambient.
	vec3 ambient = ambient_value * light_color;
	// Diffuse.
	float diff = max(dot(light_dir, normal), 0.0);
	vec3 diffuse = diff * light_color;
	// Specular.
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0), shininess);
	vec3 specular = specular_strength * spec * light_color; 	

	vec3 color = (ambient + diffuse + specular) * diff_color;

	FragColor = vec4(color, 1.0);
	// DEBUGGING:
	//FragColor = vec4(tex_coords, 0, 1.0);
	//float temp = gl_FragCoord.z*0.5+0.5;
	//FragColor = vec4(temp, temp, temp, 1.0);
}
)";

ForwardRenderer::ForwardRenderer()
{
	m_diffuse_shader = gdevGet()->createShader(diffuse_vert, diffuse_frag);
	m_diffuse_layout = gdevGet()->createVertexLayout({
		{VertexLayoutElement::VEC3, "pos"},
		{VertexLayoutElement::VEC3, "normal"},
		{VertexLayoutElement::VEC2, "tex_coords"}
		});
	m_view_matrix = glm::mat4(1);
}

ForwardRenderer::~ForwardRenderer()
{
	delete m_diffuse_shader;
	delete m_diffuse_layout;
}

void ForwardRenderer::setViewProjectionMatrix(const glm::mat4& view)
{
	m_view_matrix = view;
}

void ForwardRenderer::renderMesh(const glm::mat4& transform, Mesh* mesh)
{
	// TODO: abstract lighting.
	//const glm::vec3 light_posision = { 0, 10, 0 };
	const glm::vec3 light_posision = view_pos;
	const glm::vec3 light_color = { 1, 1, 1 };
	//const glm::vec3 view_pos = { 6, 0, 0 };

	m_diffuse_shader->bind();
	// Transform.
	m_diffuse_shader->setMat4("view_proj", m_view_matrix);
	m_diffuse_shader->setMat4("model", transform);
	// Material.
	m_diffuse_shader->setVec3("diffuse_color", mesh->material.diffuse_color);
	if (mesh->material.diffuse_tex)
	{
		m_diffuse_shader->setInt("sample_diffuse", 1);
		mesh->material.diffuse_tex->bind(1);
	}
	else
	{
		m_diffuse_shader->setInt("sample_diffuse", 0);
	}
	m_diffuse_shader->setFloat("ambient_value", mesh->material.ambient);
	m_diffuse_shader->setFloat("specular_strength", mesh->material.specular);
	m_diffuse_shader->setFloat("shininess", mesh->material.shininess);
	// Light.
	m_diffuse_shader->setVec3("light_position", light_posision);
	m_diffuse_shader->setVec3("light_color", light_color);
	m_diffuse_shader->setVec3("view_pos", view_pos);
	
	// Drawing.
	mesh->vertex_buffer->bind();
	mesh->index_buffer->bind();
	m_diffuse_layout->bind();

	gdevGet()->drawElements(TOPOLOGY_TRIANGLE_LIST, 0, mesh->index_count, INDEX_UNSIGNED_INT);
}

void ForwardRenderer::renderModel(const glm::mat4& transform, Model* model)
{
	for (int i = 0; i < model->mesh_list.size(); i++)
		renderMesh(transform, &model->mesh_list[i]);
}
