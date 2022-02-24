#pragma once
#include <glm/glm.hpp>
#include <vector>

class glShader;

class glVertexLayout;
struct Mesh;
struct Model;

class ForwardRenderer
{
public:
	struct Vertex
	{
		glm::vec3 pos;
		glm::vec3 normal;
		glm::vec2 tex_coords;
	};

public:
	ForwardRenderer();
	~ForwardRenderer();

	void setViewProjectionMatrix(const glm::mat4& view);
	void renderMesh(const glm::mat4& transform, Mesh* mesh);
	void renderModel(const glm::mat4& transform, Model* model);

public:
	// Temporary:
	glm::vec3 view_pos;

private:
	glShader* m_diffuse_shader;
	glVertexLayout* m_diffuse_layout;
	glm::mat4 m_view_matrix;
};
