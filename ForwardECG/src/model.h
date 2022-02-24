#pragma once
#include <vector>
#include <glm/glm.hpp>

class glVertexBuffer;
class glIndexBuffer;
class glTexture;

struct Material
{
	glTexture* diffuse_tex;
	glm::vec3 diffuse_color;
	float ambient;
	float specular;
	float shininess;

	static Material general();
};

struct Mesh
{
	glVertexBuffer* vertex_buffer;
	glIndexBuffer* index_buffer;
	unsigned int index_count;
	Material material;
};

struct Model
{
	std::vector<Mesh> mesh_list;
};

glTexture* loadImageToTexture(const char* file_name);
Model loadModelFile(const char* file_name);
void freeModel(Model model);

