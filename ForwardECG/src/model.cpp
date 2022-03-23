#include "model.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "main_dev.h"
#include "opengl/gl_texture.h"
#include <glm/glm.hpp>
#include "forward_renderer.h"
#include "stb/stb_image.h"

// TODO: Make a material loading system.
// TODO: Make a model format.


Material Material::general()
{
	return { NULL, glm::vec3(1, 1, 1), 0.1f, 1.0, 32 };
}

#define MAX_DEPTH 10

typedef ForwardRenderer::Vertex Vertex;

static glm::mat4 aimat4Convert(const aiMatrix4x4& m)
{
	return {
		m[0][0], m[1][0], m[2][0], m[3][0],
		m[0][1], m[1][1], m[2][1], m[3][1],
		m[0][2], m[1][2], m[2][2], m[3][2],
		m[0][3], m[1][3], m[2][3], m[3][3]
	};
}

static glm::vec3 aivec3Convert(const aiVector3D& v)
{
	return { v.x, v.y, v.z };
}

// We have to flip the image vertically because
// the first element of data corresponds to the top left, 
// but opengl expects the first element corresponds to the bottom left.

static void flipImageVertically(uint32_t* buffer, unsigned int width, unsigned int height)
{
	uint32_t* src, * dst, tmp;
	for (int i = 0; i < height / 2; i++)
		for (int j = 0; j < width; j++)
		{
			src = &buffer[i * width + j];
			dst = &buffer[(height - i - 1) * width + j];
			tmp = *src;
			*src = *dst;
			*dst = tmp;
		}
}

glTexture* loadImageToTexture(const char* file_names)
{
	int width, height;
	stbi_uc* data = stbi_load(file_names, &width, &height, NULL, 4);
	if (!data)
		return NULL;
	flipImageVertically((uint32_t*)data, width, height);
	glTexture* tex = gdevGet()->createTexture(width, height, FORMAT_RGBA, TYPE_UNSIGNED_BYTE);
	tex->update(0, 0, width, height, FORMAT_RGBA, TYPE_UNSIGNED_BYTE, data);
	stbi_image_free(data);
	return tex;
}

static Material parseMaterial(const aiScene* scene, aiMesh* mesh)
{
	Material material = Material::general();
	if (mesh->mMaterialIndex >= 0)
	{
		aiMaterial* aimat = scene->mMaterials[mesh->mMaterialIndex];
		// diffuse color.
		aiColor3D col;
		aimat->Get(AI_MATKEY_COLOR_DIFFUSE, col);
		material.diffuse_color = { col.r, col.g, col.b };
		// diffuse.
		aiString tex_path;
		aimat->Get(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0), tex_path);
		if (tex_path.data[0] == '*') // Embeded
		{
			unsigned int tex_idx = atoi(&tex_path.data[1]);
			aiTexture* tex = scene->mTextures[tex_idx];
			//diffuse_tex = get_gdev()->create_texture(width, height, FORMAT_RGBA, TYPE_UNSIGNED_BYTE);

			// FOR DEBUGGING:
			printf("Found embeded texture: %s\n", tex_path.C_Str());
		}
		else if (tex_path != aiString(""))
		{
			material.diffuse_tex = loadImageToTexture(tex_path.C_Str());

			// FOR DEBUGGING:
			printf("Found texture: %s\n", tex_path.C_Str());
		}
	}
	return material;
}

static void processNode(
	aiNode* node, const aiScene* scene, Model& model, 
	std::vector<Vertex>& vertices_temp, 
	std::vector<unsigned int> indices_temp, 
	const glm::mat4& parent_transform = glm::mat4(1), 
	unsigned depth = 0)
{
	glm::mat4 this_transform = parent_transform * aimat4Convert(node->mTransformation);
	glm::mat3 this_transform_rot = this_transform;

	for (int i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		if (!mesh->HasFaces())
			continue;
		const unsigned vertex_count = mesh->mNumVertices;
		const unsigned index_count = 3 * mesh->mNumFaces;
		vertices_temp.clear();
		indices_temp.clear();
		// Load vertices.
		for (int j = 0; j < vertex_count; j++)
		{
			glm::vec3 pos = aivec3Convert(mesh->mVertices[j]);
			glm::vec3 normal = aivec3Convert(mesh->mNormals[j]);
			glm::vec2 tex_coords = glm::vec2(0, 0);
			if(mesh->mTextureCoords[0])
				tex_coords = aivec3Convert(mesh->mTextureCoords[0][j]);
			vertices_temp.push_back({ pos, normal, tex_coords });
		}
		// Load indices.
		for (int k = 0; k < mesh->mNumFaces; k++)
		{
			indices_temp.push_back(mesh->mFaces[k].mIndices[0]);
			indices_temp.push_back(mesh->mFaces[k].mIndices[1]);
			indices_temp.push_back(mesh->mFaces[k].mIndices[2]);
		}
		// Transform positions and normals.
		for (Vertex& vertex : vertices_temp)
		{
			vertex.pos = this_transform * glm::vec4(vertex.pos, 1);
			vertex.normal = this_transform_rot * vertex.normal;
		}
		// Material.
		Material material = parseMaterial(scene, mesh);
		// Load buffers into GPU.
		glVertexBuffer* vertex_buffer = gdevGet()->createVertexBuffer(vertex_count * sizeof(Vertex), USAGE_STATIC, &vertices_temp[0]);
		glIndexBuffer* index_buffer = gdevGet()->createIndexBuffer(index_count * sizeof(int), USAGE_STATIC, &indices_temp[0]);
		model.mesh_list.push_back({ vertex_buffer, index_buffer, index_count, material });
	}

	// Process child nodes.
	if(depth <= MAX_DEPTH)
		for (int i = 0; i < node->mNumChildren; i++)
			processNode(node->mChildren[i], scene, model, vertices_temp, indices_temp, this_transform, depth+1);
}


Model loadModelFile(const char* file_name)
{
	Model model;
	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(file_name,
		aiProcessPreset_TargetRealtime_Quality |
		aiProcess_CalcTangentSpace | aiProcess_Triangulate |
		aiProcess_JoinIdenticalVertices | aiProcess_SortByPType |
		aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);

	if (!scene)
	{
		printf("Assimp importer error: %s\n", importer.GetErrorString());
		assert(false);
	}

	std::vector<Vertex> vertices_temp;
	std::vector<unsigned int> indices_temp;
	processNode(scene->mRootNode, scene, model, vertices_temp, indices_temp);
	return model;
}

void freeModel(Model model)
{
	for (Mesh& mesh : model.mesh_list)
	{
		delete mesh.vertex_buffer;
		delete mesh.index_buffer;
	}
}

void freeMesh(Mesh mesh)
{
	delete mesh.vertex_buffer;
	delete mesh.index_buffer;
}
