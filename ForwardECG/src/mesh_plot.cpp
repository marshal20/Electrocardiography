#include "mesh_plot.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "main_dev.h"
#include "opengl/gl_texture.h"
#include "opengl/gl_shader.h"
#include "opengl/gl_vertex_buffer.h"
#include "opengl/gl_index_buffer.h"
#include "opengl/gl_vertex_layout.h"


#define MAX_DEPTH 10


static glm::mat4 aimat4_convert(const aiMatrix4x4& m)
{
	return {
		m[0][0], m[1][0], m[2][0], m[3][0],
		m[0][1], m[1][1], m[2][1], m[3][1],
		m[0][2], m[1][2], m[2][2], m[3][2],
		m[0][3], m[1][3], m[2][3], m[3][3]
	};
}

static glm::vec3 aivec3_convert(const aiVector3D& v)
{
	return { v.x, v.y, v.z };
}

static void process_node(aiNode* node, const aiScene* scene, MeshPlot& mesh_plot, unsigned int depth = 0)
{
	glm::mat4 this_transform = aimat4_convert(node->mTransformation);
	glm::mat3 this_transform_rot = this_transform;

	for (int i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		if (!mesh->HasFaces())
			continue;

		const unsigned vertex_count = mesh->mNumVertices;
		const unsigned face_count = mesh->mNumFaces;

		// Load vertices.
		for (int j = 0; j < vertex_count; j++)
		{
			glm::vec3 pos = aivec3_convert(mesh->mVertices[j]);
			glm::vec3 normal = aivec3_convert(mesh->mNormals[j]);
			mesh_plot.vertices.push_back({ pos, normal, 0 });
		}
		// Load indecies.
		for (int k = 0; k < mesh->mNumFaces; k++)
		{
			MeshPlotFace face;
			face.idx[0] = mesh->mFaces[k].mIndices[0];
			face.idx[1] = mesh->mFaces[k].mIndices[1];
			face.idx[2] = mesh->mFaces[k].mIndices[2];
			mesh_plot.faces.push_back(face);
		}
		// Transform positions and normals.
		for (MeshPlotVertex& vertex : mesh_plot.vertices)
		{
			vertex.pos = this_transform * glm::vec4(vertex.pos, 1);
			vertex.value = 0;
		}
	}

	mesh_plot.faces_count = mesh_plot.faces.size();

	// Process child nodes.
	if (depth <= MAX_DEPTH)
		for (int i = 0; i < node->mNumChildren; i++)
			process_node(node->mChildren[i], scene, mesh_plot, depth+1);
}

MeshPlot::MeshPlot() :
	faces_count(0)
{

}

MeshPlot::~MeshPlot()
{

}

void MeshPlot::create_gpu_buffers()
{
	// Load buffers into GPU.
	vertex_buffer = gdevGet()->createVertexBuffer(vertices.size() * sizeof(MeshPlotVertex), USAGE_DYNAMIC, NULL);
	index_buffer = gdevGet()->createIndexBuffer(faces_count * 3 * sizeof(int), USAGE_DYNAMIC, NULL);
}

void MeshPlot::update_gpu_buffers()
{
	// update buffers into GPU.
	vertex_buffer->update(0, vertices.size() * sizeof(MeshPlotVertex), &vertices[0]);
	index_buffer->update(0, faces_count * 3 * sizeof(int), &faces[0]);
}

MeshPlot load_mesh_plot(const char* file_name)
{
	MeshPlot mesh;
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

	process_node(scene->mRootNode, scene, mesh);
	mesh.create_gpu_buffers();
	mesh.update_gpu_buffers();

	return mesh;
}

void free_mesh_plot(MeshPlot& mesh_plot)
{
	if (mesh_plot.vertex_buffer)
	{
		delete mesh_plot.vertex_buffer;
	}

	if (mesh_plot.index_buffer)
	{
		delete mesh_plot.index_buffer;
	}
}


///////////////////////////////////////////////////////////
// MeshPlotRenderer
///////////////////////////////////////////////////////////


static const char* plot_vert = R"(
#version 450 core
layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 normal;
layout (location = 2) in float value;

layout (location = 0) uniform mat4 view_proj;
layout (location = 1) uniform mat4 model;

layout (location = 0) out vec3 frag_position;
layout (location = 1) out float value_out;

void main()
{
	value_out = value;
	frag_position = (model*vec4(pos, 1.0)).xyz;
	gl_Position = view_proj*model*vec4(pos, 1.0); // w = 1 for points, w = 0 for vectors.
}
)";
static const char* plot_frag = R"(
#version 450 core
layout (location = 0) in vec3 frag_position;
layout (location = 1) in float value;

uniform vec4 color_n;
uniform vec4 color_p;
uniform float max_val;

out vec4 FragColor;

void main()
{
	float mix_percentage = (value/max_val+1)/2;
	vec4 color = mix(color_n, color_p, mix_percentage);

	FragColor = color;
}
)";

MeshPlotRenderer::MeshPlotRenderer()
{
	m_plot_shader = gdevGet()->createShader(plot_vert, plot_frag);
	m_vertex_layout = gdevGet()->createVertexLayout({
		{VertexLayoutElement::VEC3, "pos"},
		{VertexLayoutElement::VEC3, "normal"},
		{VertexLayoutElement::FLOAT, "value"}
		});
	m_view_matrix = glm::mat4(1);

	m_color_n = glm::vec4(0, 0, 1, 1);
	m_color_p = glm::vec4(1, 0, 0, 1);
	m_max_val = 1;
}

MeshPlotRenderer::~MeshPlotRenderer()
{
	delete m_plot_shader;
	delete m_vertex_layout;
}

void MeshPlotRenderer::set_view_projection_matrix(const glm::mat4& view)
{
	m_view_matrix = view;
}

void MeshPlotRenderer::set_colors(const glm::vec4& color_p, const glm::vec4& color_n)
{
	m_color_p = color_p;
	m_color_n = color_n;
}

void MeshPlotRenderer::set_max_val(float max_val)
{
	m_max_val = max_val;
}

void MeshPlotRenderer::render_mesh_plot(const glm::mat4& transform, MeshPlot* mesh_plot)
{
	m_plot_shader->bind();
	// Transform.
	m_plot_shader->setMat4(m_plot_shader->getUniformId("view_proj"), m_view_matrix);
	m_plot_shader->setMat4(m_plot_shader->getUniformId("model"), transform);
	m_plot_shader->setVec4(m_plot_shader->getUniformId("color_n"), m_color_n);
	m_plot_shader->setVec4(m_plot_shader->getUniformId("color_p"), m_color_p);
	m_plot_shader->setFloat(m_plot_shader->getUniformId("max_val"), m_max_val);

	// Drawing.
	mesh_plot->vertex_buffer->bind();
	mesh_plot->index_buffer->bind();
	m_vertex_layout->bind();

	gdevGet()->drawElements(TOPOLOGY_TRIANGLE_LIST, 0, 3*mesh_plot->faces_count, INDEX_UNSIGNED_INT);
}

