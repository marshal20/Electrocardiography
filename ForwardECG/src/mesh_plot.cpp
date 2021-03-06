#include "mesh_plot.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "main_dev.h"
#include "transform.h"
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
			mesh_plot.vertices.push_back({ pos, normal, 0, 1.0, -1 });
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
	faces_count(0),
	vertex_buffer(nullptr),
	index_buffer(nullptr)
{

}

MeshPlot::~MeshPlot()
{
	if (vertex_buffer)
	{
		delete vertex_buffer;
	}

	if (index_buffer)
	{
		delete index_buffer;
	}
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
	if (vertex_buffer)
	{
		vertex_buffer->update(0, vertices.size() * sizeof(MeshPlotVertex), &vertices[0]);
	}
	if (index_buffer)
	{
		index_buffer->update(0, faces_count * 3 * sizeof(int), &faces[0]);
	}
}

static void fix_mesh_plot_normals(MeshPlot* mesh)
{
	// zero all normals
	for (MeshPlotVertex& v : mesh->vertices)
	{
		v.normal = { 0, 0, 0 };
	}

	// recalculate normals for each face
	for (const MeshPlotFace& face : mesh->faces)
	{
		glm::vec3 a = mesh->vertices[face.idx[0]].pos;
		glm::vec3 b = mesh->vertices[face.idx[1]].pos;
		glm::vec3 c = mesh->vertices[face.idx[2]].pos;
		glm::vec3 face_normal = glm::normalize(glm::cross((b-a), (c-a)));

		mesh->vertices[face.idx[0]].normal += face_normal;
		mesh->vertices[face.idx[1]].normal += face_normal;
		mesh->vertices[face.idx[2]].normal += face_normal;
	}

	// normalize all normals
	for (MeshPlotVertex& v : mesh->vertices)
	{
		v.normal = glm::normalize(v.normal);
	}
}

static void create_mesh_vertices_graph(MeshPlot* mesh)
{
	// resize vertex_neighbour_vertices to be the same size as vertex count
	mesh->vertex_neighbour_vertices.resize(mesh->vertices.size());

	// add connections based on shared triangles
	for (const MeshPlotFace& face : mesh->faces)
	{
		// 1st vertex
		mesh->vertex_neighbour_vertices[face.idx[0]].push_back(face.idx[1]);
		mesh->vertex_neighbour_vertices[face.idx[0]].push_back(face.idx[2]);
		// 2nd vertex
		mesh->vertex_neighbour_vertices[face.idx[1]].push_back(face.idx[0]);
		mesh->vertex_neighbour_vertices[face.idx[1]].push_back(face.idx[2]);
		// 3rd vertex
		mesh->vertex_neighbour_vertices[face.idx[2]].push_back(face.idx[0]);
		mesh->vertex_neighbour_vertices[face.idx[2]].push_back(face.idx[1]);
	}

	for (std::vector<int>& neighbours : mesh->vertex_neighbour_vertices)
	{
		// sort neighbours
		std::sort(neighbours.begin(), neighbours.end());

		// remove duplicates
		neighbours.erase(std::unique(neighbours.begin(), neighbours.end()), neighbours.end());
	}
}

// depth first search
static void discover_vertices_group(MeshPlot* mesh, int starting_vertex_idx, int group_idx)
{
	// base case
	if (mesh->vertices[starting_vertex_idx].group != -1)
	{
		return;
	}

	// assign group index to the starting vertex
	mesh->vertices[starting_vertex_idx].group = group_idx;

	// discover all neighbours
	for (int neighbour_idx : mesh->vertex_neighbour_vertices[starting_vertex_idx])
	{
		discover_vertices_group(mesh, neighbour_idx, group_idx);
	}
}

static void classify_vertices_into_connected_groups(MeshPlot* mesh)
{
	int group_idx = 0;

	// classify vertices into groups
	for (int i = 0; i < mesh->vertices.size(); i++)
	{
		if (mesh->vertices[i].group == -1)
		{
			discover_vertices_group(mesh, i, group_idx);
			group_idx++;
		}
	}

	// construct the group vertices vector
	mesh->groups_vertices.resize(group_idx);
	for (int i = 0; i < mesh->vertices.size(); i++)
	{
		mesh->groups_vertices[mesh->vertices[i].group].push_back(i);
	}

}

MeshPlot* load_mesh_plot(const char* file_name, bool classify_into_groups)
{
	Assimp::Importer importer;
	importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS); // remove normals and tangents
	const aiScene* scene = importer.ReadFile(file_name,
		aiProcessPreset_TargetRealtime_Quality |
		aiProcess_CalcTangentSpace | aiProcess_Triangulate |
		aiProcess_JoinIdenticalVertices | aiProcess_SortByPType |
		aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph |
		aiProcess_RemoveComponent);

	if (!scene)
	{
		printf("Assimp importer error: %s\n", importer.GetErrorString());
		return nullptr;
	}

	MeshPlot* mesh = new MeshPlot;

	process_node(scene->mRootNode, scene, *mesh);

	fix_mesh_plot_normals(mesh);

	create_mesh_vertices_graph(mesh);
	if (classify_into_groups)
	{
		classify_vertices_into_connected_groups(mesh);
	}

	mesh->create_gpu_buffers();
	mesh->update_gpu_buffers();

	return mesh;
}


///////////////////////////////////////////////////////////
// MeshPlotRenderer
///////////////////////////////////////////////////////////


// plot shader

static const char* plot_vert = R"(
#version 330 core
layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 normal;
layout (location = 2) in float value;
layout (location = 3) in float opacity;
layout (location = 4) in float group;

uniform mat4 projection;
uniform mat4 model;

layout (location = 0) out float value_out;
layout (location = 1) out vec3 normal_out;
layout (location = 2) out float opacity_out;

void main()
{
	value_out = value;
	gl_Position = projection*model*vec4(pos, 1.0); // w = 1 for points, w = 0 for vectors.
	normal_out = (projection*model*vec4(normal, 0)).xyz;
	opacity_out = opacity;
	// ignore group for now (may be used later)
}
)";
static const char* plot_frag = R"(
#version 330 core
layout (location = 0) in float value;
layout (location = 1) in vec3 normal;
layout (location = 2) in float opacity;

uniform vec4 color_n;
uniform vec4 color_p;
uniform float mix_color_hsv; // 0 = RGB, 1 = HSV
uniform float min_val;
uniform float max_val;
uniform float opacity_threshold;
uniform float ambient;
uniform float specular;

out vec4 FragColor;

vec3 rgb2hsv(vec3 c)
{
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

vec3 hsv2rgb(vec3 c)
{
	vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

void main()
{
	float value_normalized = (value-min_val)/(max_val-min_val);
	value_normalized = clamp(value_normalized, 0, 1);	

	float mix_percentage = value_normalized;
	vec3 color_rgb = mix(color_n.xyz, color_p.xyz, mix_percentage);
	vec3 color_hsv = hsv2rgb(mix(rgb2hsv(color_n.xyz), rgb2hsv(color_p.xyz), mix_percentage));
	
	vec3 color = (1-mix_color_hsv)*color_rgb + mix_color_hsv*color_hsv;

	float brightness = ambient + (1-ambient)*max(pow(dot(normal, vec3(0, 0, -1)), specular), 0.0);
	color = color * brightness;

	// debug
	//color = normal/2+0.5 + 0.01*color;
	
	// discard fragment if opacity is less than the opacity_threshold
	if (opacity < opacity_threshold)
	{
		discard;
	}

	FragColor = vec4(color, mix(color_n.a, color_p.a, mix_percentage)*opacity);
}
)";


// wireframe shader

static const char* wireframe_vert = R"(
#version 330 core
layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 normal;
layout (location = 2) in float value;
layout (location = 3) in float opacity;
layout (location = 4) in float group;

uniform mat4 projection;
uniform mat4 model;

layout (location = 2) out float opacity_out;

void main()
{
	gl_Position = projection*model*vec4(pos, 1.0); // w = 1 for points, w = 0 for vectors.
	gl_Position.z -= 0.001; // advance the depth a little bit
	opacity_out = opacity;
}
)";
static const char* wireframe_frag = R"(
#version 330 core
layout (location = 2) in float opacity;

uniform vec4 color;
uniform float opacity_threshold;

out vec4 FragColor;

void main()
{
	// discard fragment if opacity is less than the opacity_threshold
	if (opacity < opacity_threshold)
	{
		discard;
	}

	FragColor = vec4(color.xyz, color.w*opacity);
}
)";


MeshPlotRenderer::MeshPlotRenderer()
{
	m_plot_shader = gdevGet()->createShader(plot_vert, plot_frag);
	m_wireframe_shader = gdevGet()->createShader(wireframe_vert, wireframe_frag);
	m_vertex_layout = gdevGet()->createVertexLayout({
		{VertexLayoutElement::VEC3, "pos"},
		{VertexLayoutElement::VEC3, "normal"},
		{VertexLayoutElement::FLOAT, "value"},
		{VertexLayoutElement::FLOAT, "opacity"},
		{VertexLayoutElement::INT, "group"}
		});
	m_view_matrix = glm::mat4(1);
	m_color_mix_type = MIX_HSV;
	m_color_n = glm::vec4(0, 0, 1, 1);
	m_color_p = glm::vec4(1, 0, 0, 1);
	m_min_val = -1;
	m_max_val = 1;
	m_opacity_threshold = 0;
	m_ambient = 0.8;
	m_specular = 5;
}

MeshPlotRenderer::~MeshPlotRenderer()
{
	delete m_plot_shader;
	delete m_wireframe_shader;
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

void MeshPlotRenderer::set_color_mix_type(const ColorMixType& color_mix_type)
{
	m_color_mix_type = color_mix_type;
}

void MeshPlotRenderer::set_values_range(float min_value, float max_value)
{
	m_min_val = min_value;
	m_max_val = max_value;
}

void MeshPlotRenderer::set_opacity_threshold(float opacity_threshold)
{
	m_opacity_threshold = opacity_threshold;
}

void MeshPlotRenderer::set_ambient(float ambient)
{
	m_ambient = ambient;
}

void MeshPlotRenderer::set_specular(float specular)
{
	m_specular = specular;
}

void MeshPlotRenderer::render_mesh_plot(const glm::mat4& transform, MeshPlot* mesh_plot, bool render_wireframe, const glm::vec4& wireframe_color, float wireframe_line_width)
{
	// check for vertex and index buffer
	if (!mesh_plot->vertex_buffer || !mesh_plot->index_buffer)
	{
		return;
	}

	// Drawing.
	mesh_plot->vertex_buffer->bind();
	mesh_plot->index_buffer->bind();
	m_vertex_layout->bind();


	// plot render

	// Bind shader.
	m_plot_shader->bind();
	m_plot_shader->setMat4("projection", m_view_matrix);
	m_plot_shader->setMat4("model", transform);
	m_plot_shader->setVec4("color_n", m_color_n);
	m_plot_shader->setVec4("color_p", m_color_p);
	m_plot_shader->setFloat("mix_color_hsv", m_color_mix_type == MIX_RGB ? 0 : 1);
	m_plot_shader->setFloat("min_val", m_min_val);
	m_plot_shader->setFloat("max_val", m_max_val);
	m_plot_shader->setFloat("opacity_threshold", m_opacity_threshold);
	m_plot_shader->setFloat("ambient", m_ambient);
	m_plot_shader->setFloat("specular", m_specular);

	gdevGet()->setPolygonMode(FACE_FRONT_AND_BACK, POLYGON_MODE_FILL);
	gdevGet()->drawElements(TOPOLOGY_TRIANGLE_LIST, 0, 3*mesh_plot->faces_count, INDEX_UNSIGNED_INT);


	// wireframe render
	if (render_wireframe)
	{
		// Bind shader.
		m_wireframe_shader->bind();
		m_wireframe_shader->setMat4("projection", m_view_matrix);
		m_wireframe_shader->setMat4("model", transform);
		m_wireframe_shader->setVec4("color", wireframe_color);
		m_wireframe_shader->setFloat("opacity_threshold", m_opacity_threshold);

		gdevGet()->setPolygonMode(FACE_FRONT_AND_BACK, POLYGON_MODE_LINE);
		gdevGet()->setLineWidth(wireframe_line_width);
		gdevGet()->drawElements(TOPOLOGY_TRIANGLE_LIST, 0, 3*mesh_plot->faces_count, INDEX_UNSIGNED_INT);
	}


	// restore polygon mode
	gdevGet()->setPolygonMode(FACE_FRONT_AND_BACK, POLYGON_MODE_FILL);
}

