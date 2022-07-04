#pragma once
#include <vector>
#include <glm/glm.hpp>


class glVertexLayout;
class glVertexBuffer;
class glIndexBuffer;
class glShader;

struct MeshPlotVertex
{
	glm::vec3 pos;
	glm::vec3 normal;
	float value;
	float opacity;
	int group;
};

struct MeshPlotFace
{
	int idx[3];
};

class MeshPlot
{
public:
	MeshPlot();
	~MeshPlot();

	// Mesh
	std::vector<MeshPlotVertex> vertices;
	std::vector<MeshPlotFace> faces;
	unsigned int faces_count;
	
	// Mesh Vertices Graph
	std::vector<std::vector<int>> vertex_neighbour_vertices;

	// Mesh Groups
	std::vector<std::vector<int>> groups_vertices;

	// GPU buffers
	glVertexBuffer* vertex_buffer;
	glIndexBuffer* index_buffer;

	void create_gpu_buffers();
	void update_gpu_buffers();
};

MeshPlot* load_mesh_plot(const char* file_name);


enum ColorMixType
{
	MIX_RGB,
	MIX_HSV
};

class MeshPlotRenderer
{
public:
	MeshPlotRenderer();
	~MeshPlotRenderer();

	void set_view_projection_matrix(const glm::mat4& view);
	void set_colors(const glm::vec4& color_p, const glm::vec4& color_n);
	void set_color_mix_type(const ColorMixType& color_mix_type);
	void set_values_range(float min_value, float max_value);
	void set_opacity_threshold(float opacity_threshold);
	void set_ambient(float ambient);
	void set_specular(float specular);
	void render_mesh_plot(const glm::mat4& transform, MeshPlot* mesh);

private:
	glShader* m_plot_shader;
	glVertexLayout* m_vertex_layout;
	glm::mat4 m_view_matrix;
	glm::vec4 m_color_p, m_color_n;
	ColorMixType m_color_mix_type;
	float m_min_val;
	float m_max_val;
	float m_opacity_threshold;
	float m_ambient;
	float m_specular;
};

