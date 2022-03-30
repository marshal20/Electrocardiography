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
	void set_max_val(float max_val);
	void set_ambient(float ambient);
	void render_mesh_plot(const glm::mat4& transform, MeshPlot* mesh);

private:
	glShader* m_plot_shader;
	glVertexLayout* m_vertex_layout;
	glm::mat4 m_view_matrix;
	glm::vec4 m_color_p, m_color_n;
	ColorMixType m_color_mix_type;
	float m_max_val;
	float m_ambient;
};

