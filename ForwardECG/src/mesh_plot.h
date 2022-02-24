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

MeshPlot load_mesh_plot(const char* file_name);
void free_mesh_plot(MeshPlot& mesh_plot);


class MeshPlotRenderer
{
public:
	MeshPlotRenderer();
	~MeshPlotRenderer();

	void set_view_projection_matrix(const glm::mat4& view);
	void render_mesh_plot(const glm::mat4& transform, MeshPlot* mesh);

private:
	glShader* m_plot_shader;
	glVertexLayout* m_vertex_layout;
	glm::mat4 m_view_matrix;
};

