#pragma once
#include <glm/glm.hpp>
#include "forward_renderer.h"
#include "model.h"
#include "camera.h"

class glFrameBuffer;

class AxisRenderer
{
public:
	AxisRenderer(int width, int height);
	~AxisRenderer();

	void render(const LookAtCamera& camera);
	glTexture* get_texture();

private:
	Mesh m_xarrow_mesh;
	ForwardRenderer* m_fr;
	int m_width, m_height;
	glFrameBuffer* m_frame_buffer;
};
