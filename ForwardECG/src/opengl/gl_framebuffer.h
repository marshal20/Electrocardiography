#pragma once
#include <glad/glad.h>
#include "definitions.h"

class glTexture;

class glFrameBuffer
{
public:
	static glFrameBuffer* create(const std::vector<glTexture*>& color_tex, glTexture* depth_tex);

	~glFrameBuffer();

	std::vector<glTexture*> getColorTextures();
	glTexture* getColorTexture(unsigned index);
	unsigned getColorTextureCount();
	glTexture* getDepthTexture();
	int getWidth();
	int getHeight();

	void generateMipmaps();
	void bind();
	void unbind();

private:
	GLuint m_id;
	std::vector<glTexture*> m_color_tex;
	glTexture* m_depth_tex;
	int m_width, m_height;

};

