#include "gl_framebuffer.h"
#include "gl_texture.h"
#include <assert.h>


glFrameBuffer* glFrameBuffer::create(const std::vector<glTexture*>& color_tex, glTexture* depth_tex)
{
	glFrameBuffer* created = new glFrameBuffer;

	created->m_color_tex = color_tex;
	created->m_depth_tex = depth_tex;
	assert(color_tex.size() > 0);
	created->m_width = ((glTexture*)color_tex[0])->getWidth();
	created->m_height = ((glTexture*)color_tex[0])->getHeight();
	// Checks.
	assert(created->m_width == ((glTexture*)depth_tex)->getWidth());
	assert(created->m_height == ((glTexture*)depth_tex)->getHeight());
	for (int i = 0; i < color_tex.size(); i++)
	{
		Format format = color_tex[i]->getFormat();
		assert(format != FORMAT_DEPTH && format != FORMAT_STENCIL && format != FORMAT_DEPTH_STENCIL);
		assert(created->m_width == ((glTexture*)color_tex[i])->getWidth());
		assert(created->m_height == ((glTexture*)color_tex[i])->getHeight());
	}
	// Initialize.
	glGenFramebuffers(1, &created->m_id);
	glBindFramebuffer(GL_FRAMEBUFFER, created->m_id);
	// Attach textures.
	for (int i = 0; i< created->m_color_tex.size(); i++)
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, ((glTexture*)created->m_color_tex[i])->m_id, 0);
	if (created->m_depth_tex)
	{
		GLuint attachment = GL_DEPTH_ATTACHMENT;
		switch (created->m_depth_tex->getFormat()) 
		{
		case FORMAT_DEPTH: attachment = GL_DEPTH_ATTACHMENT; break;
		case FORMAT_STENCIL: attachment = GL_STENCIL_ATTACHMENT; break;
		case FORMAT_DEPTH_STENCIL: attachment = GL_DEPTH_STENCIL_ATTACHMENT; break;
		default: assert(0);
		}
		glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, ((glTexture*)created->m_depth_tex)->m_id, 0);
	}
	// Check status.
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	assert(status == GL_FRAMEBUFFER_COMPLETE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return created;
}

glFrameBuffer::~glFrameBuffer()
{
	glDeleteFramebuffers(1, &m_id);
}

std::vector<glTexture*> glFrameBuffer::getColorTextures()
{
	return m_color_tex;
}

glTexture* glFrameBuffer::getColorTexture(unsigned index)
{
	assert(index < m_color_tex.size());
	return m_color_tex[index];
}

unsigned glFrameBuffer::getColorTextureCount()
{
	return m_color_tex.size();
}

glTexture* glFrameBuffer::getDepthTexture()
{
	return m_depth_tex;
}

int glFrameBuffer::getWidth()
{
	return m_width;
}

int glFrameBuffer::getHeight()
{
	return m_height;
}

void glFrameBuffer::generateMipmaps()
{
	for (glTexture* tex : m_color_tex)
		tex->generateMipmaps();
	m_depth_tex->generateMipmaps();
}

void glFrameBuffer::bind()
{
	glBindFramebuffer(GL_FRAMEBUFFER, m_id);
	glViewport(0, 0, m_width, m_height);
}

void glFrameBuffer::unbind()
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
