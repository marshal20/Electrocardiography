#include "gl_texture.h"
#include <assert.h>

// TODO: better error handling.


static GLenum format_to_glenum(Format format)
{
	switch (format)
	{
	case FORMAT_DEPTH:         return GL_DEPTH_COMPONENT;
	case FORMAT_STENCIL:       return GL_STENCIL_INDEX;
	case FORMAT_DEPTH_STENCIL: return GL_DEPTH_STENCIL;
	case FORMAT_RED:           return GL_RED;
	case FORMAT_RG:            return GL_RG;
	case FORMAT_RGB:           return GL_RGB;
	case FORMAT_RGBA:          return GL_RGBA;
	default: assert(0);
	}
	return NULL;
}

static GLenum type_to_glenum(Type type)
{
	switch (type)
	{
	case TYPE_BYTE:           return GL_BYTE;
	case TYPE_UNSIGNED_BYTE:  return GL_UNSIGNED_BYTE;
	case TYPE_SHORT:          return GL_SHORT;
	case TYPE_UNSIGNED_SHORT: return GL_UNSIGNED_SHORT;
	case TYPE_INT:            return GL_INT;
	case TYPE_UNSIGNED_INT:   return GL_UNSIGNED_INT;
	case TYPE_HALF_FLOAT:     return GL_HALF_FLOAT;
	case TYPE_FLOAT:          return GL_FLOAT;
	default: assert(0);
	}
	return NULL;
}

glTexture* glTexture::create(unsigned int width, unsigned int height, Format format, Type type)
{
	glTexture* created = new glTexture();
	created->m_width = width;
	created->m_height = height;
	created->m_format = format;
	created->m_type = type;

	glGenTextures(1, &created->m_id);
	glBindTexture(GL_TEXTURE_2D, created->m_id);
	glTexImage2D(GL_TEXTURE_2D, 0, format_to_glenum(format), width, height, 0, format_to_glenum(format), type_to_glenum(type), 0);
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	return created;
}

glTexture::~glTexture()
{
	glDeleteTextures(1, &m_id);
}

void glTexture::update(unsigned xoff, unsigned yoff, unsigned width, unsigned height, Format format, Type type, const void* data)
{
	glBindTexture(GL_TEXTURE_2D, m_id);
	glTexSubImage2D(GL_TEXTURE_2D, 0, xoff, yoff, width, height, format_to_glenum(format), type_to_glenum(type), data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
}

Format glTexture::getFormat() const
{
	return m_format;
}

Type glTexture::getType() const
{
	return m_type;
}

unsigned int glTexture::getWidth() const
{
	return m_width;
}

unsigned int glTexture::getHeight() const
{
	return m_height;
}

void glTexture::generateMipmaps()
{
	glBindTexture(GL_TEXTURE_2D, m_id);
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void glTexture::bind(unsigned int slot)
{
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_2D, m_id);
}

void glTexture::ubind()
{
	glBindTexture(GL_TEXTURE_2D, 0);
}
