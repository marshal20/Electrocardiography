#pragma once
#include "gl_headers.h"
#include "definitions.h"

class glTexture
{
public:
	static glTexture* create(unsigned int width, unsigned int height, Format format, Type type);

	~glTexture();

	void update(unsigned xoff, unsigned yoff, unsigned width, unsigned height, Format format, Type type, const void* data);

	Format getFormat() const;
	Type getType() const;
	unsigned int getWidth() const;
	unsigned int getHeight() const;

	void generateMipmaps();
	void bind(unsigned int slot);
	void ubind();

private:
	friend class glFrameBuffer;
	GLuint m_id;
	unsigned int m_width, m_height;
	Format m_format;
	Type m_type;
};

