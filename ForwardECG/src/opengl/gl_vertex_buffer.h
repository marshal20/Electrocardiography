#pragma once
#include "gl_headers.h"
#include "definitions.h"

class glVertexBuffer
{
public:
	static glVertexBuffer* create(unsigned int size, Usage usage, const void* data);

	~glVertexBuffer();

	void update(unsigned offset, unsigned size, const void* data);

	void bind();
	void unbind();

private:
	GLuint m_id;
	unsigned m_size;
};
