#pragma once
#include "gl_headers.h"
#include "definitions.h"

class glIndexBuffer
{
public:
	static glIndexBuffer* create(unsigned int size, Usage usage, const void* data);

	~glIndexBuffer();

	void update(unsigned offset, unsigned size, const void* data);

	void bind();
	void unbind();

private:
	GLuint m_id;
	unsigned m_size;
};
