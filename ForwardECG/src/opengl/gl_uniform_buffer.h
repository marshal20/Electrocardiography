#pragma once
#include <glad/glad.h>
#include "definitions.h"

class glUniformBuffer
{
public:
	static glUniformBuffer* create(unsigned int size, Usage usage, const void* data);

	void update(unsigned offset, unsigned size, const void* data);

	void bind(unsigned index);
	void unbind();

private:
	GLuint m_id;
	unsigned m_size;
};
