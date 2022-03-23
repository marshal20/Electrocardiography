#include "gl_uniform_buffer.h"
#include <assert.h>


static GLenum usage_to_glenum(Usage usage)
{
	switch (usage)
	{
	case USAGE_STATIC: return GL_STATIC_DRAW;
	case USAGE_DYNAMIC: return GL_DYNAMIC_DRAW;
	default: assert(0);
	}
	return NULL;
}

glUniformBuffer* glUniformBuffer::create(unsigned int size, Usage usage, const void* data)
{
	glUniformBuffer* created = new glUniformBuffer();
	created->m_size = size;

	glGenBuffers(1, &created->m_id);
	glBindBuffer(GL_UNIFORM_BUFFER, created->m_id);
	glBufferData(GL_UNIFORM_BUFFER, size, data, usage_to_glenum(usage));
	glBindBuffer(GL_UNIFORM_BUFFER, 0);

	return created;
}

void glUniformBuffer::update(unsigned offset, unsigned size, const void* data)
{
	assert((offset + size) <= m_size);

	glBindBuffer(GL_UNIFORM_BUFFER, m_id);
	glBufferSubData(GL_UNIFORM_BUFFER, offset, size, data);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void glUniformBuffer::bind(unsigned index)
{
	glBindBufferBase(GL_UNIFORM_BUFFER, index, m_id);
}

void glUniformBuffer::unbind()
{
	// NOTHING.
}
