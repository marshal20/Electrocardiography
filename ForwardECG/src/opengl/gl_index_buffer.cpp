#include "gl_index_buffer.h"
#include "gl_headers.h"
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

glIndexBuffer* glIndexBuffer::create(unsigned int size, Usage usage, const void* data)
{
	glIndexBuffer* created = new glIndexBuffer();
	created->m_size = size;

	glGenBuffers(1, &created->m_id);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, created->m_id);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, size, data, usage_to_glenum(usage));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	return created;
}

glIndexBuffer::~glIndexBuffer()
{
	glDeleteBuffers(1, &m_id);
}

void glIndexBuffer::update(unsigned offset, unsigned size, const void* data)
{
	assert((offset + size) <= m_size);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_id);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, offset, size, data);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void glIndexBuffer::bind()
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_id);
}

void glIndexBuffer::unbind()
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
