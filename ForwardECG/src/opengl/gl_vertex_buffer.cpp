#include "gl_vertex_buffer.h"
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

glVertexBuffer* glVertexBuffer::create(unsigned int size, Usage usage, const void* data)
{
	glVertexBuffer* created = new glVertexBuffer();
	created->m_size = size;

	glGenBuffers(1, &created->m_id);
	glBindBuffer(GL_ARRAY_BUFFER, created->m_id);
	glBufferData(GL_ARRAY_BUFFER, size, data, usage_to_glenum(usage));
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	return created;
}

glVertexBuffer::~glVertexBuffer()
{
	glDeleteBuffers(1, &m_id);
}

void glVertexBuffer::update(unsigned offset, unsigned size, const void* data)
{
	assert((offset + size) <= m_size);

	glBindBuffer(GL_ARRAY_BUFFER, m_id);
	glBufferSubData(GL_ARRAY_BUFFER, offset, size, data);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void glVertexBuffer::bind()
{
	glBindBuffer(GL_ARRAY_BUFFER, m_id);
}

void glVertexBuffer::unbind()
{
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
