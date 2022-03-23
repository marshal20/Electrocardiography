#include "gl_vertex_layout.h"
#include "gl_headers.h"
#include <assert.h>
#include <glm/glm.hpp>


static unsigned type_size(VertexLayoutElement::Type type)
{
	switch (type)
	{
	case VertexLayoutElement::INT:   return sizeof(int);
	case VertexLayoutElement::FLOAT: return sizeof(float);
	case VertexLayoutElement::VEC2:  return sizeof(glm::vec2);
	case VertexLayoutElement::VEC3:  return sizeof(glm::vec3);
	case VertexLayoutElement::VEC4:  return sizeof(glm::vec4);
	default: assert(0);
	}
	return 0;
}

static GLenum type_to_glenum(VertexLayoutElement::Type type)
{
	switch (type)
	{
	case VertexLayoutElement::INT:   return GL_INT;
	case VertexLayoutElement::FLOAT: return GL_FLOAT;
	case VertexLayoutElement::VEC2:  return GL_FLOAT;
	case VertexLayoutElement::VEC3:  return GL_FLOAT;
	case VertexLayoutElement::VEC4:  return GL_FLOAT;
	default: assert(0);
	}
	return NULL;
}

static GLenum type_to_count(VertexLayoutElement::Type type)
{
	switch (type)
	{
	case VertexLayoutElement::INT:   return 1;
	case VertexLayoutElement::FLOAT: return 1;
	case VertexLayoutElement::VEC2:  return 2;
	case VertexLayoutElement::VEC3:  return 3;
	case VertexLayoutElement::VEC4:  return 4;
	default: assert(0);
	}
	return 0;
}

glVertexLayout* glVertexLayout::create(const VertexLayoutElementList& element_list)
{
	glVertexLayout* created = new glVertexLayout();

	created->m_element_list = element_list;
	created->m_size = 0;
	for (const VertexLayoutElement& element : element_list)
		created->m_size += type_size(element.type);

	return created;
}

glVertexLayout::~glVertexLayout()
{
	
}

void glVertexLayout::bind()
{
	unsigned offset = 0;
	unsigned location = 0;
	for (const VertexLayoutElement& element : m_element_list)
	{
		glVertexAttribPointer(location, type_to_count(element.type), type_to_glenum(element.type), GL_TRUE, m_size, (const void*)offset);
		glEnableVertexAttribArray(location);
		offset += type_size(element.type);
		location++;
	}
}

void glVertexLayout::unbind()
{

}
