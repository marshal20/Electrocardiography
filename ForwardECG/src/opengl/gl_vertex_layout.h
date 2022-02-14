#pragma once
#include <glad/glad.h>
#include "definitions.h"

class glVertexLayout
{
public:
	static glVertexLayout* create(const VertexLayoutElementList& element_list);

	~glVertexLayout();

	void bind();
	void unbind();

private:
	VertexLayoutElementList m_element_list;
	unsigned m_size;
};
