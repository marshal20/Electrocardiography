#pragma once
#include <glad/glad.h>
#include "definitions.h"
#include <glm/glm.hpp>

class glShader
{
public:
	static glShader* create(const char* vertex_src, const char* fragment_src);

	~glShader();

	void bind();
	void unbind();

	UniformId getUniformId(const char* name);

	void setFloat(UniformId id, float val);
	void setInt(UniformId id, int val);
	void setVec2(UniformId id, const glm::vec2& val);
	void setVec3(UniformId id, const glm::vec3& val);
	void setVec4(UniformId id, const glm::vec4& val);
	void setMat2(UniformId id, const glm::mat2& val);
	void setMat3(UniformId id, const glm::mat3& val);
	void setMat4(UniformId id, const glm::mat4& val);

private:
	GLuint m_id;
};
