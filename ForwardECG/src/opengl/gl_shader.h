#pragma once
#include "gl_headers.h"
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

	void setFloatId(UniformId id, float val);
	void setIntId(UniformId id, int val);
	void setVec2Id(UniformId id, const glm::vec2& val);
	void setVec3Id(UniformId id, const glm::vec3& val);
	void setVec4Id(UniformId id, const glm::vec4& val);
	void setMat2Id(UniformId id, const glm::mat2& val);
	void setMat3Id(UniformId id, const glm::mat3& val);
	void setMat4Id(UniformId id, const glm::mat4& val);

	void setFloat(const char* name, float val);
	void setInt(const char* name, int val);
	void setVec2(const char* name, const glm::vec2& val);
	void setVec3(const char* name, const glm::vec3& val);
	void setVec4(const char* name, const glm::vec4& val);
	void setMat2(const char* name, const glm::mat2& val);
	void setMat3(const char* name, const glm::mat3& val);
	void setMat4(const char* name, const glm::mat4& val);

private:
	GLuint m_id;
};
