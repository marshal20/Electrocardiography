#include "gl_shader.h"
#include <glad/glad.h>
#include <vector>


static bool compile_shader(GLuint id, const char* source)
{
	const GLint len = strlen(source);
	glShaderSource(id, 1, &source, &len);
	glCompileShader(id);

	GLint isCompiled = 0;
	glGetShaderiv(id, GL_COMPILE_STATUS, &isCompiled);
	if (isCompiled == GL_FALSE)
	{
		GLint maxLength = 0;
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &maxLength);
		std::vector<GLchar> infoLog(maxLength);
		glGetShaderInfoLog(id, maxLength, &maxLength, &infoLog[0]);
		// Log error.
		printf("%s\n", &infoLog[0]);
		glDeleteShader(id);
		return false;
	}
	return true;
}

glShader* glShader::create(const char* vertex_src, const char* fragment_src)
{
	glShader* created = new glShader();

	GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	bool res;
	res = compile_shader(vertex_shader, vertex_src);
	assert(res);
	res = compile_shader(fragment_shader, fragment_src);
	assert(res);

	created->m_id = glCreateProgram();
	assert(created->m_id != 0);
	glAttachShader(created->m_id, vertex_shader);
	glAttachShader(created->m_id, fragment_shader);
	glLinkProgram(created->m_id);

	// Check if the program is linked successfully.
	GLint isLinked = 0;
	glGetProgramiv(created->m_id, GL_LINK_STATUS, (int*)& isLinked);
	if (isLinked == GL_FALSE)
	{
		GLint maxLength = 0;
		glGetProgramiv(created->m_id, GL_INFO_LOG_LENGTH, &maxLength);
		std::vector<GLchar> infoLog(maxLength);
		glGetProgramInfoLog(created->m_id, maxLength, &maxLength, &infoLog[0]);
		// Log error.
		printf("%s\n", &infoLog[0]);
		assert(0);
		glDeleteProgram(created->m_id);
		glDeleteShader(vertex_shader);
		glDeleteShader(fragment_shader);
		assert(0);
		return 0;
	}

	glDetachShader(created->m_id, vertex_shader);
	glDetachShader(created->m_id, fragment_shader);
	
	return created;
}

glShader::~glShader()
{
	glDeleteProgram(m_id);
}

void glShader::bind()
{
	glUseProgram(m_id);
}

void glShader::unbind()
{
	glUseProgram(0);
}

UniformId glShader::getUniformId(const char* name)
{
	UniformId id = glGetUniformLocation(m_id, name);
	if (id == -1)
	{
		printf("Warning: Couldn't find %s uniform\n", name);
	}

	return id;
}

void glShader::setFloat(UniformId id, float val)
{
	glProgramUniform1f(m_id, id, val);
}

void glShader::setInt(UniformId id, int val)
{
	glProgramUniform1i(m_id, id, val);
}

void glShader::setVec2(UniformId id, const glm::vec2& val)
{
	glProgramUniform2fv(m_id, id, 1, (const GLfloat*)&val);
}

void glShader::setVec3(UniformId id, const glm::vec3& val)
{
	glProgramUniform3fv(m_id, id, 1, (const GLfloat*)& val);
}

void glShader::setVec4(UniformId id, const glm::vec4& val)
{
	glProgramUniform4fv(m_id, id, 1, (const GLfloat*)& val);
}

void glShader::setMat2(UniformId id, const glm::mat2& val)
{
	glProgramUniformMatrix2fv(m_id, id, 1, GL_FALSE, (const GLfloat*)& val);
}

void glShader::setMat3(UniformId id, const glm::mat3& val)
{
	glProgramUniformMatrix3fv(m_id, id, 1, GL_FALSE, (const GLfloat*)& val);
}

void glShader::setMat4(UniformId id, const glm::mat4& val)
{
	glProgramUniformMatrix4fv(m_id, id, 1, GL_FALSE, (const GLfloat*)& val);
}


void glShader::setFloatName(const char* name, float val)
{
	setFloat(getUniformId(name), val);
}

void glShader::setIntName(const char* name, int val)
{
	setInt(getUniformId(name), val);
}

void glShader::setVec2Name(const char* name, const glm::vec2& val)
{
	setVec2(getUniformId(name), val);
}

void glShader::setVec3Name(const char* name, const glm::vec3& val)
{
	setVec3(getUniformId(name), val);
}

void glShader::setVec4Name(const char* name, const glm::vec4& val)
{
	setVec4(getUniformId(name), val);
}

void glShader::setMat2Name(const char* name, const glm::mat2& val)
{
	setMat2(getUniformId(name), val);
}

void glShader::setMat3Name(const char* name, const glm::mat3& val)
{
	setMat3(getUniformId(name), val);
}

void glShader::setMat4Name(const char* name, const glm::mat4& val)
{
	setMat4(getUniformId(name), val);
}

