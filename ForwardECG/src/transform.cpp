#include "transform.h"


// ** Notes:
// - Transpose the matrix because it's column major, for more:
//    https://en.wikipedia.org/wiki/Row-_and_column-major_order
// - Orthographic calculations:
//    https://en.wikipedia.org/wiki/Orthographic_projection

glm::mat4 translate(const glm::vec3& p)
{
	return glm::transpose(glm::mat4{
		1, 0, 0, p.x,
		0, 1, 0, p.y,
		0, 0, 1, p.z,
		0, 0, 0, 1
		});
}

glm::mat4 rotateEulerXYZ(float theta_x, float theta_y, float theta_z)
{
	glm::mat4 rot_x = glm::transpose(glm::mat4{
		1, 0,            0,             0,
		0, cos(theta_x), -sin(theta_x), 0,
		0, sin(theta_x), cos(theta_x),  0,
		0, 0,            0,             1
	});
	glm::mat4 rot_y = glm::transpose(glm::mat4{
		cos(theta_y),  0, sin(theta_y), 0,
		0,             1, 0,            0,
		-sin(theta_y), 0, cos(theta_y), 0,
		0,             0, 0,            1
	});
	glm::mat4 rot_z = glm::transpose(glm::mat4{
		cos(theta_z), -sin(theta_z), 0, 0,
		sin(theta_z), cos(theta_z),  0, 0,
		0,            0,             1, 0,
		0,            0,             0, 1,
	});
	return rot_z * rot_y* rot_x;
}

glm::mat4 scale(const glm::vec3& s)
{
	return glm::transpose(glm::mat4{
		s.x, 0,   0,   0,
		0,   s.y, 0,   0,
		0,   0,   s.z, 0,
		0,   0,   0,   1
		});
}

glm::mat4 ortho(float left, float right, float top, float bottom, float near, float far)
{
	return glm::transpose(glm::mat4{
		2 / (right - left), 0,              0,             -(right + left) / (right - left),
		0,              2 / (top - bottom), 0,             -(top + bottom) / (top - bottom),
		0,              0,              -2 / (far - near), -(far + near) / (far - near),
		0,              0,              0,             1,
		});
}
