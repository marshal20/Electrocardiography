#include "camera.h"
#include "transform.h"
#include <math.h>


static glm::mat4 lookAtView(const glm::vec3& eye, const glm::vec3& target, const glm::vec3& up)
{
	glm::vec3 z_dir = glm::normalize(eye - target);
	glm::vec3 x_dir = glm::normalize(glm::cross(up, z_dir));
	glm::vec3 y_dir = glm::cross(z_dir, x_dir);
	glm::mat4 rot = glm::transpose(glm::mat4{
		x_dir.x, x_dir.y, x_dir.z, 0,
		y_dir.x, y_dir.y, y_dir.z, 0,
		z_dir.x, z_dir.y, z_dir.z, 0,
		0,       0,       0,       1
		});
	return rot * translate(-eye);
}

static glm::mat4 perspective(float fov, float aspect, float near, float far)
{
	float s = 1.0 / tan(fov / 2);
	float v1 = -(far+near) / (far-near);
	float v2 = -2*far*near / (far-near);
	return glm::transpose(glm::mat4{
		s/aspect, 0, 0,  0,
		0,        s, 0,  0,
		0,        0, v1, v2,
		0,        0, -1, 0
	});
}

glm::mat4 OrthographicCamera::calculateViewProjection() const
{
	float height = size;
	float width = size * aspect;
	glm::mat4 view = translate(-position);
	glm::mat4 proj = ortho(-width / 2, width / 2, height / 2, -height / 2, near, far);
	return proj * view;
}

glm::mat4 LookAtCamera::calculateViewProjection() const
{
	glm::mat4 view = lookAtView(eye, look_at, up);
	glm::mat4 proj = perspective(fov, aspect, near, far);
	return proj*view;
}

glm::mat4 FPSCamera::calculateViewProjection() const
{
	glm::mat4 view = rotateEulerXYZ(pitch, yaw, roll)*translate(-position);
	glm::mat4 proj = perspective(fov, aspect, near, far);
	return proj * view;
}
