#pragma once
#include <glm/glm.hpp>

// For rotation the angles are defined in the counter-clockwise
// direction looking into the corresponding axis.
// Rotation is Right-Hand.

glm::mat4 translate(const glm::vec3& p);
glm::mat4 rotateEulerXYZ(float theta_x, float theta_y, float theta_z);
glm::mat4 scale(const glm::vec3& s);
glm::mat4 ortho(float left, float right, float top, float bottom, float near, float far);
