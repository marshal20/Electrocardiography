#pragma once
#include <glm/glm.hpp>


// Coordinates system:
// Z+ forward, Y+ up
//                  
//      ^           
//    Y |           
//      |           
//      |           
//      +------->   
//     /       X    
//    /             
//   Z              
//                  
//                  

class Camera
{
public:
	virtual glm::mat4 calculateViewProjection() const = 0;
};

class OrthographicCamera : public Camera
{
public:
	virtual glm::mat4 calculateViewProjection() const override;
public:
	glm::vec3 position;
	float size;
	float aspect;
	float near, far;
};

class LookAtCamera : public Camera
{
public:
	LookAtCamera() = default;
	LookAtCamera(const glm::vec3& eye_, const glm::vec3& look_at_, const glm::vec3& up_, float fov_, float aspect_, float near_, float far_);
	virtual glm::mat4 calculateViewProjection() const override;

public:
	glm::vec3 eye;
	glm::vec3 look_at;
	glm::vec3 up;
	float fov;
	float aspect;
	float near, far;
};

class FPSCamera : public Camera
{
public:
	virtual glm::mat4 calculateViewProjection() const override;
public:
	glm::vec3 position;
	float pitch, yaw, roll;
	float fov;
	float aspect;
	float near, far;
};
