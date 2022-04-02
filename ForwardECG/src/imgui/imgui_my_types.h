#pragma once
#include "imgui.h"
#include "../math.h"
#include <Eigen/Dense>

namespace ImGui
{
	bool InputReal(const char* label, Real* v, float step = 0.0f, float step_fast = 0.0f, const char* format = "%0.3f", ImGuiInputTextFlags flags = 0);
	bool DragReal3(const char* label, Real v[3], float v_speed = 1.0f, float v_min = 0.0, float v_max = 0.0, const char* format = "%0.3f", ImGuiSliderFlags flags = 0);
	bool DragVector3Eigen(const char* label, Eigen::Vector3<Real>& v, float v_speed = 1.0f, float v_min = 0.0, float v_max = 0.0, const char* format = "%0.3f", ImGuiSliderFlags flags = 0);
}
