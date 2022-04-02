#include "imgui_my_types.h"


bool ImGui::InputReal(const char* label, Real* v, float step, float step_fast, const char* format, ImGuiInputTextFlags flags)
{
	float temp = *v;
	bool result = ImGui::InputFloat(label, &temp, step, step_fast, format, flags);
	*v = temp;
	return result;
}

bool ImGui::DragReal3(const char* label, Real v[3], float v_speed, float v_min, float v_max, const char* format, ImGuiSliderFlags flags)
{
	float temp[3] = { v[0], v[1], v[2] };
	bool result = ImGui::DragFloat3(label, temp, v_speed, v_min, v_max, format, flags);
	v[0] = temp[0];
	v[1] = temp[1];
	v[2] = temp[2];
	return result;
}

bool ImGui::DragVector3Eigen(const char* label, Eigen::Vector3<Real>& v, float v_speed, float v_min, float v_max, const char* format, ImGuiSliderFlags flags)
{
	float temp[3] = { v[0], v[1], v[2] };
	bool result = ImGui::DragFloat3(label, temp, v_speed, v_min, v_max, format, flags);
	v[0] = temp[0];
	v[1] = temp[1];
	v[2] = temp[2];
	return result;
}

