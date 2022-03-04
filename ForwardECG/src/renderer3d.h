#pragma once
#include <glm/glm.hpp>

// TODO: fix fill.

class glTexture;

class Renderer3D
{
public:
	struct Style
	{
		// Stroke variables.
		bool stroke;
		float stroke_width;
		glm::vec4 stroke_color;
		// Fill variables.
		bool fill;
		glm::vec4 fill_color;
		// Functions.
		inline Style(bool s, float sw, const glm::vec4& sc, bool f, const glm::vec4& fc) :
			stroke(s), stroke_width(sw), stroke_color(sc), fill(f), fill_color(fc) {}
		inline static Style Stroke(float sw, const glm::vec4& sc) {
			return { true, sw, sc, false, glm::vec4() };
		}
		inline static Style Fill(const glm::vec4& fc) {
			return { false, 1, glm::vec4(), true, fc };
		}
		inline static Style StrokeFill(float sw, const glm::vec4& sc, const glm::vec4& fc) {
			return { true, sw, sc, true, fc };
		}
	};

public:
	static void init();
	static void cleanup();

	static void setProjection(const glm::mat4& proj);
	static void setStyle(const Style& style);

	static void fill(const glm::vec4& color);
	static void drawLine(const glm::vec3& p1, const glm::vec3& p2);
	static void drawPolygon(const glm::vec3* points, int count);
	static void drawPoint(const glm::vec3& point, const glm::vec4& color = { 0, 0, 0, 1 }, float size = 2.0);
};
