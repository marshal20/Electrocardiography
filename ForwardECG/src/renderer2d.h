#pragma once
#include <glm/glm.hpp>

// TODO: fix fill.

class glTexture;

struct Sprite
{
	glm::vec2 position;
	glm::vec2 size;
	glTexture* texture;
	glm::vec4 color;
};

class Renderer2D
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
	static void drawLine(const glm::vec2& p1, const glm::vec2& p2);
	static void drawTriangle(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3);
	static void drawRect(const glm::vec2& p1, const glm::vec2& p2);
	static void drawQuad(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3, const glm::vec2& p4);
	static void drawPolygon(const glm::vec2* points, int count);
	static void drawCircle(const glm::vec2& p, float r);
	static void drawEllipse(const glm::vec2& p, float rx, float ry);
	static void drawSprite(const Sprite& sprite);
	static void drawTexture(const glm::vec2& position, const glm::vec2& size, glTexture* texture);
};
