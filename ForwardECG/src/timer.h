#pragma once
#include <chrono>
#include "math.h"

class Timer
{
private:
	std::chrono::high_resolution_clock::time_point m_start;

public:
	void start();
	Real elapsed_seconds() const;
};
