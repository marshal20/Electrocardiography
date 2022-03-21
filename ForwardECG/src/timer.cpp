#include "timer.h"

using namespace std::chrono;

void Timer::start()
{
	m_start = high_resolution_clock::now();
}

Real Timer::elapsed_seconds() const
{
	const high_resolution_clock::time_point now = high_resolution_clock::now();
	const duration<Real> duration = now - m_start;
	return duration.count();
}
