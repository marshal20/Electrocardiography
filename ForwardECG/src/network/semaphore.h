#pragma once
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>


enum WaitResult
{
	WAIT_RESULT_TIMEOUT = 1,
	WAIT_RESULT_NO_TIMEOUT,
	WAIT_RESULT_UNKNOWN
};

class Semaphore
{
public:
	Semaphore(int initial_count = 0, int max_count = 100);
	~Semaphore();

	void notify(int count = 1);
	WaitResult wait(unsigned int duration_ms = INFINITE);

private:
	HANDLE m_sem = NULL;
};
