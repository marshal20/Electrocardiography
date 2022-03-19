#include "semaphore.h"


Semaphore::Semaphore(int initial_count, int max_count)
{
	m_sem = CreateSemaphore(NULL, initial_count, max_count, NULL);
}

Semaphore::~Semaphore()
{
	CloseHandle(m_sem);
}

void Semaphore::notify(int count)
{
	ReleaseSemaphore(m_sem, count, NULL);
}

WaitResult Semaphore::wait(unsigned int duration_ms)
{
	DWORD result = WaitForSingleObject(m_sem, duration_ms);

	if (result == WAIT_OBJECT_0)
	{
		return WAIT_RESULT_NO_TIMEOUT;
	}
	else if (result == WAIT_TIMEOUT)
	{
		return WAIT_RESULT_TIMEOUT;
	}

	return WAIT_RESULT_UNKNOWN;
}

