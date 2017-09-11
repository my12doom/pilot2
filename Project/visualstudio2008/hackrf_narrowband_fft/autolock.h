#pragma once

#include <Windows.h>


class _critical_section
{
public:
	_critical_section()
	{
		InitializeCriticalSection(&cs);
	}
	~_critical_section()
	{
		DeleteCriticalSection(&cs);
	}

	void enter()
	{
		EnterCriticalSection(&cs);
	}
	void leave()
	{
		LeaveCriticalSection(&cs);
	}

private:
	CRITICAL_SECTION cs;
};

class _autolock
{
public:
	_autolock(_critical_section *cs)
	{
		_cs = cs;
		cs->enter();
	}
	~_autolock()
	{
		_cs->leave();
	}

private:
	_critical_section *_cs;
};