#pragma once

#include <stdint.h>
#include "IInterrupt.h"

namespace HAL 
{
	typedef void (*timer_callback)(void *user_data);

	class ITimer
	{
	public:
		virtual void set_period(uint32_t period) = 0;				// micro-second
		virtual void set_callback(timer_callback cb, void *user_data = 0) = 0;
		virtual void restart(){}
		virtual void enable_cb(){}
		virtual void disable_cb(){}
		virtual void set_priority(int preemption_priority, int sub_priority = 0){}
	};
	
	class InterruptTimer: public ITimer
	{
	public:
		InterruptTimer(IInterrupt *_int)
		{
			this->_int = _int;
			_int->set_callback(entry, this);
		}
		~InterruptTimer(){}
		virtual void set_period(uint32_t period){}
		virtual void set_callback(timer_callback cb, void *user_data = 0)
		{
			_cb = cb;
			_user_data = _user_data;
		}
	private:
		static void entry(void *parameter, int flags)
		{
			InterruptTimer *_this = (InterruptTimer*) parameter;
			if (_this->_cb)
				_this->_cb(_this->_user_data);
		}
		
		IInterrupt *_int;
		timer_callback _cb;
		void *_user_data;
	};
}
