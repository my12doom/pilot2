#pragma once

namespace HAL
{
	class IFIFO
	{
		public:
		virtual int write(void *data,int count)=0;
		virtual int read(void *data,int count)=0;
		
		// return false if any error/warning
		virtual bool healthy() = 0;
	};

}
