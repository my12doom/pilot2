#pragma once

namespace HAL
{
	class IFIFO
	{
		public:
		virtual int open(const char* path)=0;
		virtual int close()=0;
		virtual int create(const char* path,int mode)=0;
		virtual int write(void *data,int count)=0;
		virtual int read(void *data,int count)=0;
	};

}
