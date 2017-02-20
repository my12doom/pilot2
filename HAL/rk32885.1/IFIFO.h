#pragma once

namespace androidUAV
{
	class IFIFO
	{
		public:
		virtual int write(void *data,int count)=0;
		virtual int read(void *data,int count)=0;
	};

}
