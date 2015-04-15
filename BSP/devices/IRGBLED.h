#pragma once
namespace devices
{
	class IRGBLED
	{
	public:
		virtual int write(float R, float G, float B) = 0;
	};
}
