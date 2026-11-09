#include <iostream>

namespace engine
{
	inline float calculateSphereVolume(float radius)
	{
		return (4.f / 3.f) * 3.14159265f * (float)pow(radius, 3);
	}
	inline float clamp(float min, float max, float value)
	{
		return std::min(max, std::max(min, value));
	}
}