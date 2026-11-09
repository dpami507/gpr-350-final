#include <chrono>

class Timer
{
public:
	Timer() = default;
	~Timer() = default;
	inline void start()
	{
		startTime = std::chrono::steady_clock::now();

		endTime = std::chrono::steady_clock::now();
	}
	inline void stop()
	{
		endTime = std::chrono::steady_clock::now();
	}

	// returns ms
	inline int getElapsedTime()
	{
		auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		return elapsedTime.count();
	}
private:
	std::chrono::steady_clock::time_point startTime;
	std::chrono::steady_clock::time_point endTime;
};