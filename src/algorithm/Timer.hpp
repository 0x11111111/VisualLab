#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <iostream>
#include <string>

class Timer
{
public:
	Timer(std::string name)
	{
		m_name = name;
		m_start = std::chrono::high_resolution_clock::now();
	}
	~Timer()
	{
		Stop();
	}
private:
	void Stop()
	{
		m_end = std::chrono::high_resolution_clock::now();
		long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_start).time_since_epoch().count();
		long long end = std::chrono::time_point_cast<std::chrono::microseconds>(m_end).time_since_epoch().count();
		long long duration = end - start;
		double ms = duration * 0.001;
		std::cout << "Timer: " << m_name << " took " << duration << "us (" << ms << "ms)" << std::endl;
	}
	std::chrono::time_point<std::chrono::high_resolution_clock> m_start;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_end;
	std::string m_name;
};

#endif // TIMER_H