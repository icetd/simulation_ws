#ifndef THREAD_H
#define THREAD_H

#include <thread>
#include <chrono>
#include <atomic>

class Thread
{
public:
	Thread();
	virtual ~Thread();
	
	std::thread::id getId();

	void start();
	void detach();
	void stop();
	void join();
	void sleepMs(int msec);
	bool isStoped();

	virtual void run() = 0;

private:
	std::atomic<bool> stopState;
	std::thread th;
};

#endif