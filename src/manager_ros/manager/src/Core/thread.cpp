#include "thread.h"


Thread::Thread() {}

Thread::~Thread() 
{
	if (!this->isStoped()) {
		this->stop();
	}

	if (this->th.joinable()) {
		this->th.join();
	}
}

void Thread::start()
{
	std::thread thr(&Thread::run, this);
	this->th = std::move(thr);
}

void Thread::stop()
{
	this->stopState = true;
	if (this->th.joinable()) {
		this->th.join();
	}
}

void Thread::join()
{
	this->th.join();
}

void Thread::detach()
{
	this->th.detach();
}

void Thread::sleep(int sec)
{
	std::this_thread::sleep_for(std::chrono::seconds(sec));
}

void Thread::sleepMs(int msec)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

std::thread::id Thread::getId()
{
	return this->th.get_id();
}


bool Thread::isStoped()
{
	return this->stopState;
}