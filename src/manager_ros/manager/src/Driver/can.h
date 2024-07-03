#ifndef CAN_H
#define CAN_H

#include <functional>
#include <string>
#include <linux/can.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can/raw.h>
#include <vector>
#include "thread.h"


class Can : public Thread
{
public:
    Can(char *deviceName);
    ~Can();
	
	int Init();
	int Destroy();

	void StartAutoRead();
	void StopAutoRead();
	virtual void run() override;
	void SetOnCanReceiveDataCallback(std::function<void (struct can_frame &&)> callback);
	
	int Transmit(struct can_frame *frame);

private:
    int m_sd;
	std::string m_deviceName;
	struct ifreq ifr;
	struct sockaddr_can addr;
	bool isAutoRead;
	std::function<void (struct can_frame &&)> onCanReceiveDataCallback;
};

#endif
