#include <iostream>
#include <ros/ros.h>
#include "manager.h"
#include "log.h"

int main(int argc, char **argv)
{
	initLogger(INFO);
	Manager manager(argc, argv);
	std::cout << "test" << std::endl;
	
	manager.init();
	manager.start();	
	manager.join();
	return 0;
}
