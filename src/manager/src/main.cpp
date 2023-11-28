#include <iostream>
#include <ros/ros.h>
#include "manager.h"

int main(int argc, char **argv)
{

	Manager manager(argc, argv);
	std::cout << "test" << std::endl;
	
	manager.init();
	manager.start();	
	manager.join();
	return 0;
}
