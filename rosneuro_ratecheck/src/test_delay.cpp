#include <ros/ros.h>
#include "rosneuro_ratecheck/DelayCheck.hpp"

int main(int argc, char** argv) {


	// ros initialization
	ros::init(argc, argv, "delaycheck");

	rosneuro::DelayCheck delaycheck;

	if(delaycheck.configure()== false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(256);
	while(ros::ok())
	{

		 if(delaycheck.CheckDelay() == true) {
			ROS_INFO_ONCE("Rate check started");
		 }

         ros::spinOnce();
		 r.sleep();
	}

	ros::shutdown();
	return 0;
}
