#include <ros/ros.h>
#include "rosneuro_ratecheck/RateCheck.hpp"

int main(int argc, char** argv) {


	// ros initialization
	ros::init(argc, argv, "ratecheck");

	rosneuro::RateCheck ratecheck;

	if(ratecheck.configure()== false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(256);
	while(ros::ok())
	{

		 if(ratecheck.CheckRateSingleTopic() == true) {
			ROS_INFO_ONCE("Rate check started");
		 }

     ros::spinOnce();
		 r.sleep();
	}

	ros::shutdown();
	return 0;
}
