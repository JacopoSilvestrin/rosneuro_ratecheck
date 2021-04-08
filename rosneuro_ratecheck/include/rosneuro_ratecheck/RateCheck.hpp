#ifndef RATE_CHECK_HPP
#define RATE_CHECK_HPP

#include <algorithm>
#include <iostream>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <topic_tools/shape_shifter.h>


namespace rosneuro {

class RateCheck {
	public:
		RateCheck(void);
		virtual ~RateCheck(void);

		bool configure(void);


		bool CheckRateSingleTopic(void);

       	private:
		void on_received_data(const topic_tools::ShapeShifter::ConstPtr& msg);


	private:
		ros::NodeHandle		   nh_;
		ros::NodeHandle		   p_nh_;
		ros::Subscriber		   sub_data_;
		std::string                sub_topic_data_;
		bool 			   new_neuro_frame_;

		unsigned int 	sampling_freq_;
		unsigned int 	n_samples_;
		double expected_rate_;
		double max_error_;
		bool first_message_flag_;
		std::chrono::time_point<std::chrono::high_resolution_clock> time_old_;
		std::chrono::time_point<std::chrono::high_resolution_clock> time_new_;
		std::chrono::duration<double> duration_;
		double estimated_frequency_;

};

}


#endif
