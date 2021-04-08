#ifndef DELAY_CHECK_HPP
#define DELAY_CHECK_HPP

#include <algorithm>
#include <iostream>
#include <string>
#include <chrono>
#include <cmath>
#include <bits/stdc++.h>

#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_msgs/NeuroEvent.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <topic_tools/shape_shifter.h>

struct QNode {
    int seq;
		double time;
    QNode* next;
    QNode(int seq, double time);
};

struct Queue {
    QNode *front, *rear;

    Queue();

    void enQueue(int seq, double time);

    QNode* deQueue();

    int getSeq();

    double getTime();
};


namespace rosneuro {


class DelayCheck {
	public:
		DelayCheck(void);
		virtual ~DelayCheck(void);

		bool configure(void);


		bool CheckDelay(void);

       	private:

    template <class T>
		void on_received_data_first(const typename T::ConstPtr& msg);

    template <class T>
		void on_received_data_second(const typename T::ConstPtr& msg);


	private:
		ros::NodeHandle		   nh_;
		ros::NodeHandle		   p_nh_;
		ros::Subscriber		   first_sub_data_;
		ros::Subscriber 			 second_sub_data_;
		std::string                first_sub_topic_;
		std::string 							 second_sub_topic_;
		bool 			   new_neuro_frame_;


		double expected_delay_;
		QNode* node_;
		Queue queue_;
		double estimated_delay_;
		int seq_;
		double time_;
    int first_queue_element_seq_;
    double first_queue_element_time_;

};

}


#endif
