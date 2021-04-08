#ifndef RATE_CHECK_CPP
#define RATE_CHECK_CPP

#include "rosneuro_ratecheck/RateCheck.hpp"



namespace rosneuro {

RateCheck::RateCheck(void) : p_nh_("~") {


}

RateCheck::~RateCheck(void) {

}

bool RateCheck::configure(void) {

	// Setup the ROS parameters.
	ros::param::param("~sampling_freq", (int&) this->sampling_freq_, 512);
	ros::param::param("~n_samples", (int&) this->n_samples_, 32);
	ros::param::param("~max_error", (double&) this->max_error_, 1.0);

	// The user must give the name of the topic they want to check.
	if(ros::param::get("~sub_topic_data", this->sub_topic_data_) == false) {
		ROS_ERROR("Missing the name of the topic to check. 'sub_data' is a mandatory parameter");
		return false;
	}

	// Setup the subscriber and set the flag new_neuro_frame
	this->sub_data_ = this->p_nh_.subscribe(this->sub_topic_data_, 1000, &RateCheck::on_received_data, this);

	this->new_neuro_frame_ = false;
	this->expected_rate_ = static_cast<double>(this->sampling_freq_) / this->n_samples_;
	this->first_message_flag_ = true;

	//ROS_WARN("Expected rate: %f.", this->expected_rate_);

	return true;
}



void RateCheck::on_received_data(const topic_tools::ShapeShifter::ConstPtr& msg) {


	this->new_neuro_frame_ = true;

	if (this->first_message_flag_ == true) {
		this->time_new_ = std::chrono::high_resolution_clock::now();
	}
	else {
		this->time_old_ = this->time_new_;
		this->time_new_ = std::chrono::high_resolution_clock::now();
	}

}


bool RateCheck::CheckRateSingleTopic(void) {

	// Copy data in eigen structure
	if(this->new_neuro_frame_== false)
	{
		//ROS_WARN("Not available data");
		return false;
	}

	if(this->first_message_flag_ == true)
	{
		// Wait to have a second time reference to compute the frequency
		ROS_WARN("This is the first message");
		this->first_message_flag_ = false;
		this->new_neuro_frame_ = false;
		return false;
	}

	this->duration_ = this->time_new_ - this->time_old_;
	this->estimated_frequency_ = 1 / this->duration_.count();
	//ROS_WARN("Duration : %f", this->duration_.count());
	//ROS_WARN("Frequency : %f", this->estimated_frequency_);

	if ( fabs(this->expected_rate_ - this->estimated_frequency_) > this->max_error_) {
		ROS_WARN("WARNING: The computed frequency is not consistent with the expected frequency. Estimated rate: %f. Expected rate: %f", this->estimated_frequency_, this->expected_rate_);
	}
	else
	{
		//ROS_WARN("WARNING: All is good.");
	}


	this->new_neuro_frame_= false;


	return true;

}


}


#endif
