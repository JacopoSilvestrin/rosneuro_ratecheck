#ifndef DELAY_CHECK_CPP
#define DELAY_CHECK_CPP

#include "rosneuro_ratecheck/DelayCheck.hpp"

QNode::QNode(int seq, double time)
{
    this->seq = seq;
    this->time = time;
    this->next = NULL;
}

Queue::Queue()
{
    this->front = this->rear = NULL;
}

void Queue::enQueue(int seq, double time)
{

    // Create a new LL node
    QNode* temp = new QNode(seq, time);

    // If queue is empty, then
    // new node is front and rear both
    if (this->rear == NULL) {
        this->front = this->rear = temp;
        return;
    }

    // Add the new node at
    // the end of queue and change rear
    this->rear->next = temp;
    this->rear = temp;
}

// Function to remove
// a key from given queue q
QNode* Queue::deQueue()
{
    // If queue is empty, return NULL.
    if (this->front == NULL)
        return NULL;

    // Store previous front and
    // move front one node ahead
    QNode* temp = this->front;
    this->front = this->front->next;

    // If front becomes NULL, then
    // change rear also as NULL
    if (this->front == NULL)
        this->rear = NULL;

    return temp;
}

int Queue::getSeq()
{
  // If queue is empty, return NULL.
  if (this->front == NULL)
      return -1;

  return this->front->seq;

}

double Queue::getTime()
{
  // If queue is empty, return NULL.
  if (this->front == NULL)
      return -1;

  return this->front->time;
}




namespace rosneuro {

DelayCheck::DelayCheck(void) : p_nh_("~") {


}

DelayCheck::~DelayCheck(void) {

}

bool DelayCheck::configure(void) {

	// Setup the ROS parameters.

	ros::param::param("~expected_delay", this->expected_delay_, 0.05);

	// The user must give the name of the topics they want to check.
	if(ros::param::get("~first_sub_topic", this->first_sub_topic_) == false) {
		ROS_ERROR("Missing the name of the first topic to check. 'first_sub_topic_' is a mandatory parameter");
		return false;
	}

  if(ros::param::get("~second_sub_topic", this->second_sub_topic_) == false) {
		ROS_ERROR("Missing the name of the second topic to check. 'second_sub_topic_' is a mandatory parameter");
		return false;
	}

	// Setup the subscriber and set the flag new_neuro_frame

  if(this->first_sub_topic_.compare("/neurodata") == 0) {
    this->first_sub_data_ = this->p_nh_.subscribe(this->first_sub_topic_, 1000, &DelayCheck::on_received_data_first<rosneuro_msgs::NeuroFrame>, this);
  }
  else if (this->first_sub_topic_.compare("/neuroprediction") == 0) {
    this->first_sub_data_ = this->p_nh_.subscribe(this->first_sub_topic_, 1000, &DelayCheck::on_received_data_first<rosneuro_msgs::NeuroOutput>, this);
  }
  else {
    ROS_WARN("The topic %s is not currently supported.", this->first_sub_topic_.c_str());
    return false;
  }

  if(this->second_sub_topic_.compare("/bus") == 0) {
    this->second_sub_data_ = this->p_nh_.subscribe(this->second_sub_topic_, 1000, &DelayCheck::on_received_data_second<rosneuro_msgs::NeuroEvent>, this);
  }
  else if (this->second_sub_topic_.compare("/neuroprediction") == 0) {
    this->second_sub_data_ = this->p_nh_.subscribe(this->second_sub_topic_, 1000, &DelayCheck::on_received_data_second<rosneuro_msgs::NeuroOutput>, this);
  }
  else {
    ROS_WARN("The topic %s is not currently supported.", this->second_sub_topic_.c_str());
    return false;
  }


	this->new_neuro_frame_ = false;
	return true;
}


template <class T>
void DelayCheck::on_received_data_first(const typename T::ConstPtr& msg) {

  this->queue_.enQueue(msg->header.seq, msg->header.stamp.toSec());
  ROS_WARN("Messo in coda. Seq: %d. Time: %f",msg->header.seq, msg->header.stamp.toSec());

}

template <class T>
void DelayCheck::on_received_data_second(const typename T::ConstPtr& msg) {

	this->new_neuro_frame_ = true;
  this->seq_ = msg->neuroheader.frame_rel_seq;
  this->time_ =  msg->header.stamp.toSec();
  ROS_WARN("Arrivato messaggio dal secondo topic. Seq: %d. Time: %f.",msg->neuroheader.frame_rel_seq, msg->header.stamp.toSec());

}


bool DelayCheck::CheckDelay(void) {

	// Copy data in eigen structure
	if(this->new_neuro_frame_== false)
	{
		//ROS_WARN("Not available data to classify");
		return false;
	}

  this->first_queue_element_seq_ = this->queue_.getSeq();
  this->first_queue_element_time_ = this->queue_.getTime();


  // if the queue is empty something went wrong and the function returns false
  if(this->first_queue_element_seq_ == -1)
  {
    ROS_WARN("The queue is empty.");
    this->new_neuro_frame_ = false;
    return false;
  }

  // if the sequence of the second topic message is lower than the sequence of the
  // first one we even things up
  if(this->seq_ < this->first_queue_element_seq_)
  {
    this->new_neuro_frame_ = false;
    return false;
  }

  // We are sure that the sequence number of our second topic msg is greater or equal to the one of the first element of the queue
  // We extract the first element of the queue
  this->node_ = this->queue_.deQueue();

  // if it is higher we dig into the queue until we find the right node
  if(this->seq_ > this->node_->seq)
  {
    while(this->seq_ != this->node_->seq)
    {
      delete (this->node_);
      this->node_ = this->queue_.deQueue();
    }
  }

  //compute delay
  this->estimated_delay_ = this->time_ - this->node_->time;
  //ROS_WARN("WARNING: Time secondo topic: %f. Time primo topic: %f.", this->time_, this->node_->time);


	if ( this->estimated_delay_ > this->expected_delay_) {
		ROS_WARN("WARNING: The computed delay exceeds the maximum tolerated delay. Expected delay: %f. Estimated delay: %f.", this->expected_delay_, this->estimated_delay_);
	}


  delete (this->node_);
	this->new_neuro_frame_= false;


	return true;

}


}

//template void DelayCheck::on_received_data<rosneuro_msgs::NeuroFrame>(rosneuro_msgs::NeuroFrame);


#endif
