/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include "people_tracking_filter/people_tracking_node.h"
#include "people_tracking_filter/tracker_particle.h"
#include "people_tracking_filter/tracker_kalman.h"
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"
#include <people_msgs/PositionMeasurement.h>


using namespace std;
using namespace tf;
using namespace BFL;
using namespace message_filters;

// Hardcoded constants. TODO: Investigate what each of these do and confirm their value is appropriate.
static const double       sequencer_delay            = 0.3; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

namespace estimation
{

// constructor
PeopleTrackingNode::PeopleTrackingNode(ros::NodeHandle nh) :
	nh_(nh),
	robot_state_(),
	tracker_counter_(0)
{
	// initialize
	meas_cloud_.points = vector<geometry_msgs::Point32>(1);
	meas_cloud_.points[0].x = 0;
	meas_cloud_.points[0].y = 0;
	meas_cloud_.points[0].z = 0;
	
	// get parameters
	ros::NodeHandle local_nh("~");
    local_nh.param("base_link_frame"      , base_link_frame_      , string(""));
	local_nh.param("fixed_frame"          , fixed_frame_          , string("default"));
	local_nh.param("freq"                 , freq_                 , 1.0);
	local_nh.param("start_distance_min"   , start_distance_min_   , 0.0);
	local_nh.param("reliability_threshold", reliability_threshold_, 1.0);
	local_nh.param("sys_sigma_pos_x"      , sys_sigma_.pos_[0]    , 0.0);
	local_nh.param("sys_sigma_pos_y"      , sys_sigma_.pos_[1]    , 0.0);
	local_nh.param("sys_sigma_pos_z"      , sys_sigma_.pos_[2]    , 0.0);
	local_nh.param("sys_sigma_vel_x"      , sys_sigma_.vel_[0]    , 0.0);
	local_nh.param("sys_sigma_vel_y"      , sys_sigma_.vel_[1]    , 0.0);
	local_nh.param("sys_sigma_vel_z"      , sys_sigma_.vel_[2]    , 0.0);
	local_nh.param("follow_one_person"    , follow_one_person_    , false);
	local_nh.param("acquisition_quality_threshold"    , acquisition_quality_threshold_ , .15);
	
	ROS_INFO_STREAM(boost::format("Acquisition quality threshold = %.2f") %acquisition_quality_threshold_);
	// advertise filter output
	people_filter_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_filter",10);
	
	// advertise visualization
	people_filter_vis_pub_ = nh_.advertise<sensor_msgs::PointCloud>("people_tracker_filter_visualization",10);
	people_tracker_vis_pub_ = nh_.advertise<sensor_msgs::PointCloud>("people_tracker_measurements_visualization",10);
	person_position_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("goal_with_covariance",1);
	                       
	ros::Duration(3.0).sleep();
	// register message sequencer
	people_meas_sub_ = nh_.subscribe("people_tracker_measurements", 10, &PeopleTrackingNode::callbackRcv, this);
}


// destructor
PeopleTrackingNode::~PeopleTrackingNode()
{
	// delete sequencer
	delete message_sequencer_;
	
	// delete all trackers
	for (list<Tracker*>::iterator it= trackers_.begin(); it!=trackers_.end(); it++)
		delete *it;
}


// callback for messages
void PeopleTrackingNode::callbackRcv(const people_msgs::PositionMeasurement::ConstPtr& message)
{
	ROS_DEBUG("(in callback) Tracking node got measurement for \"%s\" (%f,%f,%f)",message->object_id.c_str(), message->pos.x, message->pos.y, message->pos.z);
	
	// get measurement in fixed frame
	Stamped<tf::Vector3> meas_rel, meas;
	meas_rel.setData(tf::Vector3(message->pos.x, message->pos.y, message->pos.z));
	meas_rel.stamp_ = message->header.stamp;
	meas_rel.frame_id_ = message->header.frame_id;
	try
	{
		robot_state_.transformPoint(fixed_frame_, meas_rel, meas);
	}
	catch( tf::TransformException e )
	{
		ROS_WARN_STREAM("(in callback) Could not transform measurement (1), "<<e.what());
		return;
	}
	
	// get measurement covariance, which is really just used for variance along the diagonal and is hard coded for each type of measurement.
	SymmetricMatrix cov(3);
	for (unsigned int i = 0; i < 3; i++)
		for (unsigned int j = 0; j < 3; j++)
			cov(i + 1, j + 1) = message->covariance[3 * i + j];
			
	// ----- LOCKED ------
	boost::mutex::scoped_lock lock(filter_mutex_);
	
	// update tracker if matching tracker is found
	for (list<Tracker*>::iterator it = trackers_.begin(); it != trackers_.end(); it++) {
		if ((*it)->getName() == message->object_id) {
			ROS_INFO_STREAM("Updating tracker matching input measurement with object_id \"" << (*it)->getName() << "\"");
			(*it)->updatePrediction(message->header.stamp.toSec());
			(*it)->updateCorrection(meas, cov);
		}
	}
	
	// check if reliable message with no name should be a new tracker
    // Reliability values are hard coded in launch files, and face_detector outputs a high enough reliability.
	if (message->object_id == "" && message->reliability > reliability_threshold_) {

        // Find how close this message is to a current tracker. Don't allow a new tracker to start too close to a current tracker.
		double closest_tracker_dist = start_distance_min_;
		StatePosVel est;
		for (list<Tracker*>::iterator it = trackers_.begin(); it != trackers_.end(); it++) {
			(*it)->getEstimate(est);
			double dst = sqrt(pow(est.pos_[0] - meas[0], 2) + pow(est.pos_[1] - meas[1], 2));
			if (dst < closest_tracker_dist)
				closest_tracker_dist = dst;
		}
		
		// initialize a new tracker if:
        //    - The message given has been initialized, only true for face_detector
        //    - Only following one person and there is no current tracker
        //    - Not following one person and the closest tracker is far enough away
		if (message->initialization == 1 && ((!follow_one_person_ && (closest_tracker_dist >= start_distance_min_)) || (follow_one_person_ && trackers_.empty())))
		{
			tf::Point pt;
			tf::pointMsgToTF(message->pos, pt);
			tf::Stamped<tf::Point> loc(pt, message->header.stamp, message->header.frame_id);
			try {
				robot_state_.transformPoint(base_link_frame_, loc, loc);
			}
			catch( tf::TransformException e )
			{
				ROS_WARN_STREAM("(in callback) Could not transform measurement (2), "<<e.what());
				return;
			}
			
			float cur_dist;
			if ((cur_dist = pow(loc[0], 2.0) + pow(loc[1], 2.0)) < tracker_init_dist)
			{
				stringstream tracker_name;
				StatePosVel prior_sigma(tf::Vector3(sqrt(cov(1, 1)), sqrt(cov(2, 2)), sqrt(cov(3, 3))), tf::Vector3(0.0000001, 0.0000001, 0.0000001));
				tracker_name << "person " << tracker_counter_++;
				Tracker* new_tracker = new TrackerKalman(tracker_name.str(), sys_sigma_);
				new_tracker->initialize(meas, prior_sigma, message->header.stamp.toSec());
				trackers_.push_back(new_tracker);
				ROS_INFO("(in callback) Initialized new tracker %s", tracker_name.str().c_str());
			}
			else
				ROS_INFO("(in callback) New tracker found is %f away but must be less than %f away.", cur_dist , tracker_init_dist);
		}
	}
	lock.unlock();
	// ------ LOCKED ------
	
	// visualize measurement
	meas_cloud_.points[0].x = meas[0];
	meas_cloud_.points[0].y = meas[1];
	meas_cloud_.points[0].z = meas[2];
	meas_cloud_.header.frame_id = meas.frame_id_;
	people_tracker_vis_pub_.publish(meas_cloud_);
}


// filter loop
void PeopleTrackingNode::spin()
{
	ROS_INFO("Filter loop started.");
	
	ros::Rate loop_rate(freq_);
	ros::Time last_run = ros::Time::now();
	
    unsigned int count = 0;

	while (ros::ok())
	{
		while( ros::Time::now() - last_run < loop_rate.expectedCycleTime() ) {
			ros::spinOnce();
			ros::Duration(0.05).sleep();
		}
		last_run = ros::Time::now();
	
		// ------ LOCKED ------
		boost::mutex::scoped_lock lock(filter_mutex_);
		
		// visualization variables
		vector<geometry_msgs::Point32> filter_visualize(trackers_.size());
		vector<float> weights(trackers_.size());
		sensor_msgs::ChannelFloat32 channel;
		
        // Keep track of home many times since last output to ROS_INFO. Just make it output less, because otherwise it spams the console.
        count++;
        if ( count > 10 ){
            ROS_INFO("%d active trackers(s)", (int)trackers_.size());
            count = 0;
        }
		// loop over trackers, keep track of how many
        unsigned int i;
        list<Tracker*>::iterator it;
		for(i = 0, it = trackers_.begin(); it!=trackers_.end(); ++it, ++i)
		{
			// update prediction of each tracker up to the delayed time
			(*it)->updatePrediction(ros::Time::now().toSec() - sequencer_delay);
			
			// publish filter result, the estimated position
			people_msgs::PositionMeasurement est_pos;
			(*it)->getEstimate(est_pos);
			est_pos.header.frame_id = fixed_frame_;
			people_filter_pub_.publish(est_pos);
			
            // Output that info to DEBUG
			ROS_DEBUG_STREAM(boost::format("Publishing on people_tracker_filter, with %d trackers, (%.2f,%.2f)")
			  %trackers_.size() %est_pos.pos.x %est_pos.pos.y );
			
			if( follow_one_person_ ) // If only one tracker is allowed, publish on the 'goal position' topic as well.
			{
				geometry_msgs::PoseWithCovarianceStamped p;
				p.header = est_pos.header;
				p.pose.pose.position = est_pos.pos;
				p.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
				for( int j=0; j<36; j++ ){    // Since we are only tracking one person, covariance matrix doesn't really matter.
					p.pose.covariance[j] = 0; // The goal position topic isn't used for that information anyway.
				}
                // Publish this current esimate on this topic as well. Covariance is set to 0, a.k.a. "absolutely certain".
				person_position_pub_.publish(p);
			}
			
			// visualize filter result
			filter_visualize[i].x = est_pos.pos.x;
			filter_visualize[i].y = est_pos.pos.y;
			filter_visualize[i].z = est_pos.pos.z;
			weights[i] = *(float*)&(rgb[min(998, 999-max(1, (int)trunc( (*it)->getQuality()*999.0 )))]);
			
			// Remove trackers that have zero quality
			ROS_DEBUG("Quality of tracker %s = %f",(*it)->getName().c_str(), (*it)->getQuality());
			if ((*it)->getQuality() <= 0) {
				ROS_DEBUG("Removing tracker %s",(*it)->getName().c_str());
				delete *it;               // Delete the tracker. Don't want no memory leaks, y'all.
				it = trackers_.erase(it); // Erase returns the next item in the list. Decrement this iterator, because we only
                it--;                     // only the next item in the list and this iterator is about to be incremented.
			}
		}
		lock.unlock();
		// ------ LOCKED ------
		
		// visualize all trackers
		channel.name = "rgb";
		channel.values = weights;
		sensor_msgs::PointCloud  people_cloud;
		people_cloud.channels.push_back(channel);
		people_cloud.header.frame_id = fixed_frame_;
		people_cloud.points  = filter_visualize;
		people_filter_vis_pub_.publish(people_cloud);
		
		ros::spinOnce();
	}
} // end filter loop

}; // namespace

// Main
using namespace estimation;
int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv,"tracking_filter");
	ros::NodeHandle(nh);
	
	// create tracker node
	PeopleTrackingNode my_tracking_node(nh);
	
	// wait for filter initialization to finish then...
	my_tracking_node.spin();
	
	return 0;
}
