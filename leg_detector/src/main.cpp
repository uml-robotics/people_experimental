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

#include "leg_detector/leg_detector.h"

// Non-standard ROS includes
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"

// Message includes
#include "std_msgs/Header.h" // TODO: Necessary?

// C++ includes
#include <algorithm> // TODO: Necessary?


// Hardcoded constants... Be careful. Note: s for seconds, m for meters
static const double no_observation_timeout_s = 0.5; // How long a leg can last without a measurement before it is deleted.
static const double max_second_leg_age_s     = 2.0; // Used to disregard old legs without ID's, because they are unlikely to be our tracker.
static const double max_track_jump_m         = 1.0; // Max distance a leg can be from an input tracker and still update it.
static const double max_meas_jump_m          = 0.75; // Max distance a measurement can be away from a leg and still be used to update it.
static const double leg_pair_separation_m    = 1.0; // Max distance that should be between two legs associated with the same tracker.

// Input ROS params
static std::string fixed_frame_       = "/odom";
static std::string laser_scan_topic_  = "/scan";
static double max_pub_rate_s_    = 1.0;
static double connected_thresh_  = 0.06;

// Input from command line or launch file commands
int g_argc;
char** g_argv;

// Main
int main(int argc, char **argv)
{
	ros::init( argc, argv, "leg_detector" );
	g_argc = argc;
	g_argv = argv;

    // ROS parameters
    ros::NodeHandle pnh( "~" );
    pnh.param("odom_frame", fixed_frame_, std::string("") );
    pnh.param("laser_scan_topic", laser_scan_topic_, std::string("") );
    pnh.param("max_pub_rate"    , max_pub_rate_s_, 1.0);
    pnh.param("connected_thresh", connected_thresh_, .06);

	ros::NodeHandle nh;

	LegDetector ld( nh, fixed_frame_, laser_scan_topic_, max_pub_rate_s_, connected_thresh_, g_argc, g_argv );

	ros::spin();
	
	return 0;
}
