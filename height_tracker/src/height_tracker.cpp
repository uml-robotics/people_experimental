/**********************************************************************
 *
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

/*      Author: Ethan Dreyfuss        */

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PointStamped.h>

#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "people_msgs/PositionMeasurement.h"
//#include "face_detector/utils.h"
#include <opencv/highgui.h>
#include <boost/format.hpp>

#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

using namespace std;

namespace people {

class HeightTracker {
private:

	// Node handle
	ros::NodeHandle &nh_;
	ros::NodeHandle private_nh_;
	
	string openni_namespace_;
	
	sensor_msgs::CvBridge vis_flat_bridge_;
	
	IplImage *disparity;
	IplImage *windowBG; //Current disparity image to use as background
	IplImage *windowImg; //Current image to display (=windowBG with rendered boxes, etc)
	IplImage *left_image;
	CvRect window_;
	CvRect top_down_window_;
	
	boost::mutex window_mutex_;
	boost::mutex image_mutex_;
	boost::mutex mouse_mutex_;
	
	bool tracking_;
	
	string window_name_;
	string dimg_window_name_;
	string limg_window_name_;
	string prob_window_name_;
	string flattened_window_name_;
	
	ros::Rate max_pub_rate_;
	
	bool mouse_down_;
	CvRect selection_rect_;
	
	double median_;
	bool new_track_from_measurement_;
	
	double hack_height_;
	
	string objID_;
	
	tf::TransformListener tf_client_;
	tf::TransformBroadcaster tf_server_;
	
	tf::MessageFilter<sensor_msgs::PointCloud> *filter_;
	
	message_filters::Subscriber<people_msgs::PositionMeasurement> *people_sub_;
	tf::MessageFilter<people_msgs::PositionMeasurement> *people_pos_notifier_;
	
	ros::Subscriber rebroadcast_sub_;
	ros::Subscriber cloud_only_sub_;
	
	ros::Publisher pos_pub_;
//	ros::Publisher head_target_pub_;

	ros::Publisher marker_pub_;
	image_transport::Publisher flat_pub_;
	
	//fixed frame to transform incoming stereo point clouds into
	string fixed_frame_;
	
	bool do_display_;
	bool do_cloud_only_;
	
	//Clipping constants for the stereo point cloud
	//The defaults are set to get a reasonable pretty far in front
	//of the robot and a little bit to either side, and almost but
	//not quite up to the ceiling
	double minLX, maxLX;
	double minLY, maxLY;
	double minLZ, maxLZ;
	
	//Width and height in pixels for the image plane which the stereo point cloud
	//gets projected down into
	unsigned int W, H;
	
public:

	/**
	 * Initialize a HeightTracker node which is responsible for listening to a stereo point cloud
	 * and tracking people using meanshift on the heightmap resulting from projecting the
	 * point cloud down to the ground plane.
	 */
	HeightTracker(ros::NodeHandle &nh) :
		nh_(nh),
		private_nh_("~"),
		max_pub_rate_(0)
	{
		//Set Logger level to DEBUG
		//log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
		//logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
		//ros::console::notifyLoggerLevelsChanged();
		
		//Default objectID
		objID_ = "NOT_AN_ID";
		
		//State variables, tracking_ is true when the tracker is running, new_track_from_measurement_ means that a
		//measurement has been received from the person tracking filter and it was sufficiently far away from the
		//current tracking window location to force a restart
		tracking_ = false;
		mouse_down_ = false;
		new_track_from_measurement_ = false;
		
		windowImg = 0;
		windowBG = 0;
		
		//TODO: render this obsolete
		private_nh_.param("hack_height", hack_height_, 170.0);
		
		//Stereo namespace to use and fixed frame to transform cloud (robot-relative frame is probably best)
		private_nh_.param("openni_namespace", openni_namespace_, string("camera"));
		private_nh_.param("fixed_frame", fixed_frame_, string("base_link"));
		ROS_INFO_STREAM("OpenNI namespace=\""<<openni_namespace_<<"\", fixed frame=\""<<fixed_frame_<<"\"");
		
		//Should we output debugging visualization images?
		private_nh_.param("do_display", do_display_, false);
		ROS_INFO_STREAM("do_display = " << (do_display_==true ? "true" : "false"));
		
		//Clipping plane constants for the stereo point cloud
		private_nh_.param("minimum_x", minLX, 0.0);
		private_nh_.param("maximum_x", maxLX, 10.0);
		private_nh_.param("minimum_y", minLY,-3.0);
		private_nh_.param("maximum_y", maxLY, 3.0);
		private_nh_.param("minimum_z", minLZ, 0.0);
		private_nh_.param("maximum_z", maxLZ, 2.5);
		
		double max_pub_rate_temp;
		private_nh_.param("max_pub_rate", max_pub_rate_temp, 2.0);
		max_pub_rate_ = ros::Rate(max_pub_rate_temp);
		
		//Width and height for the image to project the point cloud down into
		//More resolution means closer to true point-based meanshift but less efficient
		//Also, some amount of smoothing may not be such a bad thing
		int tmp_W, tmp_H;
		private_nh_.param("flattened_width" , tmp_W, 640);
		private_nh_.param("flattened_height", tmp_H, 480);
		W = (unsigned int)tmp_W;
		H = (unsigned int)tmp_H;
		
		ROS_INFO_STREAM("Projecting cloud down to ("<<W<<","<<H<<")");
		
		//Advertise the measurements in the local namespace
		pos_pub_ = private_nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_measurements",1);
		//head_target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/head_controller/point_head",1);
		
		//Visualization support
		marker_pub_ = private_nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
		image_transport::ImageTransport it(private_nh_);
		flat_pub_ = it.advertise("flattened_image", 1);
		
		window_name_ = "Window";
		limg_window_name_ = "limg";
		dimg_window_name_ = "dimg";
		prob_window_name_ = "prob";
		flattened_window_name_ = "flattened";
		/*cvNamedWindow(window_name_          .c_str(),CV_WINDOW_AUTOSIZE);
		/*cvNamedWindow(limg_window_name_     .c_str(),CV_WINDOW_AUTOSIZE);
		cvNamedWindow(dimg_window_name_     .c_str(),CV_WINDOW_AUTOSIZE);
		cvNamedWindow(prob_window_name_     .c_str(),CV_WINDOW_AUTOSIZE);
		cvNamedWindow(flattened_window_name_.c_str(),CV_WINDOW_AUTOSIZE);*/
		
		//Subscribe to stereo cloud
		cloud_only_sub_ = nh_.subscribe(openni_namespace_ + "/depth_registered/points", 1, &HeightTracker::cloudCB, this);
		
		//Listen to position measurements back from the overall people tracking filter, delaying the messages
		//until they can be transformed into the fixed_frame_ (e.g. /base_footprint)
		people_sub_ = new message_filters::Subscriber<people_msgs::PositionMeasurement>(nh_, "people_tracker_filter", 1);
		people_pos_notifier_ = new tf::MessageFilter<people_msgs::PositionMeasurement>(*people_sub_, tf_client_, fixed_frame_, 10);
		people_pos_notifier_->registerCallback(boost::bind(&HeightTracker::peopleCallback, this, _1));
		
		//Initialize tracking default window to legal window in top-left corner
		window_.x = 0; window_.y = 0;
		window_.width = 1; window_.height = 1;
	}
	
	/**
	*  Callback for the stereo point cloud, this function also does the tracking
	*/
//  void cloudCB(const sensor_msgs::PointCloud::ConstPtr &cloud_ptr)
	void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &cloud2_ptr)
	{
		static ros::Time last_cb_time(0);
		if( ros::Time::now() - last_cb_time < max_pub_rate_.expectedCycleTime() ) {
			return;
		}
		last_cb_time = ros::Time::now();
		max_pub_rate_.reset();
		
		ROS_INFO("=============================================");
		ROS_INFO("Cloud");
		
		
		//ROS_INFO_STREAM("Got cloud in frame \""<<raw_cloud.header.frame_id<<"\"");
		sensor_msgs::PointCloud raw_cloud;
		if( !sensor_msgs::convertPointCloud2ToPointCloud(*cloud2_ptr, raw_cloud) ) {
			ROS_ERROR("Could not convert point cloud!");
			return;
		}
		
		sensor_msgs::PointCloud cloud;
		try {
			tf_client_.transformPointCloud(fixed_frame_, raw_cloud, cloud);
		} catch( tf::TransformException e) {
			ROS_ERROR_STREAM("TF exception despite using message notifier: "<<e.what());
			return;
		}
		
		if(cloud.points.size() == 0)
		{
			ROS_WARN("Received empty point cloud!");
			return;
		}
		
		//Go through every point and drop those which are not within the clipping region
		//Also incidentally compute the min/max in each dimension after clipping, which
		//can be useful debugging information
		double minX, maxX, minY, maxY, minZ, maxZ;
		minX =  9999; maxX = -9999;
		minY =  9999; maxY = -9999;
		minZ =  9999; maxZ = -9999;
		
		static vector<geometry_msgs::Point32> pts(100000);
		pts.clear();
		for(unsigned int i=1; i<cloud.points.size(); i++)
		{
			geometry_msgs::Point32 &p = cloud.points[i];
			
			if(p.z < maxLZ && p.z > minLZ && p.x > minLX && p.x < maxLX && p.y > minLY && p.y < maxLY)
			{
				pts.push_back(p);
				
				if(p.x < minX) minX = p.x;
				if(p.x > maxX) maxX = p.x;
				if(p.y < minY) minY = p.y;
				if(p.y > maxY) maxY = p.y;
				if(p.z < minZ) minZ = p.z;
				if(p.z > maxZ) maxZ = p.z;
			}
		}
		
		ROS_INFO_STREAM(boost::format("x:[%.2f,%.2f] y:[%.2f,%.2f] z:[%.2f,%.2f] Dropped %d/%d")
		                % minX % maxX % minY % maxY % minZ % maxZ % (cloud.points.size()-pts.size()) % cloud.points.size());
		                
		//Project the remaining points down onto the ground plane (or whatever the x/y plane in fixed_frame is)
		IplImage *flattened = cvCreateImage(cvSize(W,H), 8, 1);
		for(unsigned int i=0; i<W*H; i++)
			flattened->imageData[i] = (unsigned char)0;
		for(unsigned int i=0; i<pts.size(); i++)
		{
			geometry_msgs::Point32 &p = pts[i];
			unsigned int x = (int)((W-1)*((p.x - minLX) / (maxLX-minLX)));
			unsigned int y = (int)((H-1)*((p.y - minLY) / (maxLY-minLY)));
			y = (H-1) - y;
			unsigned char val = (int)(255.0*((p.z - minLZ) / (maxLZ-minLZ)));
			unsigned char cur = ((unsigned char *)(flattened->imageData + y*flattened->widthStep))[x];
			if(val > cur) {
				((unsigned char *)(flattened->imageData + y*flattened->widthStep))[x] = val;
			}
		}
		
		//if(do_display_) {
		//  cvShowImage(flattened_window_name_.c_str(), flattened);
		//}
		
		//If there is a new track, the height to use must be computed TODO: this should be removable soon
		if(new_track_from_measurement_)
		{
			//median_ = computeMedian(flattened, top_down_window_.y, top_down_window_.y+top_down_window_.height, top_down_window_.x, top_down_window_.x+top_down_window_.width);
			median_ = hack_height_; //TODO: remove this hack
			new_track_from_measurement_ = false;
		}
		
		//Perform the actual mean-shift tracking if we are currently in tracking mode
		if(tracking_)
		{
			IplImage *probIm = cvCreateImage(cvSize(flattened->width, flattened->height), IPL_DEPTH_8U, 1);;
			
			for(int i=0; i<flattened->width*flattened->height; i++)
			{
				unsigned char val = ((unsigned char *)(flattened->imageData))[i];
				unsigned char med = (unsigned char)median_;
				((unsigned char *)(probIm->imageData))[i] = 255 - (med < val ? val-med : med-val);
			}
			
			/*if(do_display_) {
			  cvShowImage(prob_window_name_.c_str(), probIm);
			}*/
			
			CvTermCriteria crit = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0); //Use either movement of only one pixel or 10 iterations as the termination criteria
			CvConnectedComp comp;
			
			ros::Time t1 = ros::Time::now();
			cvMeanShift(probIm, top_down_window_, crit, &comp);
			ROS_INFO_STREAM(boost::format("MeanShift took %.4f seconds") %(ros::Time::now()-t1).toSec() );
			
			
			top_down_window_ = comp.rect;
			
			cvReleaseImage(&probIm);
		}
		
		//Declare the annotated image which will be output TODO: make this output with image publisher and compute only
		//if someone is listening to it.
		IplImage *annotatedFlattened = cvCreateImage(cvSize(flattened->width, flattened->height), IPL_DEPTH_8U, 3);
		cvCvtColor(flattened, annotatedFlattened, CV_GRAY2BGR);
		
		//Compute and send the people position measurement to the filter
		if(tracking_)
		{
			cvRectangle(annotatedFlattened, cvPoint(top_down_window_.x, top_down_window_.y), cvPoint(top_down_window_.x+top_down_window_.width, top_down_window_.y+top_down_window_.height), cvScalar(0,0,200), 1);
			
			sensor_msgs::Image flat_img_msg;
			vis_flat_bridge_.fromIpltoRosImage(annotatedFlattened, flat_img_msg);
			flat_pub_.publish(flat_img_msg);
			
			people_msgs::PositionMeasurement pos;
			pos.header = cloud.header;
			pos.name = "head_height_tracker";
			pos.object_id = objID_;
			double outx;
			double outy;
			int inx = top_down_window_.x + top_down_window_.width / 2;
			int iny = top_down_window_.y + top_down_window_.height / 2;
			windowFrameToGlobal(inx, iny, outx, outy);
			ROS_INFO_STREAM(boost::format("Current window: (%d,%d) global: (%.2f,%.2f)") %inx %iny %outx %outy);
			pos.pos.x = outx;
			pos.pos.y = outy;
			pos.pos.z = windowDepthToGlobal(median_);
			double left, right, top, bottom;
			windowFrameToGlobal(top_down_window_.x, top_down_window_.y, left, top);
			windowFrameToGlobal(top_down_window_.x+top_down_window_.width, top_down_window_.y+top_down_window_.height, right, bottom);
			visualizeWindow(left, right, top, bottom, pos.pos.z);
			
			double reliability = 0.7; //on a scale of 0.1 to 1.0 (or actually 0 to 1, but below 0.1 isn't worth publishing)
			pos.covariance[0] = pow(0.3 / reliability,2.0);
			pos.covariance[1] = 0.0;
			pos.covariance[2] = 0.0;
			pos.covariance[3] = 0.0;
			pos.covariance[4] = pow(0.3 / reliability,2.0);
			pos.covariance[5] = 0.0;
			pos.covariance[6] = 0.0;
			pos.covariance[7] = 0.0;
			pos.covariance[8] = 10000.0;
			pos.initialization = 0;
			
			ROS_INFO_STREAM(boost::format("Publishing position measurement: (%.2f,%.2f)")
			                %pos.pos.x %pos.pos.y );
			pos_pub_.publish(pos);
		}
		
		//TODO: replace with image_publishers
		if(do_display_)
		{
		
			cvShowImage(window_name_.c_str(), annotatedFlattened);
			//cvShowImage(dimg_window_name_.c_str(), windowBG);
			//cvShowImage(limg_window_name_.c_str(), left_image);
			
			if (windowImg != 0)
				cvReleaseImage(&windowImg);
			/*windowImg = cvCreateImage(cvSize(windowBG->width, windowBG->height), IPL_DEPTH_8U, 3);
			cvCvtColor(windowBG, windowImg, CV_GRAY2BGR);
			
			if (tracking_) {
			  cvRectangle(windowImg,
			              cvPoint(window_.x                , window_.y                 ),
			              cvPoint(window_.x + window_.width, window_.y + window_.height),
			              cvScalar(255, 0, 0), 1);
			}
			cvShowImage(limg_window_name_.c_str(), windowImg);*/
			cvWaitKey(5);
		}
		
		cvReleaseImage(&flattened);
		cvReleaseImage(&annotatedFlattened);
	}
	
private:

//TODO: why doesn't this work?
	void visualizeWindow(double left, double right, double top, double bottom, double z)
	{
		static int trail_counter_ = 0;
		
		visualization_msgs::Marker marker;
		marker.header.frame_id = fixed_frame_;
		marker.header.stamp    = ros::Time::now();
		marker.ns              = "person_trail";
		marker.id              = trail_counter_;
		marker.lifetime        = ros::Duration(5.0);
		marker.type            = visualization_msgs::Marker::LINE_STRIP;
		marker.action          = visualization_msgs::Marker::ADD;
		
		trail_counter_++;
		if(trail_counter_ > 20) trail_counter_ = 0;
		
		geometry_msgs::Point p;
		p.x = left;
		p.y = top;
		p.z = z;
		marker.points.push_back(p);
		p.x = right;
		p.y = top;
		marker.points.push_back(p);
		p.x = right;
		p.y = bottom;
		marker.points.push_back(p);
		p.x = left;
		p.y = bottom;
		marker.points.push_back(p);
		p.x = left;
		p.y = top;
		marker.points.push_back(p);
		
		marker.scale.x = 0.02;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.color.a = 0.7;
		marker.color.b = 0.0 + 0.9 * ((double)trail_counter_ / 40.0);
		marker.color.g = 0.9 - 0.9 * ((double)trail_counter_ / 40.0);;
		marker.color.r = 0.1;
		
		marker_pub_.publish( marker );
	}
	
	/**
	 * Receive a message from the overall person tracking filter and
	 * start or update the tracked box if necessary
	 */
	void peopleCallback(const people_msgs::PositionMeasurement::ConstPtr &pos_ptr)
	{
		/*static ros::Time last_cb_(0);
		
		if( ros::Time::now() - last_cb_ < ros::Duration(0.2) )
		  return;
		last_cb_ = ros::Time::now();*/
		
		// Transform the position measurement to this frame
		const people_msgs::PositionMeasurement &pos = *pos_ptr;
		geometry_msgs::PointStamped in, out;
		in.point  = pos.pos;
		in.header = pos.header;
		try {
			tf_client_.transformPoint(fixed_frame_, in, out);
		}
		catch ( ... ) {
			ROS_ERROR("TF exception transforming incoming people_msgs::PositionMeasurement.");
			return;
		}
		
		ROS_INFO("=============================================");
		ROS_INFO_STREAM("Measurement in \""<<pos.header.frame_id<<"\", transforming to \""<<fixed_frame_<<"\"");
		
		// Find the location of the measurement in our window
		int xWin, yWin;
		globalFrameToWindow(out.point.x, out.point.y, xWin, yWin);
		
		int x = (top_down_window_.x+top_down_window_.width/2);
		int y = (top_down_window_.y+top_down_window_.height/2);
		int dist = abs((xWin - x) * (xWin - x) + (yWin - y) * (yWin - y));
		
		ROS_INFO_STREAM(boost::format("Got measurement at (%.2f,%.2f), transformed to (%.2f,%.2f)")
		                %pos.pos.x %pos.pos.y %out.point.x %out.point.y );
		                
		ROS_INFO_STREAM(boost::format("Current window (%.2f,%.2f) size (%.2f,%.2f)")
		                %top_down_window_.x %top_down_window_.y %top_down_window_.width %top_down_window_.height );
		                
		ROS_INFO_STREAM(boost::format("In window, measurement is (%.2f,%.2f), %.2f away from center")
		                %xWin %yWin %dist);
		                
		                
		const static int defaultW = 35;
		const static int defaultH = 55;
		if(dist > 1000)
		{
			top_down_window_.x      = xWin - defaultW/2;
			top_down_window_.width  = defaultW;
			top_down_window_.y      = yWin - defaultH/2;
			top_down_window_.height = defaultH;
			
			if(top_down_window_.x > 639-defaultW) top_down_window_.x = 639-defaultW;
			if(top_down_window_.x < 0)            top_down_window_.x = 0;
			if(top_down_window_.y > 479-defaultH) top_down_window_.y = 478-defaultH;
			if(top_down_window_.y < 0)            top_down_window_.y = 0;
			
			ROS_INFO_STREAM(boost::format("Lost tracker, setting window (%d,%d) size (%d,%d)")
			                %top_down_window_.x %top_down_window_.y %top_down_window_.width %top_down_window_.height);
			objID_ = pos.object_id;
			tracking_ = true;
			new_track_from_measurement_ = true;
		}
	}
	
// W=640, H=480

//Convert to and from the pixel coordinates in the flattened image frame of reference
	void windowFrameToGlobal(int x, int y, double &outx, double &outy)
	{
		//outx = (((double)x)*(maxLX-minLX)+minLX) / (double)(W-1); // original
		//outy = (((double)((H-1)-y))*(maxLY-minLY)+minLY) / (double)(H-1); // original
		
		double span_x = maxLX-minLX;
		double span_y = maxLY-minLY;
		
		
		outx =  ((double)x)*span_x / (double)(W-1) +minLX;
		outy =  -1* (((double)y)*span_y / (double)(H-1) +minLY);
	}
	
	void globalFrameToWindow(double x, double y, int &outx, int &outy)
	{
		//outx = (int)((W-1)*((x - minLX) / (maxLX-minLX))); // original
		//outy = (H-1) - (int)((H-1)*((y - minLY) / (maxLY-minLY))); // original
		
		double span_x = maxLX-minLX;
		double span_y = maxLY-minLY;
		
		outx = (int)((W-1)*((x - minLX) / span_x)); // original
		outy = (int)((H-1)*((-y - minLY) / span_y));
	}
	
	double windowDepthToGlobal(int depth)
	{
		return (((double)depth)*(maxLZ-minLZ)+minLZ) / 255.0;
	}
	
}; // end height_tracker class

}; // end namespace people


/**
 * Create a node, stick a HeightTracker in it, and launch
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "height_tracker");
	
	ros::NodeHandle nh;
	people::HeightTracker ft(nh);
	
	ros::spin();
}
