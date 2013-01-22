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
#include "leg_detector/matched_feature.h"
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"

// Message includes
#include "std_msgs/Header.h" // TODO: Necessary?

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

// C++ includes
#include <algorithm> // TODO: Necessary?

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

// Hardcoded constants... Be careful. Note: s for seconds, m for meters
static const double no_observation_timeout_s = 0.5; // How long a leg can last without a measurement before it is deleted.
static const double max_second_leg_age_s     = 2.0; // Used to disregard old legs without ID's, because they are unlikely to be our tracker.
static const double max_track_jump_m         = 1.0; // Max distance a leg can be from an input tracker and still update it.
static const double max_meas_jump_m          = 0.75; // Max distance a measurement can be away from a leg and still be used to update it.
static const double leg_pair_separation_m    = 1.0; // Max distance that should be between two legs associated with the same tracker.

// Input ROS params
static string fixed_frame_       = "/odom";
static string laser_scan_topic_  = "/scan";
static double connected_thresh_  = 0.0;

// Used to uniquely identify SavedFeature objects (tracked legs)
int SavedFeature::next_id = 0;

// Input from command line or launch file commands
int g_argc;
char** g_argv;


// Calculates two-dimensional distance between two points
double planeDist(Stamped<Point> p1, Stamped<Point> p2)
{
	return sqrt( (p1.getX()-p2.getX())*(p1.getX()-p2.getX()) + (p1.getY()-p2.getY())*(p1.getY()-p2.getY()) );
}

LegDetector::LegDetector(ros::NodeHandle nh)
    : nh_( nh ), 
    mask_count_( 0 ), 
    connected_thresh_( 0.06 ), 
    feat_count_( 0 ),
    people_sub_( nh_, "people_tracker_filter", 10 ),
    laser_sub_( nh_, laser_scan_topic, 1 ),
    filter_notifier_( filter_sub_, tfl_, fixed_frame, 10 ),
    laser_notifier_( laser_sub_, tfl_, fixed_frame, 1 ),
    max_pub_rate_( 0 )
{
    if (g_argc > 1) {
        forest.load(g_argv[1]);
        feat_count_ = forest.get_active_var_mask()->cols;
        printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
    } else {
        printf("Please provide a trained random forests classifier as an input.\n");
        shutdown();
    }

    ROS_INFO_STREAM("Loop rate is "       << max_pub_rate_temp << "Hz");
    ROS_INFO_STREAM("connected_thresh = " << connected_thresh_);
    
    // advertise topics
    leg_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("kalman_filt_cloud",10);
    tracker_measurements_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_measurements",1);

    filter_notifier_.registerCallback(boost::bind(&LegDetector::filterCallback, this, _1));
    filter_notifier_.setTolerance(ros::Duration(0.01));
    laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));

    feature_id_ = 0;
}


// People_tracker_filter callback                                       
// Find the legs (one or two) that are closest to this tracker message 
// If a leg was already assigned to a person, keep this assignment when
// the distance between them is not too large.                         
void filterCallback(const people_msgs::PositionMeasurement::ConstPtr& people_meas)
{
    ROS_DEBUG("(in tracker callback) Received a tracker message.");
    
    // If there are no legs, return.
    if (saved_features_.empty()) {
        ROS_DEBUG("(in tracker callback) Nothing to do here, no legs found yet.");
        return;
    }

    // Convert to a better data type
    Point pt;
    pointMsgToTF(people_meas->pos, pt);
    Stamped<Point> person_loc(pt, people_meas->header.stamp, people_meas->header.frame_id);
    person_loc[2] = 0.0; // Ignore the height of the person measurement.
    Stamped<Point> dest_loc(pt, people_meas->header.stamp, people_meas->header.frame_id); // Holder for all transformed pts.

    //-------- Locked ---------- 
    boost::mutex::scoped_lock lock(saved_mutex_);

    // Transform the input tracker measurement to fixed_frame
    try {
        tfl_.transformPoint(fixed_frame, people_meas->header.stamp, person_loc, fixed_frame, dest_loc);
    } catch(tf::TransformException e) {
        ROS_WARN_STREAM("(in tracker callback) Could not transform tracker to \""<<fixed_frame<<"\" at time "<<people_meas->header.stamp<<" - "<<e.what());
    }
    ROS_DEBUG_STREAM(boost::format("(in tracker callback) Tracker is at (%.2f,%.2f,%.2f)")%dest_loc.getX() %dest_loc.getY() %dest_loc.getZ());

    list<SavedFeature*>::iterator begin = saved_features_.begin();
    list<SavedFeature*>::iterator end = saved_features_.end();
    list<SavedFeature*>::iterator it1, it2;

    // If two legs are found with ID's matching the input tracker and still close enough to the input tracker, return
    // If one leg is found with an ID matching the input tracker and within the max dist, find a second leg that is
    //    close enough to the tracker and currently without an ID; assign it the tracker's ID. If no second leg exists, return.
    // If there are no legs with a matching ID and within the max dist, find a pair of legs with no ID that are close enough; ID them.
    // If all of the above cases fail, find a single close leg that is without an ID and assign the tracker's ID to it.

    // For each leg, get the two-dimensinoal distance to the input tracker
    ROS_DEBUG("(in tracker callback) Legs currently being tracked:");
    for (it1 = begin; it1 != end; ++it1){
        (*it1)->dist_to_person_ = planeDist(dest_loc, (*it1)->position_);
        ROS_DEBUG_STREAM(boost::format("(in tracker callback) Leg ID %s, Object ID %s at (%.2f,%.2f,%.2f) has distance %.2f")
            % (*it1)->id_ % (*it1)->object_id % (*it1)->position_.getX() % (*it1)->position_.getY() % (*it1)->position_.getZ() % (*it1)->dist_to_person_ );
    }

    // Try to find one or two trackers with ID's matching the input tracker and close enough to the tracker
    it2 = end; // Assign to end to give a basis for comparison to see if it has been changed
    for (it1 = begin; it1 != end; ++it1){
        // If this leg's ID matchest the tracker,
        if ((*it1)->object_id == people_meas->object_id){
            // and the distance from the leg to the tracker is small enough,
            if ((*it1)->dist_to_person_ < max_meas_jump_m){
                // if this is the first leg we've found, assign it to it2. Otherwise, leave it assigned to it1 and break.
                if (it2 == end)
                    it2 = it1;
                else
                    break;
            }
            // Otherwise, the matching ID isn't close enough to the tracker anymore, so remove the ID of the leg.
            else
                (*it1)->object_id = ""; // the leg moved too far apart from the tracker. This shouldn't happen, oops.
        }
    }

    // If we found two legs with the correct ID and within the max distance, all is good, return.
    if (it1 != end && it2 != end){
        ROS_DEBUG_STREAM("(in tracker callback) Found two legs with ID's matching the input tracker");
        return;
    }

    list<SavedFeature*>::iterator closest = end;  // Single leg found by proximity
    list<SavedFeature*>::iterator closest1 = end; // First leg found by proximity
    list<SavedFeature*>::iterator closest2 = end; // Second leg found by proximity

    // If we found ONE close leg with a matching ID, let's try to find a second leg that 
    //   * doesn't yet have an ID,
    //   * is within the max distance,
    //   * is less than max_second_leg_age_s old. TODO: Is this condition necessary?
    float dist_between_legs, closest_dist, closest_pair_dist;
    if (it2 != end){
        closest_dist = max_meas_jump_m;
        closest = saved_features_.end();

        for (it1 = begin; it1 != end; ++it1){
            // Skip this leg if:
            // - you're already using it.
            // - it already has an id.
            // - it's too old (Old legs without ID's are unlikely to be the second leg in a pair)
            // - it's too far away from the person.
            if ((it1 == it2) || ((*it1)->object_id != "") ||  ((*it1)->getLifetime() > max_second_leg_age_s) || ((*it1)->dist_to_person_ >= closest_dist) )
                continue;
            
            dist_between_legs = planeDist((*it1)->position_, (*it2)->position_);
        
            // If this is the closest dist (and within range), and the legs are close together and unlabeled, mark it.
            if ( dist_between_legs < leg_pair_separation_m ){
                closest = it1;
                closest_dist = (*it1)->dist_to_person_;
            }
        }

        // If we found a close leg without an ID, set it's ID to match the input tracker.
        if (closest != end){
            ROS_DEBUG_STREAM("(in tracker callback) Found a leg without an ID close enough to the tracker. Now this tracker has two legs with matching ID's");
            (*closest)->object_id = people_meas->object_id;
        }
        else 
            ROS_DEBUG("(in tracker callback) Only one leg could be matched to the tracker (by ID)");
        
        // Regardless of whether we found a second leg, return.
        return;
    }

    // If we didn't find any legs with the input tracker's ID, try to find two legs without ID's that are close together and close to the tracker.
    closest = end;
    closest1 = end; // These variable are set this way to have something to compare to.
    closest2 = end;

    closest_dist = max_meas_jump_m;
    closest_pair_dist = 2*max_meas_jump_m;
    for (it1 = begin, it2 = begin; it1 != end; ++it1){
        // Only look at trackers without ids and that are not too far away.
        if ((*it1)->object_id != "" || (*it1)->dist_to_person_ >= max_meas_jump_m )
            continue;

        // Keep the single closest leg around in case none of the pairs work out.
        if ( (*it1)->dist_to_person_ < closest_dist ){
            closest_dist = (*it1)->dist_to_person_;
            closest = it1;
        }

        // Find a second leg.
        it2 = it1;
        it2++;
        for (; it2 != end; ++it2){
            // Only look at trackers without ids and that are not too far away.
            if ((*it2)->object_id != "" || (*it2)->dist_to_person_ >= max_meas_jump_m ) 
                continue;
             
            dist_between_legs = planeDist((*it1)->position_, (*it2)->position_);
        
            // Ensure that this pair of legs is the closest pair to the tracker, and that the distance between the legs isn't too large.
            if ( (*it1)->dist_to_person_+(*it2)->dist_to_person_ < closest_pair_dist && dist_between_legs < leg_pair_separation_m ){
                closest_pair_dist = (*it1)->dist_to_person_+(*it2)->dist_to_person_;
                closest1 = it1;
                closest2 = it2;
            }
        }
    }
    // Found a pair of legs.
    if (closest1 != end && closest2 != end){
        (*closest1)->object_id = people_meas->object_id;
        (*closest2)->object_id = people_meas->object_id;
        ROS_DEBUG("(in tracker callback) Found two legs without ID's close enough to the input tracker. The tracker now has two matching legs.");
        return;
    }

    // No pair worked, try for just one leg.
    if (closest != end){
        (*closest)->object_id = people_meas->object_id;
        ROS_DEBUG("(in tracker callback) Only one leg could be matched to the tracker (by proximity)");
        return;
    }

    lock.unlock();
    // -------- Locked ---------

    ROS_DEBUG("(in tracker callback) No legs could be matched to the input tracker by ID or by proximity");
}


///////////////////////////////////////////////////////////////////////////////
// laser scan callback                                                       //
//                                                                           //
// Process the laser scan and find possible legs in that data. Output        //
// each of these in a point cloud. However, only publish back updated        //
// measurements for the trackers if the legs have an tracker ID (object ID)  //
///////////////////////////////////////////////////////////////////////////////
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // Ignore laser scans if they come too quickly. This keeps updates consistent.
    static ros::Time last_cb_time(0);
    if( ros::Time::now() - last_cb_time < max_pub_rate_.expectedCycleTime() )
        return;

    last_cb_time = ros::Time::now();
    max_pub_rate_.reset();
    
    ROS_DEBUG("(in laser callback) Received new laser scan");
    
    // Process the laser scan into 'clusters'
    ScanProcessor processor(*scan, mask_);
    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(3);

    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

    // -------- Locked ------------
    boost::mutex::scoped_lock lock(saved_mutex_);

    // if a leg's measurement hasn't been updated in <no_observation_timeout> seconds: erase that leg
    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);
    list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end()){
        if ((*sf_iter)->meas_time_ < purge){
            delete (*sf_iter);
            saved_features_.erase(sf_iter++);
        }
        else
            ++sf_iter;
    }

    // System update of trackers, based on predicted movement, and copy updated ones in propagate list
    list<SavedFeature*> propagated;
    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin(); sf_iter != saved_features_.end(); sf_iter++){
        (*sf_iter)->propagate(scan->header.stamp);
        propagated.push_back(*sf_iter);
    }

    string s = "(in laser callback) Predicted values from calculated leg features are: ";
    // Detection step: build up the set of "candidate" clusters; based on predicted value from clusters' calculated leg features.
    list<SampleSet*> candidates;
    for (list<SampleSet*>::iterator i = processor.getClusters().begin(); i != processor.getClusters().end(); i++){
        vector<float> f = calcLegFeatures(*i, *scan);

        for (int k = 0; k < feat_count_; k++)
            tmp_mat->data.fl[k] = (float)(f[k]);

        s += ((boost::format("%.1f  ") % forest.predict( tmp_mat )).str());
        if (forest.predict( tmp_mat ) > 0)
            candidates.push_back(*i);
    }
    ROS_DEBUG_STREAM(s);

    // For each candidate, find the closest leg (within threshold) and add to the match list
    // If no close leg is found, start a new one
    multiset<MatchedFeature> matches;
    for (list<SampleSet*>::iterator cf_iter = candidates.begin(); cf_iter != candidates.end(); cf_iter++){
        Stamped<Point> loc((*cf_iter)->center(), scan->header.stamp, scan->header.frame_id);
        try {
            tfl_.transformPoint(fixed_frame, loc, loc);
        } 
        catch( tf::TransformException e ) {
                ROS_WARN_STREAM("(in laser callback) Could not transform leg candidate to fixed_frame (1)- " << e.what());
        }

        list<SavedFeature*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;
        
        for (list<SavedFeature*>::iterator pf_iter = propagated.begin(); pf_iter != propagated.end(); pf_iter++){
            // find the closest leg to each candidate
            float dist = loc.distance((*pf_iter)->position_);
            if ( dist < closest_dist ){
                closest = pf_iter;
                closest_dist = dist;
            }
        }
        // Nothing close to it, start a new track
        if (closest == propagated.end()) 
            saved_features_.insert(saved_features_.end(), new SavedFeature(loc, &tfl_));

        // Add the candidate, the leg, and the distance distance between them to a match list
        else
            matches.insert(MatchedFeature(*cf_iter,*closest,closest_dist));
    }

    // loop through _sorted_ (by distance) matches list
    // find the match with the shortest distance for each leg
    while (matches.size() > 0){
        multiset<MatchedFeature>::iterator matched_iter = matches.begin();
        bool found = false;
        list<SavedFeature*>::iterator pf_iter = propagated.begin();
        while (pf_iter != propagated.end()){
            // update the leg with this candidate because it is the closest
            if (matched_iter->closest_ == *pf_iter){
                // Transform candidate to fixed frame
                Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
                try {
                tfl_.transformPoint(fixed_frame, loc, loc);
                } catch( tf::TransformException e ) {
                    ROS_WARN_STREAM("(in laser callback) Could not transform leg candidate to fixed_frame (2)- " << e.what());
                }

                // Update the tracker with the candidate location
                matched_iter->closest_->update(loc);
                
                // remove this match and 
                matches.erase(matched_iter);
                propagated.erase(pf_iter++);
                found = true;
                break;
            }
            // still looking for the leg to update
            else
                pf_iter++;
        }

        // didn't find leg to update, because it was deleted above from the propagated list 
        // (happens when this candidate is not the closest to the leg but still within threshold)
        // try to assign the candidate to another leg
        if (!found){
            Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
            try {
                tfl_.transformPoint(fixed_frame, loc, loc);
            } catch( tf::TransformException e ) {
                ROS_WARN_STREAM("(in laser callback) Could not transform leg candidate to fixed_frame (3) - " << e.what());
            }

            list<SavedFeature*>::iterator closest = propagated.end();
            float closest_dist = max_track_jump_m;
        
            for (list<SavedFeature*>::iterator remain_iter = propagated.begin(); remain_iter != propagated.end(); remain_iter++){
                float dist = loc.distance((*remain_iter)->position_);
                if ( dist < closest_dist ){
                    closest = remain_iter;
                    closest_dist = dist;
                }
            }

            // no leg is within a threshold of this candidate, since the leg we thought it was close to was actually closer to another measurement,
            // so create a new leg for this candidate
            if (closest == propagated.end())
                saved_features_.insert(saved_features_.end(), new SavedFeature(loc, &tfl_));
            // Otherwise, we have made another match so insert it into the list. (It is inserted in sorted order automatically)
            else
                matches.insert(MatchedFeature(matched_iter->candidate_,*closest,closest_dist));

            matches.erase(matched_iter);
        }
    }// End updating legs via matched candidates

    cvReleaseMat(&tmp_mat); tmp_mat = 0;

    vector<geometry_msgs::Point32> filter_visualize(saved_features_.size());
    vector<float> weights(saved_features_.size());
    sensor_msgs::ChannelFloat32 channel;

    // Now that saved features have all been updated with laser values, output them!
    int i = 0;
    map<string, SavedFeature*> legs_matched;
    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin(); sf_iter != saved_features_.end(); sf_iter++,i++){
        // reliability and covariance calculationsV
        Stamped<Point> pos = (*sf_iter)->position_;
        double reliability = 1.0;// fmin(1.0, fmax(0.2, est.vel_.length() / 0.5)); // Not used as reliability, just for covariance calculation.
        double covariance = pow(0.3 / reliability,2.0); // TODO: Investigate this calculation a bit more to see if it's reasonable to use it.

        // point cloud output for each leg (will be outputed after this loop once the entire cloud has been constructed)
        filter_visualize[i].x = pos[0];
        filter_visualize[i].y = pos[1];
        filter_visualize[i].z = pos[2];
        weights[i] = *(float*)&(rgb[min(998, max(1, (int)trunc( reliability*999.0 )))]);

        // If there are two leg's with the same ID (both have been associated with the same tracker), output their updated center to the filter.
        // If the current object has no ID, obviously skip over it.
        if ((*sf_iter)->object_id == "")
            continue;
        // If the current leg matches the ID of a leg that has already been seen, output the average of their locations to the filter
        map<string, SavedFeature*>::iterator first_leg;
        if ( (first_leg = legs_matched.find((*sf_iter)->object_id)) != legs_matched.end() ){
            // Average posiiton of the first and second leg
            Stamped<Point> first_pos = (first_leg->second)->position_;
            float average_x = (pos[0] + first_pos[0])/2;
            float average_y = (pos[1] + first_pos[1])/2;

            // output measurement to the filter to update this tracker's position
            people_msgs::PositionMeasurement output;
            output.header.stamp = (*sf_iter)->time_;
            output.header.frame_id = fixed_frame;
            output.name = "leg_detector";
            output.object_id = (*sf_iter)->object_id;
            output.pos.x = average_x;
            output.pos.y = average_y;
            output.pos.z = 0.0;
            output.reliability = 0.5;//reliability; // calculated reliability is given up and replaced with hard coded value. Kind of stupid, if you ask me.
            output.covariance[0] = covariance; // Doing the above comment causes only face detector to be able to create a new tracker, and that's all it's used for.
            output.covariance[1] = 0.0;
            output.covariance[2] = 0.0;
            output.covariance[3] = 0.0;
            output.covariance[4] = covariance; 
            output.covariance[5] = 0.0;
            output.covariance[6] = 0.0;
            output.covariance[7] = 0.0;
            output.covariance[8] = 0.0; // Variance for the z component is really high (unreliable) because we don't care about it, so we just put it to 0 before.
            output.initialization = 0; // initialization=1 is required to instantiate new trackers in the filter, so leg_detector is never allowed to do so because of this.
        
            tracker_measurements_pub_.publish(output);
            ROS_INFO_STREAM(boost::format("(in laser callback) Publishing new measurement for \"%s\", covariance=%.2f")%(*sf_iter)->object_id %covariance );

            // Delete this ID from legs_matched so that it isn't accidentally republished
            legs_matched.erase( first_leg );
        }
        // Otherwise, add this leg to the map and keep looking
        else
            legs_matched.insert( pair<string, SavedFeature*>((*sf_iter)->object_id, *sf_iter) );
    }

    lock.unlock();
    // ----------- Locked ----------

    // visualize all legs as points in a point cloud based on the center of each leg.
    channel.name = "rgb";
    channel.values = weights;
    sensor_msgs::PointCloud  people_cloud; 
    people_cloud.channels.push_back(channel);
    people_cloud.header.frame_id = fixed_frame;//scan_.header.frame_id;
    people_cloud.header.stamp = scan->header.stamp;
    people_cloud.points  = filter_visualize;
    leg_cloud_pub_.publish(people_cloud);

} // End laser callback


// Calculate leg features function
vector<float> calcLegFeatures(SampleSet* cluster, const sensor_msgs::LaserScan& scan)
{

  vector<float> features;

  // Number of points
  int num_points = cluster->size();
  //  features.push_back(num_points);

  // Compute mean and median points for future use
  float x_mean = 0.0;
  float y_mean = 0.0;
  vector<float> x_median_set;
  vector<float> y_median_set;
  for (SampleSet::iterator i = cluster->begin();
       i != cluster->end();
       i++)

  {
    x_mean += ((*i)->x)/num_points;
    y_mean += ((*i)->y)/num_points;
    x_median_set.push_back((*i)->x);
    y_median_set.push_back((*i)->y);
  }

  std::sort(x_median_set.begin(), x_median_set.end());
  std::sort(y_median_set.begin(), y_median_set.end());

  float x_median = 0.5 * ( *(x_median_set.begin() + (num_points-1)/2) + *(x_median_set.begin() + num_points/2) );
  float y_median = 0.5 * ( *(y_median_set.begin() + (num_points-1)/2) + *(y_median_set.begin() + num_points/2) );

  //Compute std and avg diff from median

  double sum_std_diff = 0.0;
  double sum_med_diff = 0.0;


  for (SampleSet::iterator i = cluster->begin();
       i != cluster->end();
       i++)

  {
    sum_std_diff += pow( (*i)->x - x_mean, 2) + pow((*i)->y - y_mean, 2);
    sum_med_diff += sqrt(pow( (*i)->x - x_median, 2) + pow((*i)->y - y_median, 2));
  }

  float std = sqrt( 1.0/(num_points - 1.0) * sum_std_diff);
  float avg_median_dev = sum_med_diff / num_points;

  features.push_back(std);
  features.push_back(avg_median_dev);


  // Take first at last
  SampleSet::iterator first = cluster->begin();
  SampleSet::iterator last = cluster->end();
  last--;

  // Compute Jump distance
  int prev_ind = (*first)->index - 1;
  int next_ind = (*last)->index + 1;

  float prev_jump = 0;
  float next_jump = 0;

  if (prev_ind >= 0)
  {
    Sample* prev = Sample::Extract(prev_ind, scan);
    if (prev)
    {
      prev_jump = sqrt( pow( (*first)->x - prev->x, 2) + pow((*first)->y - prev->y, 2));
      delete prev;
    }

  }

  if (next_ind < (int)scan.ranges.size())
  {
    Sample* next = Sample::Extract(next_ind, scan);
    if (next)
    {
      next_jump = sqrt( pow( (*last)->x - next->x, 2) + pow((*last)->y - next->y, 2));
      delete next;
    }
  }

  features.push_back(prev_jump);
  features.push_back(next_jump);

  // Compute Width
  float width = sqrt( pow( (*first)->x - (*last)->x, 2) + pow((*first)->y - (*last)->y, 2));
  features.push_back(width);

  // Compute Linearity

  CvMat* points = cvCreateMat( num_points, 2, CV_64FC1);
  {
    int j = 0;
    for (SampleSet::iterator i = cluster->begin();
         i != cluster->end();
         i++)
    {
      cvmSet(points, j, 0, (*i)->x - x_mean);
      cvmSet(points, j, 1, (*i)->y - y_mean);
      j++;
    }
  }

  CvMat* W = cvCreateMat( 2, 2, CV_64FC1);
  CvMat* U = cvCreateMat( num_points, 2, CV_64FC1);
  CvMat* V = cvCreateMat( 2, 2, CV_64FC1);
  cvSVD(points, W, U, V);

  CvMat* rot_points = cvCreateMat(num_points, 2, CV_64FC1);
  cvMatMul(U,W,rot_points);

  float linearity = 0.0;
  for (int i = 0; i < num_points; i++)
  {
    linearity += pow(cvmGet(rot_points, i, 1), 2);
  }

  cvReleaseMat(&points); points = 0;
  cvReleaseMat(&W); W = 0;
  cvReleaseMat(&U); U = 0;
  cvReleaseMat(&V); V = 0;
  cvReleaseMat(&rot_points); rot_points = 0;

  features.push_back(linearity);

  // Compute Circularity
  CvMat* A = cvCreateMat( num_points, 3, CV_64FC1);
  CvMat* B = cvCreateMat( num_points, 1, CV_64FC1);
  {
    int j = 0;
    for (SampleSet::iterator i = cluster->begin();
         i != cluster->end();
         i++)
    {
      float x = (*i)->x;
      float y = (*i)->y;

      cvmSet(A, j, 0, -2.0*x);
      cvmSet(A, j, 1, -2.0*y);
      cvmSet(A, j, 2, 1);

      cvmSet(B, j, 0, -pow(x,2)-pow(y,2));
      j++;
    }
  }
  CvMat* sol = cvCreateMat( 3, 1, CV_64FC1);

  cvSolve(A, B, sol, CV_SVD);

  float xc = cvmGet(sol, 0, 0);
  float yc = cvmGet(sol, 1, 0);
  float rc = sqrt(pow(xc,2) + pow(yc,2) - cvmGet(sol, 2, 0));

  cvReleaseMat(&A); A = 0;
  cvReleaseMat(&B); B = 0;
  cvReleaseMat(&sol); sol = 0;

  float circularity = 0.0;
  for (SampleSet::iterator i = cluster->begin();
       i != cluster->end();
       i++)
  {
    circularity += pow( rc - sqrt( pow(xc - (*i)->x, 2) + pow( yc - (*i)->y, 2) ), 2);
  }

  features.push_back(circularity);

  // Radius
  float radius = rc;

  features.push_back(radius);

  //Curvature:
  float mean_curvature = 0.0;

  //Boundary length:
  float boundary_length = 0.0;
  float last_boundary_seg = 0.0;

  float boundary_regularity = 0.0;
  double sum_boundary_reg_sq = 0.0;

  // Mean angular difference
  SampleSet::iterator left = cluster->begin();
  left++;
  left++;
  SampleSet::iterator mid = cluster->begin();
  mid++;
  SampleSet::iterator right = cluster->begin();

  float ang_diff = 0.0;

  while (left != cluster->end())
  {
    float mlx = (*left)->x - (*mid)->x;
    float mly = (*left)->y - (*mid)->y;
    float L_ml = sqrt(mlx*mlx + mly*mly);

    float mrx = (*right)->x - (*mid)->x;
    float mry = (*right)->y - (*mid)->y;
    float L_mr = sqrt(mrx*mrx + mry*mry);

    float lrx = (*left)->x - (*right)->x;
    float lry = (*left)->y - (*right)->y;
    float L_lr = sqrt(lrx*lrx + lry*lry);

    boundary_length += L_mr;
    sum_boundary_reg_sq += L_mr*L_mr;
    last_boundary_seg = L_ml;

    float A = (mlx*mrx + mly*mry) / pow(L_mr, 2);
    float B = (mlx*mry - mly*mrx) / pow(L_mr, 2);

    float th = atan2(B,A);

    if (th < 0)
      th += 2*M_PI;

    ang_diff += th / num_points;

    float s = 0.5*(L_ml+L_mr+L_lr);
    float area = sqrt( s*(s-L_ml)*(s-L_mr)*(s-L_lr) );

    if (th > 0)
      mean_curvature += 4*(area)/(L_ml*L_mr*L_lr*num_points);
    else
      mean_curvature -= 4*(area)/(L_ml*L_mr*L_lr*num_points);

    left++;
    mid++;
    right++;
  }

  boundary_length += last_boundary_seg;
  sum_boundary_reg_sq += last_boundary_seg*last_boundary_seg;

  boundary_regularity = sqrt( (sum_boundary_reg_sq - pow(boundary_length,2)/num_points)/(num_points - 1) );

  features.push_back(boundary_length);
  features.push_back(ang_diff);
  features.push_back(mean_curvature);

  features.push_back(boundary_regularity);


  // Mean angular difference
  first = cluster->begin();
  mid = cluster->begin();
  mid++;
  last = cluster->end();
  last--;
  
  double sum_iav = 0.0;
  double sum_iav_sq  = 0.0;

  while (mid != last)
  {
    float mlx = (*first)->x - (*mid)->x;
    float mly = (*first)->y - (*mid)->y;
    //float L_ml = sqrt(mlx*mlx + mly*mly);

    float mrx = (*last)->x - (*mid)->x;
    float mry = (*last)->y - (*mid)->y;
    float L_mr = sqrt(mrx*mrx + mry*mry);

    //float lrx = (*first)->x - (*last)->x;
    //float lry = (*first)->y - (*last)->y;
    //float L_lr = sqrt(lrx*lrx + lry*lry);
      
    float A = (mlx*mrx + mly*mry) / pow(L_mr, 2);
    float B = (mlx*mry - mly*mrx) / pow(L_mr, 2);

    float th = atan2(B,A);

    if (th < 0)
      th += 2*M_PI;

    sum_iav += th;
    sum_iav_sq += th*th;

    mid++;
  }

  float iav = sum_iav / num_points;
  float std_iav = sqrt( (sum_iav_sq - pow(sum_iav,2)/num_points)/(num_points - 1) );

  features.push_back(iav);
  features.push_back(std_iav);

  return features;
}


// Main
int main(int argc, char **argv)
{
	ros::init( argc, argv, "leg_detector" );
	g_argc = argc;
	g_argv = argv;

    // ROS parameters
    ros::NodeHandle pnh( "~" );
    pnh.param( "odom_frame", fixed_frame, string("") );
    pnh.param( "laser_scan_topic", laser_scan_topic, string("") );
    pnh.param("max_pub_rate"    , max_pub_rate_s_, 1.0);
    pnh.param("connected_thresh", connected_thresh_, .06);

	ros::NodeHandle nh;

	LegDetector ld( nh );

	ros::spin();
	
	return 0;
}
