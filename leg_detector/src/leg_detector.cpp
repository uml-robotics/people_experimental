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
#include "leg_detector/saved_feature.h"
#include "leg_detector/laser_processor.h"
#include "leg_detector/calc_leg_features.h"

// Non-standard ROS includes
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"

// Message includes
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud.h"

// C++ includes
#include <algorithm>


// Namespaces
using namespace saved_feature;
using namespace laser_processor;
using namespace estimation;
using namespace std;
using namespace tf;
using namespace ros;
using namespace BFL;
using namespace MatrixWrapper;


// Used to uniquely identify SavedFeature objects (tracked legs)
int SavedFeature::next_id = 0;

// Hardcoded constants... Be careful. Note: s for seconds, m for meters
static const double no_observation_timeout_s = 0.8; // How long a leg will last without a measurement before it is deleted.
static const double max_second_leg_age_s     = 1.0; // Used to disregard old legs without ID's, because they are unlikely to be our tracker.
static const double max_track_jump_m         = 0.6; // Max distance a leg can be from an input tracker and still update it.
static const double max_meas_jump_m          = 0.8; // Max distance a measurement can be away from a leg and still be used to update it.
static const double leg_pair_separation_m    = 0.6; // Max distance that should be between two legs associated with the same tracker.


// Helper function that calculates two-dimensional distance between two points
double planeDist(Stamped<Point> p1, Stamped<Point> p2)
{
	return sqrt( (p1.getX()-p2.getX())*(p1.getX()-p2.getX()) + (p1.getY()-p2.getY())*(p1.getY()-p2.getY()) );
}


LegDetector::LegDetector(ros::NodeHandle nh, string fixed_frame, string laser_scan_topic, double max_pub_rate_s, double connected_thresh, int g_argc, char** g_argv)
    : nh_( nh ), 
    connected_thresh_( connected_thresh ),
    fixed_frame_( fixed_frame ),
    feat_count_( 0 ),
    filter_sub_( nh_, "people_tracker_filter", 10 ),
    laser_sub_( nh_, laser_scan_topic, 1 ),
    filter_notifier_( filter_sub_, tfl_, fixed_frame, 10 ),
    laser_notifier_( laser_sub_, tfl_, fixed_frame, 1 ),
    max_pub_rate_( max_pub_rate_s )
{
    if (g_argc > 1) {
        forest_.load(g_argv[1]);
        feat_count_ = forest_.get_active_var_mask()->cols;
        ROS_INFO("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
    } else {
        ROS_INFO("Please provide a trained random forests classifier as an input.\n");
        shutdown();
    }

    ROS_INFO_STREAM("Loop rate is "       << max_pub_rate_s << "Hz");
    ROS_INFO_STREAM("connected_thresh = " << connected_thresh_);
    
    // advertise topics
    leg_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("kalman_filt_cloud",10);
    measurements_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_measurements",1);

    filter_notifier_.registerCallback(boost::bind(&LegDetector::filterCallback, this, _1));
    filter_notifier_.setTolerance(ros::Duration(0.01));
    laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
}


// People_tracker_filter callback                                       
// Find the legs (one or two) that are closest to this tracker message 
// If a leg was already assigned to a person, keep this assignment when
// the distance between them is not too large.                         
void LegDetector::filterCallback(const people_msgs::PositionMeasurement::ConstPtr& people_meas)
{
    ROS_DEBUG("(in tracker callback) Received a tracker message from the filter.");
    
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

    //-------- Locked ---------- 
    //boost::mutex::scoped_lock lock(saved_mutex_);

    // Transform the input tracker measurement to fixed_frame
    try {
        tfl_.transformPoint(fixed_frame_, person_loc, person_loc);
    } catch(tf::TransformException e) {
        ROS_WARN_STREAM("(in tracker callback) Could not transform tracker to \""<<fixed_frame_<<"\" at time "<<people_meas->header.stamp<<" - "<<e.what());
    }
    ROS_DEBUG_STREAM(boost::format("(in tracker callback) Tracker is at (%.2f,%.2f,%.2f)")%person_loc.getX() %person_loc.getY() %person_loc.getZ());

    list<SavedFeature*>::iterator begin = saved_features_.begin();
    list<SavedFeature*>::iterator end = saved_features_.end();
    list<SavedFeature*>::iterator it1, it2;

    // If two legs are found with ID's matching the input tracker and still close enough to the input tracker, return
    // If one leg is found with an ID matching the input tracker and within the max dist, find a second leg that is
    //    close enough to the tracker and currently without an ID; assign it the tracker's ID. If no second leg exists, return.
    // If there are no legs with a matching ID and within the max dist, find a pair of legs with no ID that are close enough; ID them.
    // If all of the above cases fail, find a single close leg that is without an ID and assign the tracker's ID to it.

    // For each leg, get the two-dimensional distance to the tracker
    ROS_DEBUG("(in tracker callback) Legs currently being tracked:");
    for (it1 = begin; it1 != end; ++it1){
        (*it1)->setDist( planeDist(person_loc, (*it1)->getPosition()) );
        ROS_DEBUG_STREAM(boost::format("(in tracker callback) Leg ID %s, Object ID %s at (%.2f,%.2f,%.2f) has distance %.2f")
            % (*it1)->getID() % (*it1)->getObjectID() % (*it1)->getPosition().getX() % (*it1)->getPosition().getY() % (*it1)->getPosition().getZ() % (*it1)->getDist() );
    }

    // Try to find one or two trackers with ID's matching the input tracker and close enough to the tracker
    for (it1 = begin, it2=end; it1 != end; ++it1){
        // If this leg's ID matchest the tracker,
        if ((*it1)->getObjectID() == people_meas->object_id){
            // and the distance from the leg to the tracker is small enough,
            if ((*it1)->getDist() < max_meas_jump_m){
                // if this is the first leg we've found, assign it to it2. Otherwise, leave it assigned to it1 and break.
                if (it2 == end)
                    it2 = it1;
                else
                    break;
            }
            // Otherwise, the matching ID isn't close enough to the tracker anymore, so remove the ID of the leg.
            else
                (*it1)->disassociateTracker(); // the leg moved too far apart from the tracker. This shouldn't happen, oops.
        }
    }

    // If we found two legs with the correct ID and within the max distance, all is good, return.
    if (it1 != end && it2 != end){
        ROS_DEBUG_STREAM("(in tracker callback) Found two legs with ID's matching the input tracker");
        return;
    }

    list<SavedFeature*>::iterator closest = end;  // Single leg found by proximity

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
            if ((it1 == it2) || ((*it1)->getObjectID() != "") ||  ((*it1)->getLifetime() > max_second_leg_age_s) || ((*it1)->getDist() >= closest_dist) )
                continue;
            
            dist_between_legs = planeDist((*it1)->getPosition(), (*it2)->getPosition());
        
            // If this is the closest dist (and within range), and the legs are close together and unlabeled, mark it.
            if ( dist_between_legs < leg_pair_separation_m ){
                closest = it1;
                closest_dist = (*it1)->getDist();
            }                                                   
        }

        // If we found a close leg without an ID, set it's ID to match the input tracker.
        if (closest != end){
            ROS_DEBUG_STREAM("(in tracker callback) Found a leg without an ID close enough to the tracker. Now this tracker has two legs with matching ID's");
            (*closest)->associateTracker( people_meas->object_id );
        }
        else 
            ROS_DEBUG("(in tracker callback) Only one leg could be matched to the tracker (by ID)");
        
        // Regardless of whether we found a second leg, return.
        return;
    }

    // If we didn't find any legs with the input tracker's ID, try to find two legs without ID's that are close together and close to the tracker.
    closest = end;
    list<SavedFeature*>::iterator closest1 = end; // These variable are set this way to have something to compare to.
    list<SavedFeature*>::iterator closest2 = end;

    closest_dist = max_meas_jump_m;
    closest_pair_dist = 2*max_meas_jump_m;
    for (it1 = begin, it2 = begin; it1 != end; ++it1){
        // Only look at legs without IDs and that are not too far away.
        if ((*it1)->getObjectID() != "" || (*it1)->getDist() >= max_meas_jump_m )
            continue;

        // Keep the single closest leg around in case none of the pairs work out.
        if ( (*it1)->getDist() < closest_dist ){
            closest_dist = (*it1)->getDist();
            closest = it1;
        }

        // Find a second leg.
        it2 = it1;
        it2++;
        for (; it2 != end; ++it2){
            // Only look at trackers without ids and that are not too far away.
            if ((*it2)->getObjectID() != "" || (*it2)->getDist() >= max_meas_jump_m ) 
                continue;
             
            dist_between_legs = planeDist((*it1)->getPosition(), (*it2)->getPosition());
        
            // Ensure that this pair of legs is the closest pair to the tracker, and that the distance between the legs isn't too large.
            if ( (*it1)->getDist()+(*it2)->getDist() < closest_pair_dist && dist_between_legs < leg_pair_separation_m ){
                closest_pair_dist = (*it1)->getDist()+(*it2)->getDist();
                closest1 = it1;
                closest2 = it2;
            }
        }
    }
    // Found a pair of legs.
    if (closest1 != end && closest2 != end){
        (*closest1)->associateTracker( people_meas->object_id );
        (*closest2)->associateTracker( people_meas->object_id );
        ROS_DEBUG("(in tracker callback) Found two legs without ID's close enough to the input tracker. The tracker now has two matching legs.");
        return;
    }

    // No pair worked, try for just one leg.
    if (closest != end){
        (*closest)->associateTracker( people_meas->object_id );
        ROS_DEBUG("(in tracker callback) Only one leg could be matched to the tracker (by proximity)");
        return;
    }

    //lock.unlock();
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
void LegDetector::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // Ignore laser scans if they come too quickly. This keeps updates consistent.
    static ros::Time last_cb_time(0);
    if( ros::Time::now() - last_cb_time < max_pub_rate_.expectedCycleTime() ){
        return;
    }

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
    ros::Time purge = scan->header.stamp - ros::Duration(no_observation_timeout_s);
    list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end()){
        if ((*sf_iter)->getMeasTime() < purge){
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

        s += ((boost::format("%.1f  ") % forest_.predict( tmp_mat )).str());
        if (forest_.predict( tmp_mat ) > 0)
            candidates.push_back(*i);
    }
    ROS_DEBUG_STREAM(s);


    // For each candidate, find the closest leg (within threshold) and add to the match list
    // If no close leg is found, start a new one
    multiset<MatchedFeature> matches;
    for (list<SampleSet*>::iterator cf_iter = candidates.begin(); cf_iter != candidates.end(); cf_iter++){
        Stamped<Point> loc((*cf_iter)->center(), scan->header.stamp, scan->header.frame_id);
        try {
            tfl_.transformPoint(fixed_frame_, loc, loc);
        } 
        catch( tf::TransformException e ) {
                ROS_WARN_STREAM("(in laser callback) Could not transform leg candidate to fixed_frame (1)- " << e.what());
        }

        list<SavedFeature*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;
        
        // find the closest leg to each candidate
        for (list<SavedFeature*>::iterator pf_iter = propagated.begin(); pf_iter != propagated.end(); pf_iter++){
            float dist = loc.distance((*pf_iter)->getPosition());
            if ( dist < closest_dist ){
                closest = pf_iter;
                closest_dist = dist;
            }
        }
        // Nothing close to it, start a new track
        if (closest == propagated.end()) 
            saved_features_.insert(saved_features_.end(), new SavedFeature(loc, &tfl_, fixed_frame_));

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
                tfl_.transformPoint(fixed_frame_, loc, loc);
                } catch( tf::TransformException e ) {
                    ROS_WARN_STREAM("(in laser callback) Could not transform leg candidate to fixed_frame (2)- " << e.what());
                }

                // Update the tracker with the candidate location
                matched_iter->closest_->addMeasurement(loc);
                
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
                tfl_.transformPoint(fixed_frame_, loc, loc);
            } catch( tf::TransformException e ) {
                ROS_WARN_STREAM("(in laser callback) Could not transform leg candidate to fixed_frame (3) - " << e.what());
            }

            list<SavedFeature*>::iterator closest = propagated.end();
            float closest_dist = max_track_jump_m;
        
            for (list<SavedFeature*>::iterator remain_iter = propagated.begin(); remain_iter != propagated.end(); remain_iter++){
                float dist = loc.distance((*remain_iter)->getPosition());
                if ( dist < closest_dist ){
                    closest = remain_iter;
                    closest_dist = dist;
                }
            }

            // no leg is within a threshold of this candidate, since the leg we thought it was close to was actually closer to another measurement,
            // so create a new leg for this candidate
            if (closest == propagated.end())
                saved_features_.insert(saved_features_.end(), new SavedFeature(loc, &tfl_, fixed_frame_));
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

    // TODO: Put the rest of this callback back to the uncommented state where both legs are required to output to the tracker.
    // Now that saved features have all been updated with laser values, output them!
    int i = 0;
    map<string, SavedFeature*> legs_matched;
    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin(); sf_iter != saved_features_.end(); sf_iter++,i++){
        // reliability and covariance calculationsV
        Stamped<Point> pos = (*sf_iter)->getUpdatedPosition();
        double vel_mag = (*sf_iter)->getVelocityMagnitude();
        double reliability = fmin(1.0, fmax(0.2, vel_mag / 0.5)); // Not used as reliability, just for covariance calculation.
        double covariance = pow(0.3 / reliability,2.0); // TODO: Investigate this calculation a bit more to see if it's reasonable to use it.

        // point cloud output for each leg (will be outputed after this loop once the entire cloud has been constructed)
        filter_visualize[i].x = pos[0];
        filter_visualize[i].y = pos[1];
        filter_visualize[i].z = pos[2];
        weights[i] = *(float*)&(rgb[min(998, max(1, (int)trunc( reliability*999.0 )))]); // TODO: Investigate this calculation more. What is it even for other than Rviz?

        // If there are two leg's with the same ID (both have been associated with the same tracker), output their updated center to the filter.
        // If the current object has no ID, obviously skip over it.
        if ((*sf_iter)->getObjectID() == "")
            continue;
        // If the current leg matches the ID of a leg that has already been seen, output the average of their locations to the filter
        map<string, SavedFeature*>::iterator first_leg;
        if ( (first_leg = legs_matched.find((*sf_iter)->getObjectID())) != legs_matched.end() ){
            // Average posiiton of the first and second leg
            Stamped<Point> first_pos = (first_leg->second)->getUpdatedPosition();
            double first_vel_mag = (first_leg->second)->getVelocityMagnitude();
            double first_reliability = fmin(1.0, fmax(0.2, first_vel_mag/0.5));
            double first_covariance = pow(0.3 / first_reliability, 2.0);

            double average_x = (pos[0] + first_pos[0])/2;
            double average_y = (pos[1] + first_pos[1])/2;
            double average_cov = (covariance + first_covariance)/2;
            double actual_cov = average_cov/2.0; // The covariance should be smaller because two legs are being used to give a single output.

            // output measurement to the filter to update this tracker's position
            people_msgs::PositionMeasurement output;
            output.header.stamp = (*sf_iter)->getTime();
            output.header.frame_id = fixed_frame_;
            output.name = "leg_detector";
            output.object_id = (*sf_iter)->getObjectID();
            output.pos.x = average_x;
            output.pos.y = average_y;
            output.pos.z = 0.0;
            output.reliability = 0.5; // Calculated reliability is given up and replaced with hard coded value to ensure that leg_detector cannot initiate a new tracker in the filter.
            output.covariance[0] = actual_cov; // Doing the above comment causes only face detector to be able to create a new tracker, and that's all it's used for.
            output.covariance[1] = 0.0;
            output.covariance[2] = 0.0;
            output.covariance[3] = 0.0;
            output.covariance[4] = actual_cov; 
            output.covariance[5] = 0.0;
            output.covariance[6] = 0.0;
            output.covariance[7] = 0.0;
            output.covariance[8] = 10000.0; // Variance for the z component is really high (unreliable) because we don't care about it, so we just put it to 0 before.
            output.initialization = 0; // initialization=1 is required to instantiate new trackers in the filter, so leg_detector is never allowed to do so because of this.
        
            measurements_pub_.publish(output);
            ROS_INFO_STREAM(boost::format("(in laser callback) Publishing new measurement for \"%s\", covariance=%.2f")%(*sf_iter)->getObjectID() %actual_cov );

            // Delete this ID from legs_matched so that it isn't accidentally republished
            legs_matched.erase( first_leg );
        }
        // Otherwise, add this leg to the map and keep looking
        else
            legs_matched.insert( pair<string, SavedFeature*>((*sf_iter)->getObjectID(), *sf_iter) );
    }

    lock.unlock();
    // ----------- Locked ----------

    // visualize all legs as points in a point cloud based on the center of each leg.
    channel.name = "rgb";
    channel.values = weights;
    sensor_msgs::PointCloud  people_cloud; 
    people_cloud.channels.push_back(channel);
    people_cloud.header.frame_id = fixed_frame_;
    people_cloud.header.stamp = scan->header.stamp;
    people_cloud.points  = filter_visualize;
    leg_cloud_pub_.publish(people_cloud);

} // End laser callback
