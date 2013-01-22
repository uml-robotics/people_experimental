// saved_feature.h

#ifndef _SAVED_FEATURE_H
#define _SAVED_FEATURE_H

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// non-standard ROS includes
#include "people_tracking_filter/tracker_kalman.h"

// ROS messages
#include "std_msgs/Header.h"

// c++ includes
#include <boost/format.hpp>
#include <string>

using namespace tf;
using namespace std;
using namespace BFL;
using namespace estimation;
using namespace MatrixWrapper;

// A kalman filter to track the location of each individual leg found
class SavedFeature
{
public:
    // Counter to give unique ID's to each new leg.
    static int next_id;

    // Constructor
    SavedFeature(Stamped<Point> , TransformListener *, string );

    // Propagate; Updates filter's prediction of location to the given time
    void propagate(const ros::Time);

    // Update; Adds the calculated position to the filter for this leg
    void addMeasurement(const Stamped<Point>);

    // Getters
    double getLifetime() const;
    double getTime() const;
    double getDist() const;
    string getObjectID() const;
    Stamped<Point> getPosition() const;

    // Mutators for object_ID and distance to tracker
    void associateTracker( string, float );
    void disassociateTracker();

private:
    TransformListener* tfl_; // Transform listener to set up and update a transform to this leg's frame. TODO Is this even used? It may be extra overhead
    string fixed_frame_;     // Used to transform all input points to this frame 

    StatePosVel sys_sigma_; // Variance of the system
    TrackerKalman filter_;       // Kalman filter that does all of the work

    string id_;     // Unique identifier TODO: Necessary?
    string object_id_;    // ID of the associated tracker. "" if none.
    ros::Time init_time_; // Time of initialization
    ros::Time meas_time_; // Time of last input measurement
    ros::Time time_;      // Time of last filter update

    Stamped<Point> position_; // Latest position estimated by the filter
    float dist_to_tracker_;   // Distance from it's associated tracker since the last update.

    void updatePosition(); // Private function that gets the latest estimate from the filter. 

}; // End SavedFeature class

#endif
