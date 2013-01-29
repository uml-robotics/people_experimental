// saved_feature.h

#ifndef _SAVED_FEATURE_H
#define _SAVED_FEATURE_H

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "opencv/cxcore.h" // TODO: Are each of these necessary?
#include "opencv/cv.h"
#include "opencv/ml.h"

// non-standard ROS includes
#include "people_tracking_filter/tracker_kalman.h"
#include "leg_detector/laser_processor.h"

// ROS messages
#include "std_msgs/Header.h"

// c++ includes
#include <boost/format.hpp>
#include <string> //TODO: Necessary?

namespace saved_feature
{
    // A kalman filter to track the location of each individual leg found
    class SavedFeature
    {
    public:
        // Counter to give unique ID's to each new leg.
        static int next_id;

        // Constructor
        SavedFeature(tf::Stamped<tf::Point> , tf::TransformListener *, std::string );

        // Propagate; Updates filter's prediction of location to the given time
        void propagate(const ros::Time);

        // Update; Adds the calculated position to the filter for this leg
        void addMeasurement(const tf::Stamped<tf::Point>);

        // Getters
        double getLifetime() const;
        ros::Time getTime() const;
        ros::Time getMeasTime() const;
        double getDist() const;
        std::string getID() const;
        std::string getObjectID() const;
        tf::Stamped<tf::Point> getPosition() const;

        // Mutators for object_ID and distance to tracker
        void associateTracker( std::string );
        void disassociateTracker();
        void setDist( float );

    private:
        tf::TransformListener* tfl_; // Transform listener to set up and update a transform to this leg's frame. TODO Is this even used? It may be extra overhead
        std::string fixed_frame_;     // Used to transform all input points to this frame 

        BFL::StatePosVel sys_sigma_; // Variance of the system
        estimation::TrackerKalman filter_; // Kalman filter that does all of the work

        std::string id_;     // Unique identifier TODO: Necessary?
        std::string object_id_;    // ID of the associated tracker. "" if none.
        ros::Time init_time_; // Time of initialization
        ros::Time meas_time_; // Time of last input measurement
        ros::Time time_;      // Time of last filter update

        tf::Stamped<tf::Point> position_; // Latest position estimated by the filter
        float dist_to_tracker_;   // Distance from it's associated tracker since the last update.

        void updatePosition(); // Private function that gets the latest estimate from the filter. 

    }; // End SavedFeature class

    // MatchedFeature is a class temporarily used to store a cluster of points that look like a leg with a saved leg that it is closest to
    class MatchedFeature
    {
    public:
        laser_processor::SampleSet* candidate_;
        SavedFeature* closest_;
        float distance_;    

        // Constructor
        MatchedFeature( laser_processor::SampleSet* candidate, SavedFeature* closest, float distance )
            : candidate_( candidate ),
            closest_( closest ),
            distance_( distance )
        {}

        // Comparison operator
        inline bool operator< (const MatchedFeature& b) const
        {
            return distance_ < b.distance_;
        }
    }; // End MathcedFeature class
}; // End saved_feature namespace
#endif
