// saved_feature.cpp

#include "leg_detector/saved_feature.h"

using namespace saved_feature;
using namespace tf;
using namespace std;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

// Constructor
SavedFeature::SavedFeature(Stamped<Point> pos, TransformListener *tfl, string fixed_frame )
    : tfl_( tfl ),
    fixed_frame_( fixed_frame ),
    sys_sigma_( Vector3(0.05,0.05,0.05), Vector3(1.0,1.0,1.0) ),
    filter_( (boost::format("Leg %s")%next_id).str(), sys_sigma_ ),
    object_id_( "" ),
    init_time_( pos.stamp_ ),
    time_( pos.stamp_ ),
    dist_to_tracker_( 10000.0 ) // Arbitrary large number
{
    // Create unique ID
    id_ = (boost::format("Leg %d")%next_id++).str();

    // Transform pos to fixed frame and assign it to position
    try {
        tfl_->transformPoint( fixed_frame_, pos, pos );
    }
    catch ( TransformException e ) {
        ROS_WARN_STREAM( "(in SavedFeature constructor) Error transforming input to fixed_frame (1) - " << e.what() );
    }
    position_ = Stamped<Point>( pos );

    // Create this leg's transform frame
    StampedTransform pose( Transform( Quaternion(0.0,0.0,0.0,1.0), pos ), pos.stamp_, id_, pos.frame_id_ );
    tfl_->setTransform( pose );

    StatePosVel prior_sigma( Vector3(0.1,0.1,0.1), Vector3(0.0000001,0.0000001,0.0000001) );
    filter_.initialize( pos, prior_sigma, pos.stamp_.toSec() );

    updatePosition();
} // End constructor


// Propagate; Update filter's prediction of location to the given time
void SavedFeature::propagate(const ros::Time time)
{
    time_ = time;
    filter_.updatePrediction( time.toSec() );
    updatePosition();
} // End propagate


// Update; Adds the new measurement to the filter for this leg.
void SavedFeature::addMeasurement(const Stamped<Point> pos)
{
    // Update the transform frame
    StampedTransform pose( Transform( Quaternion(0.0,0.0,0.0,1.0), pos ), pos.stamp_, id_, pos.frame_id_ );
    tfl_->setTransform( pose );

    // Update times
    meas_time_ = pos.stamp_;
    time_ = pos.stamp_;

    // Construct covariance matrix and send measurement to the filter
    SymmetricMatrix cov(3);
    cov = 0.0;
    cov(1,1) = 0.0025;
    cov(2,2) = 0.0025;
    cov(3,3) = 0.0025;
    filter_.updateCorrection( pos, cov );

    updatePosition();
} // End add_measurement


// Returns number of seconds that this leg has been alive
double SavedFeature::getLifetime() const
{
    return (time_ - init_time_).toSec();
} // end getLifetime


// Returns time of last filter update
ros::Time SavedFeature::getTime() const
{
    return time_;
} // end getter

// Returns time of last measurement update
ros::Time SavedFeature::getMeasTime() const
{
    return meas_time_;
} // end getter


// Returns distance to the assosciated tracker
double SavedFeature::getDist() const
{
    if ( object_id_ != "" )
        return dist_to_tracker_;
    return 10000.0;
} // end getter

// Returns associated leg ID
string SavedFeature::getID() const
{
    return id_;
} // end getter

// Returns associated object ID
string SavedFeature::getObjectID() const
{
    return object_id_;
} // end getter


// Returns latest position
Stamped<Point> SavedFeature::getPosition() const
{
    return position_;
} // end getter


// Associates a leg to a tracker
void SavedFeature::associateTracker( string object_id )
{
    // Set new object ID
    object_id_ = object_id;

} // End associateTracker


// Disassociates a leg from any tracker it is currently associated to
void SavedFeature::disassociateTracker()
{
    // Clear object ID and set distance to a high, arbitrary number
    object_id_ = "";
    dist_to_tracker_ = 10000.0;
} // End disassociateTracker


// Set the distance from the leg to it's associated tracker
void SavedFeature::setDist( float dist )
{
    dist_to_tracker_ = dist;
}


// Get the latest estimate from the filter and store it in position
void SavedFeature::updatePosition()
{
    // Get current estimate
    StatePosVel est;
    filter_.getEstimate( est );

    // Store in position
    position_[0] = est.pos_[0];
    position_[1] = est.pos_[1];
    position_[2] = est.pos_[2];
    position_.stamp_ = time_;
    position_.frame_id_ = fixed_frame_;
} // End updatePosition
