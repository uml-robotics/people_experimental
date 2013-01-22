// matched_feature.h

#ifndef _MATCHED_FEATURE_H
#define _MATCHED_FEATURE_H

// non-standard ROS includes
#include "saved_feature.h"
#include "laser_processor.h"

// MatchedFeature is a class temporarily used to store a cluster of points that look like a leg with a saved leg that it is closest to
class MatchedFeature
{
public:
    SampleSet* candidate_;
    SavedFeature* closest_;
    float distance_;    

    // Constructor
    MatchedFeature( SampleSet* candidate, SavedFeature* closest, float distance )
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

#endif
