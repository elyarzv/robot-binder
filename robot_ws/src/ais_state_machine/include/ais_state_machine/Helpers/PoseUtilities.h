/*
    AIS CONFIDENTIAL
*/

#ifndef POSE_UTILITIES_H_
#define POSE_UTILITIES_H_

#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



using std::pow;

namespace ais_state_machine
{
/**
 * @brief This class is meant to provide a simple utility class for calculating movements in the
 *          3D space. Ideally should be ported to its own library at some point. 
 * 
 */

class PoseUtilities
{
    public:
        PoseUtilities();
        ~PoseUtilities();
        // TODO Template below function for later
        float getDistanceBetweenPoses(geometry_msgs::Pose, geometry_msgs::Pose);
        tf2::Transform getTransformFromOriginToPose(geometry_msgs::Pose);
        tf2::Transform getTransformFromPoseToOrigin(geometry_msgs::Pose);
        

};
} // end namespace ais_state_machine 
#endif // POSE_UTILITIES_H_