#pragma once

namespace colmag_ros
{

// Application-level envelope for the supervised visible-motion demo. This is
// intentionally far below the FR3 mechanical joint range and is enforced by
// both trajectory generators before a FollowJointTrajectory goal can exist.
constexpr double kVisibleMotionHardLimitRad = 0.12;

}  // namespace colmag_ros
