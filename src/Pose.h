#ifndef LATTICE_PLANNER_POSE_H
#define LATTICE_PLANNER_POSE_H

#include "LatticePose.h"
#include "OrientationLUT.h"


class Pose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose();
    Pose(const Eigen::Vector3d& position,
         double roll, double pitch, double yaw);
    Pose(const Pose& p);

    void set(const Eigen::Vector3d& position,
             double roll, double pitch, double yaw);

    Eigen::Vector3d& position(void);
    const Eigen::Vector3d& position(void) const;

    double& yaw(void);
    double yaw(void) const;

    LatticePose discretize(double resolution,
                           const OrientationLUT& orientationLUT) const;

private:
    Eigen::Vector3d m_position;
    Eigen::Vector3d m_attitude;
};

typedef boost::shared_ptr<Pose> PosePtr;
typedef boost::shared_ptr<const Pose> PoseConstPtr;



#endif
