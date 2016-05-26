#include "Pose.h"



Pose::Pose()
 : m_position(Eigen::Vector3d::Zero())
 , m_attitude(Eigen::Vector3d::Zero())
{

}

Pose::Pose(const Eigen::Vector3d& position,
           double roll, double pitch, double yaw)
 : m_position(position)
 , m_attitude(0.0, 0.0, yaw)
{

}

Pose::Pose(const Pose& p)
 : m_position(p.m_position)
 , m_attitude(p.m_attitude)
{

}

void
Pose::set(const Eigen::Vector3d& position,
          double roll, double pitch, double yaw)
{
    m_position = position;
    m_attitude = Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Vector3d&
Pose::position(void)
{
    return m_position;
}

const Eigen::Vector3d&
Pose::position(void) const
{
    return m_position;
}

double&
Pose::yaw(void)
{
    return m_attitude(2);
}

double
Pose::yaw(void) const
{
    return m_attitude(2);
}

LatticePose
Pose::discretize(double resolution,
                 const OrientationLUT& orientationLUT) const
{
    Eigen::Vector2i orientation = orientationLUT.lookupDiscreteValue(m_attitude(2));

    assert(orientation(0) != 0 || orientation(1) != 0);

    Eigen::Vector3i latticePos;
    latticePos << floor(m_position(0) / resolution),
                  floor(m_position(1) / resolution),
                  floor(m_position(2) / resolution);

    return LatticePose(latticePos, orientation);
}


