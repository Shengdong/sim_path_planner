#include "LatticePose.h"

LatticePose::LatticePose()
 : m_position(Eigen::Vector3i::Zero())
 , m_attitude(Eigen::Matrix<int,6,1>::Zero())
{

}

LatticePose::LatticePose(const Eigen::Vector3i& position,
                         const Eigen::Vector2i& yaw)
 : m_position(position)
{
    m_attitude << 0, 0, 0, 0, yaw(0), yaw(1);
}

LatticePose::LatticePose(const LatticePose& p)
 : m_position(p.m_position)
 , m_attitude(p.m_attitude)
{

}

void
LatticePose::set(const Eigen::Vector3i& position,
                 const Eigen::Vector2i& yaw)
{
    m_position = position;
    m_attitude(4) = yaw(0);
    m_attitude(5) = yaw(1);
}

Eigen::Vector3i&
LatticePose::position(void)
{
    return m_position;
}

int&
LatticePose::x(void)
{
    return m_position(0);
}

int
LatticePose::x(void) const
{
    return m_position(0);
}

int&
LatticePose::y(void)
{
    return m_position(1);
}

int
LatticePose::y(void) const
{
    return m_position(1);
}

int&
LatticePose::z(void)
{
    return m_position(2);
}

int
LatticePose::z(void) const
{
    return m_position(2);
}

const Eigen::Vector3i&
LatticePose::position(void) const
{
    return m_position;
}

int&
LatticePose::xYaw(void)
{
    return m_attitude(4);
}

int
LatticePose::xYaw(void) const
{
    return m_attitude(4);
}

int&
LatticePose::yYaw(void)
{
    return m_attitude(5);
}

int
LatticePose::yYaw(void) const
{
    return m_attitude(5);
}

bool
LatticePose::operator==(const LatticePose& other) const
{
    return (m_position(0) == other.m_position(0) &&
            m_position(1) == other.m_position(1) &&
            xYaw() == other.xYaw() &&
            yYaw() == other.yYaw());
}

std::string
LatticePose::print(void) const
{
    using namespace std;

    ostringstream oss(ostringstream::out);
    oss << "(" << m_position(0) << " " << m_position(1) << " " << xYaw() << " " << yYaw() << ")";

    return oss.str();
}

