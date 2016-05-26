#include "PrimitivePath.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>


PrimitivePath::PrimitivePath(double resolution)
 : m_resolution(resolution)
 , m_length(0.0)
 , m_reverse(false)
{

}

void
PrimitivePath::addWaypoint(const LatticePose& wp)
{
    m_waypoints.push_back(wp);

    if (m_waypoints.size() > 1)
    {
        size_t prevIdx = m_waypoints.size() - 2;

        double segmentLength =
            hypot(m_waypoints.back().x() - m_waypoints.at(prevIdx).x(),
                  m_waypoints.back().y() - m_waypoints.at(prevIdx).y());
        segmentLength *= m_resolution;
        m_length += segmentLength;
    }

    if (m_waypoints.size() == 2)
    {
        // check if path is reverse
        Eigen::Vector2i v0(m_waypoints.at(1).xYaw(), m_waypoints.at(1).yYaw());
        Eigen::Vector2i v1 = (m_waypoints.at(1).position() - m_waypoints.at(0).position()).topRows(2);
        if (v0.dot(v1) >= 0)
        {
            m_reverse = false;
        }
        else
        {
            m_reverse = true;
        }
    }
}

std::vector<LatticePose, Eigen::aligned_allocator<LatticePose> >&
PrimitivePath::waypoints(void)
{
    return m_waypoints;
}

std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> >&
PrimitivePath::swath(void)
{
    return m_swath;
}

double
PrimitivePath::length(void) const
{
    return m_length;
}

bool
PrimitivePath::reverse(void) const
{
    return m_reverse;
}

std::string
PrimitivePath::print(void) const
{
    using namespace std;

    ostringstream oss(ostringstream::out);
    oss << fixed << setprecision(3);

    oss << m_waypoints.front().print() << " -> ";
    oss << m_waypoints.back().print();

    return oss.str();
}

PrimitivePath&
PrimitivePath::operator=(const PrimitivePath& other)
{
    if (this != &other)
    {
        m_waypoints = other.m_waypoints;
        m_swath = other.m_swath;

        m_resolution = other.m_resolution;
        m_length = other.m_length;
    }

    return *this;
}


