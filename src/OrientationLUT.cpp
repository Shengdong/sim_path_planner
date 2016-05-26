#include "OrientationLUT.h"
#include <map>
//#include "cauldron/cauldron.h"

void
OrientationLUT::build(const std::vector< std::vector<PrimitivePathPtr> >& primitivePathSets)
{
    assert(!primitivePathSets.empty());

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > orientations;
    int xMax = -1, yMax = -1;
    for (size_t i = 0; i < primitivePathSets.size(); ++i)
    {
        assert(!primitivePathSets.at(i).empty());

        LatticePose& p = primitivePathSets.at(i).at(0)->waypoints().front();

        orientations.push_back(Eigen::Vector2i(p.xYaw(), p.yYaw()));
        if (abs(p.xYaw()) > xMax)
        {
            xMax = abs(p.xYaw());
        }
        if (abs(p.yYaw()) > yMax)
        {
            yMax = abs(p.yYaw());
        }
    }

    m_lut = Eigen::MatrixXi::Zero(yMax * 2 + 1, xMax * 2 + 1);
    m_center << xMax, yMax;

    int idx = 1;
    for (size_t i = 0; i < orientations.size(); ++i)
    {
        int x = orientations.at(i)(0) + m_center(0);
        int y = orientations.at(i)(1) + m_center(1);

        if (m_lut(y,x) == 0)
        {
            m_lut(y,x) = idx;
            ++idx;
        }
    }
}

Eigen::Vector2i
OrientationLUT::lookupDiscreteValue(double yaw) const
{
    double errorMin = M_PI * 2.0;
    Eigen::Vector2i orientation;

    for (int r = 0; r < m_lut.rows(); ++r)
    {
        for (int c = 0; c < m_lut.cols(); ++c)
        {
            if (m_lut(r,c) > 0)
            {
                double error = fabs(atan2(r - m_center(1), c - m_center(0)) - yaw);
                if (error < errorMin)
                {
                    errorMin = error;
                    orientation = Eigen::Vector2i(c - m_center(0), r - m_center(1));
                }
            }
        }
    }

    return orientation;
}

int
OrientationLUT::lookupIndex(const Eigen::Vector2i& orientation) const
{
    return lookupIndex(orientation(0), orientation(1));
}

int
OrientationLUT::lookupIndex(int xOrientation, int yOrientation) const
{
    return m_lut(yOrientation + m_center(1), xOrientation + m_center(0));
}

std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> >
OrientationLUT::availableOrientations(void) const
{
    std::map<int, Eigen::Vector2i, std::less<int>,
             Eigen::aligned_allocator<std::pair<const int, Eigen::Vector2i> > > orientations;

    for (int r = 0; r < m_lut.rows(); ++r)
    {
        for (int c = 0; c < m_lut.cols(); ++c)
        {
            if (m_lut(r,c) > 0)
            {
                orientations[m_lut(r,c)] = Eigen::Vector2i(c - m_center(0), r - m_center(1));
            }
        }
    }

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > ret;
    for (std::map<int, Eigen::Vector2i, std::less<int>,
                  Eigen::aligned_allocator<std::pair<const int, Eigen::Vector2i> > >::iterator it = orientations.begin();
         it != orientations.end(); ++it)
    {
        ret.push_back(it->second);
    }

    return ret;
}

int
OrientationLUT::size(void) const
{
    int nonZeroCount = 0;

    for (int r = 0; r < m_lut.rows(); ++r)
    {
        for (int c = 0; c < m_lut.cols(); ++c)
        {
            if (m_lut(r,c) > 0)
            {
                ++nonZeroCount;
            }
        }
    }

    return nonZeroCount;
}

