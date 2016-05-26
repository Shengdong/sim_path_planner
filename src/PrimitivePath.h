#ifndef LATTICE_PLANNER_PRIMITIVEPATH_H
#define LATTICE_PLANNER_PRIMITIVEPATH_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <Eigen/Dense>
#include <vector>

#include "LatticePose.h"



class PrimitivePath
{
public:
    PrimitivePath(double resolution);

    void addWaypoint(const LatticePose& wp);
    std::vector<LatticePose, Eigen::aligned_allocator<LatticePose> >& waypoints(void);

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> >& swath(void);

    double length(void) const;

    bool reverse(void) const;

    std::string print(void) const;

    PrimitivePath& operator=(const PrimitivePath& other);

private:
    std::vector<LatticePose, Eigen::aligned_allocator<LatticePose> > m_waypoints;
    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > m_swath;

    double m_resolution;
    double m_length;

    bool m_reverse;
};

typedef boost::shared_ptr<PrimitivePath> PrimitivePathPtr;
typedef boost::weak_ptr<PrimitivePath> PrimitivePathWPtr;


#endif
