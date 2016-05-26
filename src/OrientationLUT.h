#ifndef LATTICE_PLANNER_ORIENTATIONLUT_H
#define LATTICE_PLANNER_ORIENTATIONLUT_H

#include <Eigen/Dense>

#include "PrimitivePath.h"



class OrientationLUT
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void build(const std::vector< std::vector<PrimitivePathPtr> >& primitivePathSets);
    Eigen::Vector2i lookupDiscreteValue(double yaw) const;
    int lookupIndex(const Eigen::Vector2i& orientation) const;
    int lookupIndex(int xOrientation, int yOrientation) const;

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > availableOrientations(void) const;
    int size(void) const;

private:
    Eigen::MatrixXi m_lut;
    Eigen::Vector2i m_center;
};



#endif
