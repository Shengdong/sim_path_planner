#ifndef LATTICE_PLANNER_STATELATTICE_H
#define LATTICE_PLANNER_STATELATTICE_H

#include "OrientationLUT.h"
class StateLattice
{
public:
    StateLattice();

    void init(double resolution);

    double resolution(void) const;
    const std::vector<std::vector<PrimitivePathPtr> >& primitivePathSets(void) const;
    const OrientationLUT& orientationLUT(void) const;

private:
    void generatePrimitivePathSets(void);
    void generateOrientationLUT(void);

    double m_resolution;

    // m_primitivePathSet.at(i) contains all possible paths for orientation i
    std::vector<std::vector<PrimitivePathPtr> > m_primitivePathSets;
    OrientationLUT m_orientationLUT;
};



#endif
