#ifndef STATEGRID_H
#define STATEGRID_H

#include <boost/unordered_map.hpp>

#include "Grid2D.h"
#include "State.h"
#include "OrientationLUT.h"


class StateCell
{
public:
    StateCell()
    {

    }

    void insert(const StatePtr& s)
    {
        m_stateVector.push_back(s);
    }

    bool find(const StatePtr& s) const
    {
        for (size_t i = 0; i < m_stateVector.size(); ++i)
        {
            if (m_stateVector.at(i) == s)
            {
                return true;
            }
        }

        return false;
    }

    std::vector<StatePtr>& stateVector(void)
    {
        return m_stateVector;
    }

    friend class StateGrid;

private:
    std::vector<StatePtr> m_stateVector;
};

class StateGrid : public Grid2D<StateCell>
{
public:
    StateGrid(double width, double resolution,
              const OrientationLUT& orientationLUT);

    void precompute(double z,
                    boost::unordered_map<int, std::vector<PrimitivePathPtr> >& lutPredecessors,
                    boost::unordered_map<int, std::vector<PrimitivePathPtr> >& lutSuccessors,
                    float reversePenaltyFactor = 2.0f);

    bool isVisited(const LatticePose& p) const;
    bool outsideBounds(const LatticePose& p) const;

    StatePtr& at(const LatticePose& s);

    std::vector<StatePtr> containedStates(void) const;

private:
    OrientationLUT m_orientationLUT;
};


#endif
