#include "StateGrid.h"

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <ros/ros.h>


StateGrid::StateGrid(double width, double resolution,
                     const OrientationLUT& orientationLUT)
 : Grid2D<StateCell>(resolution, width / resolution, width / resolution)
 , m_orientationLUT(orientationLUT)
{

}

void
StateGrid::precompute(double z,
                      boost::unordered_map<int, std::vector<PrimitivePathPtr> >& lutPredecessors,
                      boost::unordered_map<int, std::vector<PrimitivePathPtr> >& lutSuccessors,
                      float reversePenaltyFactor)
{
    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > orientations = m_orientationLUT.availableOrientations();

    // initialize state grid
    for (int r = mapRowStart(); r < mapRowStart() + rows(); ++r)
    {
        for (int c = mapColStart(); c < mapColStart() + cols(); ++c)
        {
            StateCell* cell = getMapRC(r,c);

            cell->m_stateVector.resize(orientations.size());
            for (size_t i = 0; i < cell->m_stateVector.size(); ++i)
            {
                Eigen::Vector2i& orientation = orientations.at(i);

                // create state
                LatticePose p(Eigen::Vector3i(c, r, z), orientation);
                cell->m_stateVector.at(i) = boost::make_shared<State>(p);
            }
        }
    }

    int cellCount = rows() * cols();

    size_t nEdges = 0;

    // find predecessor/successor states for each state
    for (size_t orientationIdx = 1; orientationIdx <= orientations.size(); ++orientationIdx)
    {
        const std::vector<PrimitivePathPtr>& pathsPred = lutPredecessors[orientationIdx];
        const std::vector<PrimitivePathPtr>& pathsSucc = lutSuccessors[orientationIdx];

        StateCell* cell = getRawCopy();

        for (int i = 0; i < cellCount; ++i)
        {
            StatePtr& s = cell->m_stateVector.at(orientationIdx - 1);

            // find predecessor states
            s->incomingEdges().reserve(pathsPred.size());
            BOOST_FOREACH(const PrimitivePathPtr& path, pathsPred)
            {
                const LatticePose& p0 = path->waypoints().front();

                LatticePose p(s->position() + p0.position(),
                              Eigen::Vector2i(p0.xYaw(), p0.yYaw()));

                if (outsideBounds(p))
                {
                    continue;
                }

                StatePtr neighbor = at(p);

                EdgePtr edge;
                BOOST_FOREACH(EdgePtr& e, neighbor->outgoingEdges())
                {
                    if (e->stateIn() == s)
                    {
                        edge = e;
                        break;
                    }
                }

                if (!edge)
                {
                    edge = boost::make_shared<Edge>();
                    edge->traversalCost() = path->length();
                    if (path->reverse())
                    {
                        edge->traversalCost() *= reversePenaltyFactor; // add penalty for reverse
                    }
                    edge->stateOut() = neighbor;
                    edge->stateIn() = s;
                    edge->path() = path;

                    ++nEdges;
                }

                s->incomingEdges().push_back(edge);
            }

            // find successor states
            s->outgoingEdges().reserve(pathsSucc.size());
            BOOST_FOREACH(const PrimitivePathPtr& path, pathsSucc)
            {
                const LatticePose& pn = path->waypoints().back();

                LatticePose p(s->position() + pn.position(),
                              Eigen::Vector2i(pn.xYaw(), pn.yYaw()));

                if (outsideBounds(p))
                {
                    continue;
                }

                StatePtr neighbor = at(p);

                EdgePtr edge;
                BOOST_FOREACH(EdgePtr& e, neighbor->incomingEdges())
                {
                    if (e->stateOut() == s)
                    {
                        edge = e;
                        break;
                    }
                }

                if (!edge)
                {
                    edge = boost::make_shared<Edge>();
                    edge->traversalCost() = path->length();
                    if (path->reverse())
                    {
                        edge->traversalCost() *= reversePenaltyFactor; // add penalty for reverse
                    }
                    edge->stateOut() = s;
                    edge->stateIn() = neighbor;
                    edge->path() = path;

                    ++nEdges;
                }

                s->outgoingEdges().push_back(edge);
            }

            ++cell;
        }
    }

    ROS_INFO("Created a %d x %d state grid with resolution %.2f m.",
             cols(), rows(), resolution());
    ROS_INFO("Created %zu edges.", nEdges);
}

bool
StateGrid::isVisited(const LatticePose& p) const
{
    const StateCell* cell = getMapRC(p.y(), p.x());

    int orientationIdx = m_orientationLUT.lookupIndex(p.xYaw(), p.yYaw());

    return (cell->m_stateVector.at(orientationIdx - 1)->initialized());
}

bool
StateGrid::outsideBounds(const LatticePose& p) const
{
    return (getMapRC(p.y(), p.x()) == NULL);
}

StatePtr&
StateGrid::at(const LatticePose& p)
{
    StateCell* cell = getMapRC(p.y(), p.x());

    int orientationIdx = m_orientationLUT.lookupIndex(p.xYaw(), p.yYaw());

    return cell->m_stateVector.at(orientationIdx - 1);
}

std::vector<StatePtr>
StateGrid::containedStates(void) const
{
    std::vector<StatePtr> stateVec;

    for (int r = 0; r < rows(); ++r)
    {
        for (int c = 0; c < cols(); ++c)
        {
            const StateCell* cell = getLocalRC(r, c);
            if (cell != NULL)
            {
                for (size_t i = 0; i < cell->m_stateVector.size(); ++i)
                {
                    const StatePtr& s = cell->m_stateVector.at(i);
                    if (s->initialized())
                    {
                        stateVec.push_back(s);
                    }
                }
            }
        }
    }

    return stateVec;
}


