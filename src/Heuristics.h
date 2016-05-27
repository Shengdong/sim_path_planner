#ifndef LATTICE_PLANNER_HEURISTICS_H
#define LATTICE_PLANNER HEURISTICS_H

#include "StateGrid.h"

class Heuristics
{
public:
    explicit Heuristics();

    bool init(const StateGrid& stateGrid);

    bool compute(const StatePtr& start, const StatePtr& goal);

    float minPathCost(const StatePtr& s) const;

private:
    void populateHeuristicMap(const Eigen::Vector2i& startPoint);

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > m_successors;
    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > m_bucket[2];

    class HeuristicCell
    {
    public:
        HeuristicCell()
         : distance(-1)
         , obstacle(false)
        {}

        int distance;
        bool obstacle;
    };

    boost::shared_ptr<Grid2D<HeuristicCell> > m_grid;

    StatePtr m_start;
    StatePtr m_goal;
};


#endif
