#include "Heuristics.h"

#include <boost/make_shared.hpp>

int kUnknownDist = -1;

Heuristics::Heuristics()
{
}

bool
Heuristics::init(const StateGrid& stateGrid)
{
    double r = stateGrid.resolution();

    m_grid = boost::make_shared<Grid2D<HeuristicCell> >(stateGrid.resolution(), stateGrid.rows(), stateGrid.cols());

    m_grid->mapRowStart() = stateGrid.mapRowStart();
    m_grid->mapColStart() = stateGrid.mapColStart();
    m_grid->arrayRowStart() = stateGrid.arrayRowStart();
    m_grid->arrayColStart() = stateGrid.arrayColStart();

    // one-time memory allocation - avoid future reallocation which is expensive!
    m_bucket[0].reserve(m_grid->rows() * m_grid->cols());
    m_bucket[1].reserve(m_grid->rows() * m_grid->cols());

    m_successors.clear();
    m_successors.push_back(Eigen::Vector2i(-1,  0));
    m_successors.push_back(Eigen::Vector2i( 1,  0));
    m_successors.push_back(Eigen::Vector2i( 0, -1));
    m_successors.push_back(Eigen::Vector2i( 0,  1));

    return true;
}

bool
Heuristics::compute(const StatePtr& start, const StatePtr& goal)
{
    int xStart = start->x();
    int yStart = start->y();
    int xGoal = goal->x();
    int yGoal = goal->y();

    // initialize grid
    for (int r = 0; r < m_grid->rows(); ++r)
    {
        for (int c = 0; c < m_grid->cols(); ++c)
        {
            HeuristicCell* cell = m_grid->getLocalRC(r,c);
            cell->distance = -1;
        }
    }


    m_start = start;
    m_goal = goal;

    m_grid->getMapRC(yStart, xStart)->distance = 0;

    populateHeuristicMap(Eigen::Vector2i(xStart, yStart));

    if (m_grid->getMapRC(yGoal, xGoal)->distance != kUnknownDist)
    {
        return true;
    }
    else
    {
        return false;
    }
}

float
Heuristics::minPathCost(const StatePtr& s) const
{
    if (m_grid->getMapRC(s->y(), s->x()) &&
        m_grid->getMapRC(s->y(), s->x())->distance != kUnknownDist)
    {
        return m_grid->getMapRC(s->y(), s->x())->distance * m_grid->resolution();
    }
}

void
Heuristics::populateHeuristicMap(const Eigen::Vector2i& startPoint)
{
    m_bucket[0].clear();
    m_bucket[0].push_back(startPoint);
    m_bucket[1].clear();

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> >* bucket0 = &(m_bucket[0]);
    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> >* bucket1 = &(m_bucket[1]);
    while (!bucket0->empty() || !bucket1->empty())
    {
        if (bucket0->empty())
        {
            std::swap(bucket0, bucket1);
        }

        int x = bucket0->back()(0);
        int y = bucket0->back()(1);

        int distNeighbor = m_grid->getMapRC(y,x)->distance + 1;

        bucket0->pop_back();

        for (size_t i = 0; i < m_successors.size(); ++i)
        {
            int xNeighbor = x + m_successors.at(i)(0);
            int yNeighbor = y + m_successors.at(i)(1);

            HeuristicCell* cellNeighbor = m_grid->getMapRC(yNeighbor, xNeighbor);
            if (cellNeighbor && cellNeighbor->distance == kUnknownDist)
            {
                if(!cellNeighbor->obstacle)
                {
                    bucket1->push_back(Eigen::Vector2i(xNeighbor, yNeighbor));
                
                    cellNeighbor->distance = distNeighbor;
                }
                else
                {
                    cellNeighbor->distance = 100000;
                }
            }
        }
    }
}


