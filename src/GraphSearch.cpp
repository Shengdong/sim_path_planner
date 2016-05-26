#include "GraphSearch.h"

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <iomanip>
#include <ros/ros.h>
#include <math.h>
#include <iostream>

const double FLOAT_MAX = std::numeric_limits<float>::max();

GraphSearch::GraphSearch(ros::NodeHandle& nh,
                         double gridWidth,
                         const boost::shared_ptr<StateLattice>& stateLattice,

                         const Options& options)
 : m_stateGrid(gridWidth, stateLattice->resolution(), stateLattice->orientationLUT())
 , m_primitivePathSets(stateLattice->primitivePathSets())
 , m_resolution(stateLattice->resolution())
 , m_orientationLUT(stateLattice->orientationLUT())
 , m_options(options)
{
    computePredecessorLUT();
    computeSuccessorLUT();

    m_heuristics = boost::make_shared<Heuristics>();

    ros::Time startTime = ros::Time::now();
    ROS_INFO("Initializing state grid...");

    m_stateGrid.recenterMapRC(0, 0);
    m_stateGrid.precompute(0, m_lutPredecessors, m_lutSuccessors);

    ROS_INFO("Done. Process took %.4f s.",
             (ros::Time::now() - startTime).toSec());
}

bool
GraphSearch::init(const Pose& poseStart,
                  const Pose& poseGoal)
{
    reset();

    m_lpStart = poseStart.discretize(m_resolution, m_orientationLUT);
    m_lpGoal = poseGoal.discretize(m_resolution, m_orientationLUT);

    ros::Time startTime = ros::Time::now();

    // initialize state grid - reset state positions and edge obstacle costs
    m_stateGrid.mapRowStart() = m_lpStart.y() - m_stateGrid.rows() / 2;
    m_stateGrid.mapColStart() = m_lpStart.x() - m_stateGrid.cols() / 2;

    for (int r = m_stateGrid.mapRowStart(); r < m_stateGrid.mapRowStart() + m_stateGrid.rows(); ++r)
    {
        for (int c = m_stateGrid.mapColStart(); c < m_stateGrid.mapColStart() + m_stateGrid.cols(); ++c)
        {
            StateCell* cell = m_stateGrid.getMapRC(r, c);

            for (size_t i = 0; i < cell->stateVector().size(); ++i)
            {
                StatePtr& s = cell->stateVector().at(i);

                s->clear();
                s->x() = c;
                s->y() = r;
                s->z() = m_lpStart.z();
                s->g() = FLOAT_MAX;
                s->cost() = FLOAT_MAX;
            }
        }
    }

    ROS_INFO("State grid initialization took %.4f s.",
             (ros::Time::now() - startTime).toSec());

    // initialize heuristics
    m_heuristics->init(m_stateGrid);

    m_startS = m_stateGrid.at(m_lpStart);
    m_goalS = m_stateGrid.at(m_lpGoal);

    ROS_INFO("Initialization took %.4f s.",
             (ros::Time::now() - startTime).toSec());

    return true;
}

bool
GraphSearch::isGoalReached(void) const
{
    if (m_fixedSolution.empty())
    {
        return false;
    }
    return (m_fixedSolution.front() == m_goalS);
}

void
GraphSearch::reset(void)
{
    if (m_startS)
    {
        m_startS.reset();
    }
    if (m_goalS)
    {
        m_goalS.reset();
    }

    m_closedSet.clear();

    m_fixedSolution.clear();
    m_currentSolution.clear();
}



StatePtr
GraphSearch::getGoalState(void) const
{
    return m_goalS;
}

std::vector<StatePtr>
GraphSearch::PathSearch(LatticePose Start, LatticePose Goal)
{
   
    StatePtr m_startS = m_stateGrid.at(Start);
    StatePtr m_goalS  = m_stateGrid.at(Goal);

    m_heuristics->compute(m_goalS, m_startS);

    m_startS->g() = 0;
    std::vector<StatePtr> path;

    m_openPQ.push(m_startS);
    // A* graph search
    StatePtr p;
    double tentativeScore;
    bool tentative_is_better;
    double stepDistance;

    while(!m_openPQ.empty())
    {
        p = m_openPQ.top();

        if (p == m_goalS)
        {
            ROS_INFO("Path find out!! Path length is %.4f", p->cost());
            StatePtr waypoint = p;

            path.push_back(waypoint);  

            while(waypoint != NULL)
            {              
                 printf("Waypoint Pose is (%d, %d, %f)\n", waypoint->x(), waypoint->y(), atan2(waypoint->xYaw(), waypoint->yYaw()));

                 StatePtr N_neighbor = waypoint->previous();
                 waypoint = N_neighbor;
                 if (waypoint)
                 {
                     path.push_back(waypoint);
                 }
            }   
            return path;
        }

        m_openPQ.pop(); 

        p->setclosed();
        m_closedSet.push_back(p);


        for( std::vector<EdgePtr>::iterator itr = p->outgoingEdges().begin() ; itr != p->outgoingEdges().end(); ++itr)
        {
             StatePtr currentState = (*itr)->stateIn();

             float heuristicdistance = m_heuristics->minPathCost(currentState);

             stepDistance = 1;

             tentativeScore = p->g() + stepDistance;

             tentative_is_better = false;

             if(currentState->isunexplored())
             {
                 currentState->setopen();
                 tentative_is_better = true;
             }
             else if(tentativeScore < currentState->g())
             {
                 tentative_is_better = true;
             }
             else
             {
                 tentative_is_better = false;
             }

             if (tentative_is_better)
             {                
                 currentState->setopen();
                 currentState->g() = tentativeScore;
                 currentState->previous() = p;
                 currentState->cost() =  tentativeScore +  heuristicdistance;
                 m_openPQ.push(currentState);
             }             
        }
    }
    ROS_INFO("Path is not found\n");
}


int
GraphSearch::iterate(const geometry_msgs::PoseStamped::ConstPtr& pose,
                     nav_msgs::Path& path)
{
    ros::Time startTime = ros::Time::now();

     if (isGoalReached())
     {
         return STATUS_REACHED_GOAL;
     }


     ROS_INFO("Start Searching for a path from (%.3f %.3f %.3f) to (%.3f %.3f %.3f)...",
                 m_lpStart.x() * m_resolution,
                 m_lpStart.y() * m_resolution,
                 m_lpStart.z() * m_resolution,
                 m_lpGoal.x() * m_resolution,
                 m_lpGoal.y() * m_resolution,
                 m_lpGoal.z() * m_resolution);

        computePath();
        m_lastWaypoint = m_lpStart;

        ros::Time start_extract = ros::Time::now();
        extractPathSolution(pose, path);
        return STATUS_OK;
}

bool
GraphSearch::computePath(void)
{
    while(!m_openPQ.empty())
    {
        m_openPQ.pop();
    }

    m_closedSet.clear();

    m_path.clear();

    m_path = PathSearch(m_lpStart, m_lpGoal);

}

void
GraphSearch::extractPathSolution(const geometry_msgs::PoseStamped::ConstPtr& pose,
                                     nav_msgs::Path& path)
{
    for (size_t i = 0; i < m_path.size(); ++i)
    {
        StatePtr s = m_path.at(i);

        geometry_msgs::PoseStamped p;

        p.header.frame_id = "world";
        p.header.stamp = ros::Time::now();

        double yaw = atan2(s->yYaw(), s->xYaw());
        p.pose.orientation.w = cos(yaw / 2.0);
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = sin(yaw / 2.0);

        p.pose.position.x = (s->x() + 0.5) * m_resolution;
        p.pose.position.y = (s->y() + 0.5) * m_resolution;
        p.pose.position.z = (s->z() + 0.5) * m_resolution;

        path.poses.push_back(p);
    }
}

void
GraphSearch::computePredecessorLUT(void)
{
    BOOST_FOREACH(const std::vector<PrimitivePathPtr>& set, m_primitivePathSets)
    {
        BOOST_FOREACH(const PrimitivePathPtr& path, set)
        {
            PrimitivePathPtr pathPred = boost::make_shared<PrimitivePath>(m_resolution);
            *pathPred = *path;

            const LatticePose& wpn = path->waypoints().back();
            for (size_t i = 0; i < pathPred->waypoints().size(); ++i)
            {
                LatticePose& wp = pathPred->waypoints().at(i);

                wp.x() -= wpn.x();
                wp.y() -= wpn.y();
                wp.z() -= wpn.z();
            }

            int orientationIdx = m_orientationLUT.lookupIndex(Eigen::Vector2i(wpn.xYaw(), wpn.yYaw()));
            m_lutPredecessors[orientationIdx].push_back(pathPred);
        }
    }
}

void
GraphSearch::computeSuccessorLUT(void)
{
    BOOST_FOREACH(const std::vector<PrimitivePathPtr>& set, m_primitivePathSets)
    {
        BOOST_FOREACH(const PrimitivePathPtr& path, set)
        {
            const LatticePose& p = path->waypoints().front();

            int orientationIdx = m_orientationLUT.lookupIndex(Eigen::Vector2i(p.xYaw(), p.yYaw()));
            m_lutSuccessors[orientationIdx].push_back(path);
        }
    }
}



