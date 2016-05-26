#ifndef LATTICE_PLANNER_GRAPHSEARCH_H
#define LATTICE_PLANNER_GRAPHSEARCH_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "Heuristics.h"
#include "Pose.h"
#include <queue>
#include "StateGrid.h"
#include "StateLattice.h"


// Implements Anytime Dynamic A*.
class GraphSearch
{
public:
    class Options
    {
    public:
        Options()
         : path_hold_distance(0.5)
         , wp_angular_tolerance(0.35)
         , wp_distance_tolerance(0.3)
        {

        }

        double path_hold_distance;
        double wp_angular_tolerance;
        double wp_distance_tolerance;
    };

    enum
    {
        STATUS_OK = 0,
        STATUS_REACHED_GOAL = 1,
        STATUS_NO_PATH = 2
    };

    GraphSearch(ros::NodeHandle& nh,
                double gridWidth,
                const boost::shared_ptr<StateLattice>& stateLattice,
                const Options& options);

    bool init(const Pose& poseStart,
              const Pose& poseGoal);
    int iterate(const geometry_msgs::PoseStamped::ConstPtr& pose,
                nav_msgs::Path& path);
    void extractPathFromSolution(const geometry_msgs::PoseStamped::ConstPtr& pose,
                                 nav_msgs::Path& path);

    bool isGoalReached(void) const;

private:
    void reset(void);
    void computeOrImprovePath(void);
    void computeKey(StatePtr& s);
    StatePtr getGoalState(void) const;

    void initializeState(StatePtr& s);
    bool computePath(void);
    void publishCurrentSolution(void);
    void findSolution(std::deque<StatePtr>& solution);
    bool findNearestFeasibleState(StatePtr& nearestState);
    void printTable(void);
    void extractPathSolution(const geometry_msgs::PoseStamped::ConstPtr& pose, nav_msgs::Path& path);

    void computePredecessorLUT(void);
    void computeSuccessorLUT(void);

    std::vector<StatePtr> PathSearch(LatticePose Start, LatticePose Goal);

    // visualization functions


    // inflation factor for heuristic values

    // grid stores explored states
    StateGrid m_stateGrid;


    // OPEN priority queue
    std::priority_queue<StatePtr, std::vector<StatePtr>, CompareState> m_openPQ;
    // CLOSED list
    std::vector<StatePtr> m_closedSet;
    std::vector<StatePtr> m_path;

    std::vector<std::vector<PrimitivePathPtr> > m_primitivePathSets;

    boost::shared_ptr<Heuristics> m_heuristics;

    double m_resolution;

    StatePtr m_startS;
    StatePtr m_goalS;
    LatticePose m_lastWaypoint;
    LatticePose m_lpStart;
    LatticePose m_lpGoal;

    boost::unordered_map<int, std::vector<PrimitivePathPtr> > m_lutPredecessors;
    boost::unordered_map<int, std::vector<PrimitivePathPtr> > m_lutSuccessors;
    OrientationLUT m_orientationLUT;

    std::vector<StatePtr> m_fixedSolution;
    std::deque<StatePtr> m_currentSolution;

    Options m_options;
};


#endif
