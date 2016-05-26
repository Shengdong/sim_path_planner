#ifndef LATTICE_PLANNER_LATTICE_PLANNER_H
#define LATTICE_PLANNER_LATTICE_PLANNER_H

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>



#include "sim_lattice_planner_2d/GoalAction.h"
#include "GraphSearch.h"
#include "StateLattice.h"

class LatticePlanner
{
public:
    class Options
    {
    public:
        Options()
         : lattice_resolution(1)
         , path_hold_distance(0.5)
         , planning_hz(20.0)
         , wp_angular_tolerance(0.35)
         , wp_distance_tolerance(0.3)
        {

        }

        std::string boundary_filename;
        double lattice_resolution;
        double path_hold_distance;
        double planning_hz;
        double wp_angular_tolerance;
        double wp_distance_tolerance;
    };

    enum State
    {
        STATE_IDLE = 0,     // waiting for command to start planning
        STATE_NEW_GOAL = 1, // goal has been changed
        STATE_REACHED_GOAL = 2,
        STATE_RUNNING = 3,  // actively planning
        STATE_PLANNED = 4,
    };

    LatticePlanner(const Options& options);

    bool init(ros::NodeHandle& nh);
    void replan(double allocatedTimeInSec);



private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void executeAction(const sim_lattice_planner_2d::GoalGoal::ConstPtr& goal);

    double k_notificationInterval;
void mat2RPY(const Eigen::Matrix3d& m, double& roll, double& pitch, double& yaw);

    Options m_options;
    State m_state;

    ros::Subscriber m_goalSub;
    ros::Subscriber m_poseSub;
    ros::Publisher m_boundaryMarkerPub;
    ros::Publisher m_pathPub;
    boost::shared_ptr<actionlib::SimpleActionServer<sim_lattice_planner_2d::GoalAction> > m_actionServer;

    geometry_msgs::PoseStamped::ConstPtr m_pose;

    boost::shared_ptr<StateLattice> m_lattice;
    boost::shared_ptr<GraphSearch> m_graphSearch;

    PosePtr m_startPose;
    PosePtr m_goalPose;
    geometry_msgs::PoseStamped m_detourPose;

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > m_boundaryCellVec;

    ros::Time m_missingPoseNotificationTime;
    ros::Time m_missingMapNotificationTime;
    ros::Time m_missingGoalNotificationTime;
};

#endif
