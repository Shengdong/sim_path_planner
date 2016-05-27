#include "LatticePlanner.h"

#include <boost/unordered_set.hpp>
#include <fstream>
#include <nav_msgs/Path.h>


LatticePlanner::LatticePlanner(const Options& options)
 : k_notificationInterval(2.0)
 , m_options(options)
 , m_state(STATE_IDLE)
{

}

bool
LatticePlanner::init(ros::NodeHandle& nh)
{
    // set up subscribers
    m_goalSub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&LatticePlanner::goalCallback, this, _1));

    m_poseSub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, boost::bind(&LatticePlanner::poseCallback, this, _1));

    // set up publishers

    m_pathPub = nh.advertise<nav_msgs::Path>("path", 1);



    m_actionServer = boost::make_shared<actionlib::SimpleActionServer<sim_lattice_planner_2d::GoalAction> >(nh, "goal", boost::bind(&LatticePlanner::executeAction, this, _1), false);

    // state lattice
    m_lattice = boost::make_shared<StateLattice>();
    m_lattice->init(m_options.lattice_resolution);

    GraphSearch::Options options;
    options.path_hold_distance = m_options.path_hold_distance;
    options.wp_angular_tolerance = m_options.wp_angular_tolerance;
    options.wp_distance_tolerance = m_options.wp_distance_tolerance;

    m_graphSearch = boost::make_shared<GraphSearch>(boost::ref(nh),
                                                    18.0, m_lattice,
                                                    options);

    m_actionServer->start();
    m_state = STATE_IDLE;

    return true;
}

void
LatticePlanner::replan(double allocatedTimeInSec)
{
    // update sensor data
    if (!m_pose)
    {
        // pose has not yet been received
        if ((ros::Time::now() - m_missingPoseNotificationTime).toSec() > k_notificationInterval)
        {
            ROS_WARN("Waiting for pose information...");
            m_missingPoseNotificationTime = ros::Time::now();
        }
        return;
    }

    if (!m_goalPose)
    {
        // goal has not yet been received
        if ((ros::Time::now() - m_missingGoalNotificationTime).toSec() > k_notificationInterval)
        {
            ROS_WARN("Waiting for goal information...");
            m_missingGoalNotificationTime = ros::Time::now();
        }
        return;
    }

    switch (m_state)
    {
    case STATE_IDLE:
    {
        Eigen::Quaterniond startOrientation(m_pose->pose.orientation.w,
                                            m_pose->pose.orientation.x,
                                            m_pose->pose.orientation.y,
                                            m_pose->pose.orientation.z);
        double startR, startP, startY;
        mat2RPY(startOrientation.toRotationMatrix(), startR, startP, startY);

        m_startPose = boost::make_shared<Pose>(Eigen::Vector3d(m_pose->pose.position.x,
                                                               m_pose->pose.position.y,
                                                               0),
                                               0.0, 0.0, startY);

        ROS_INFO("Planning to goal | p = [%.2f %.2f %.2f] | y = %.2f",
                 m_goalPose->position()(0), m_goalPose->position()(1),
                 m_goalPose->position()(2), m_goalPose->yaw());

        m_graphSearch->init(*m_startPose, *m_goalPose);

        m_detourPose.header.frame_id = "world";
        m_detourPose.header.stamp = ros::Time::now();

        m_detourPose.pose.orientation.w = cos(m_startPose->yaw() / 2.0);
        m_detourPose.pose.orientation.x = 0.0;
        m_detourPose.pose.orientation.y = 0.0;
        m_detourPose.pose.orientation.z = sin(m_startPose->yaw() / 2.0);

        m_detourPose.pose.position.x = m_startPose->position()(0);
        m_detourPose.pose.position.y = m_startPose->position()(1);
        m_detourPose.pose.position.z = 0;

        m_state = STATE_RUNNING;

        ROS_INFO("Waiting at [%.2f %.2f %.2f]",
                 m_detourPose.pose.position.x,
                 m_detourPose.pose.position.y,
                 m_detourPose.pose.position.z);
        break;
    }

    case STATE_NEW_GOAL:
    {
        ROS_INFO("Received new goal.");

        m_state = STATE_IDLE;

        break;
    }

    case STATE_REACHED_GOAL:
    {
        m_goalPose.reset();

        ROS_INFO("Reached goal.");

        m_state = STATE_IDLE;

        break;
    }

    case STATE_PLANNED:
    {
        nav_msgs::Path path1;
        m_pathPub.publish(path1);

        m_state = STATE_PLANNED;
        break;
    }
    case STATE_RUNNING:
    {

        nav_msgs::Path path;
        int status;

        ros::Time startPlanTime = ros::Time::now();


        status = m_graphSearch->iterate(m_pose, path);



        switch (status)
        {
        case GraphSearch::STATUS_NO_PATH:
            ROS_WARN("No path to the goal. Attempting to replan...");
            m_state = STATE_IDLE;
            break;
        case GraphSearch::STATUS_OK:
            m_pathPub.publish(path);
            m_state = STATE_PLANNED;
            break;
        case GraphSearch::STATUS_REACHED_GOAL:
            m_state = STATE_REACHED_GOAL;
            break;
        }
        break;
    }
    default: {}
    }
}


void
LatticePlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!m_goalPose)
    {
        m_goalPose = boost::make_shared<Pose>();
    }
    else
    {
            m_state = STATE_NEW_GOAL;
    }

    m_goalPose->position() << msg->pose.position.x,
                              msg->pose.position.y,
                              0;

    Eigen::Quaterniond q(msg->pose.orientation.w,
                         msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z);
    double roll, pitch, yaw;
    mat2RPY(q.toRotationMatrix(), roll, pitch, yaw);

    m_goalPose->yaw() = yaw;
}


void
LatticePlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    m_pose = msg;
}

void
LatticePlanner::executeAction(const sim_lattice_planner_2d::GoalGoal::ConstPtr& goal)
{
    if (!m_goalPose)
    {
        m_goalPose = boost::make_shared<Pose>();
    }
    else
    {
            m_state = STATE_NEW_GOAL;
     }

    m_goalPose->position() << goal->goal_pos.x,
                              goal->goal_pos.y,
                              goal->goal_pos.z;
    m_goalPose->yaw() = goal->goal_yaw;
}

void 
LatticePlanner::mat2RPY(const Eigen::Matrix3d& m, double& roll, double& pitch, double& yaw)
{
    roll = atan2(m(2,1), m(2,2));
    pitch = atan2(-m(2,0), sqrt(m(2,1) * m(2,1) + m(2,2) * m(2,2)));
    yaw = atan2(m(1,0), m(0,0));
}

