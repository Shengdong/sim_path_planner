#include <ros/ros.h>

#include "LatticePlanner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lattice_planner");

    ros::NodeHandle nh;

    LatticePlanner::Options options;

    double planningPeriod = 1.0 / options.planning_hz;

    LatticePlanner planner(options);
    if (!planner.init(nh))
    {
        ROS_ERROR("Cannot initialize planner.");
        return 1;
    }

    if (ros::ok())
    {
        ros::spinOnce();
    }

    while (ros::ok())
    {
        ros::Time startTime = ros::Time::now();

        planner.replan(planningPeriod);
        sleep(2);

        ros::spinOnce();

        double elapsedTime = (ros::Time::now() - startTime).toSec();
        double sleepTime = planningPeriod - elapsedTime;
        if (sleepTime > 0.0)
        {
            ros::Duration(sleepTime).sleep();
        }
    }
    return 0;
}
