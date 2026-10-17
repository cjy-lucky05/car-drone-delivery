
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include "ros/ros.h"
#include <string>
#include <vector>
#include <ros/time.h>
#include <signal.h>
void MySigintHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

ros::Publisher goal_pub;

ros::Subscriber res_sub;
bool wayPointFinished;
std::vector<double> goalListX;
std::vector<double> goalListY;
std::vector<double> goalListZ;
std::string map_frame;
int goalId;
geometry_msgs::PoseStamped goalMsg = geometry_msgs::PoseStamped();
int loopTimes;
int loop;
void statusCB(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
    if (loopTimes && (loop > loopTimes))
    {
        ROS_INFO("Loop: %d Times Finshed", loopTimes);
        wayPointFinished = true;
        return;
    }

    if (msg->status.status == 3 && (!wayPointFinished))
    {
        goalMsg.header.stamp = ros::Time::now();
        goalMsg.pose.position.x = goalListX[goalId];
        goalMsg.pose.position.y = goalListY[goalId];

        if (abs(goalListZ[goalId]) > 1.0)
        {
            goalMsg.pose.orientation.z = 0.0;
            goalMsg.pose.orientation.w = 1.0;
        }

        else
        {
            double w = sqrt(1 - (goalListZ[goalId]) * goalListZ[goalId]);
            goalMsg.pose.orientation.z = goalListZ[goalId];
            goalMsg.pose.orientation.w = w;
        }

        goal_pub.publish(goalMsg);
        ROS_INFO("Current Goal ID is: %d", goalId);
        if (goalId < (goalListX.size() - 1))
            goalId = goalId + 1;
        else
        {

            goalId = 0;
            loop += 1;
        }
    }
}

void  init_vector(std::string &list,std::vector<double> & goalList)
{
     for (int i = 0, t = 0; i < list.length(); i++)
        if (list[i] == ',' || i == (list.size() - 1))
        {
            goalList.push_back(stold(list.substr(t, i - t).append("\0")));
            t = i + 1;
        }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "way_point");
    signal(SIGINT, MySigintHandler);
    ros::NodeHandle handle("~");
    std::string listx, listy, listz;
    handle.getParam("goalListX", listx);
    handle.getParam("goalListY", listy);
    handle.getParam("goalListZ", listz);
    handle.getParam("map_frame", map_frame);
    handle.getParam("loopTimes", loopTimes);
    int t;
    init_vector(listx,goalListX);
    init_vector(listy,goalListY);
    init_vector(listz,goalListZ);

    ROS_INFO(" way_point ...");
    
    goal_pub = handle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    res_sub = handle.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 20, statusCB);

    loop = 1;

    wayPointFinished = false;
    goalId = 0;
    goalMsg.header.frame_id = map_frame;
    goalMsg.pose.orientation.z = 0.0;
    goalMsg.pose.orientation.w = 1.0;
    sleep(1);
    goalMsg.header.stamp = ros::Time::now();
    goalMsg.pose.position.x = goalListX[goalId];
    goalMsg.pose.position.y = goalListY[goalId];
    goalMsg.pose.orientation.x = 0.0;
    goalMsg.pose.orientation.y = 0.0;
    if (abs(goalListZ[goalId]) > 1.0)
    {
        goalMsg.pose.orientation.z = 0.0;
        goalMsg.pose.orientation.w = 1.0;
    }

    else
    {
        double w = sqrt(1 - (goalListZ[goalId]) * (goalListZ[goalId]));
        goalMsg.pose.orientation.z = goalListZ[goalId];
        goalMsg.pose.orientation.w = w;
    }
    goal_pub.publish(goalMsg);
    ROS_INFO("Current Goal ID is: %d", goalId);
    goalId++;
    ros::spin();
}