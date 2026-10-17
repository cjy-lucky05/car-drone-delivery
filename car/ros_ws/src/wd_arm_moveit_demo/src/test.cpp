#include <iostream>
#include "ros/ros.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include "tf/transform_listener.h"
#include <vector>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#define AR_ID_MAX 20


extern void pub_open(int run_time=1000);
extern void pub_close(int run_time=1000);






    ros::Publisher pub_targetnm, pub_targetpose, pub_goal;
    ros::Subscriber sub_ar_mark, sub_res, sub_moveit_result;
    std_msgs::String name_pose;
    geometry_msgs::PoseStamped target_pose;

void  test2( ros::NodeHandle & handle)
{

    pub_targetnm = handle.advertise<std_msgs::String>("target_named", 5);
    pub_targetpose = handle.advertise<geometry_msgs::PoseStamped>("target_pose", 5);
    sleep(5);
    pub_close();
    sleep(5);

    pub_open();
    sleep(1);
    name_pose.data = "init";
    pub_targetnm.publish(name_pose);
    sleep(5);
    target_pose.pose.position.x = 40;
    target_pose.pose.position.y = 40;
    target_pose.pose.position.z = 40;
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 1.0;
    pub_targetpose.publish(target_pose);
     sleep(5);
 

}