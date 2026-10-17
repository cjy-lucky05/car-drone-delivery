
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

 ros::Publisher  mark_pub;
 ros::Publisher  goal_pub;
 ros::Publisher  goal_status_pub;
ros::Subscriber  goal_sub;
ros::Subscriber goal_status_sub;
int count = 0 ;      //total goal num
int index1 = 0   ;    //current goal point index1
int add_more_point = 0 ;// after all goal arrive, if add some more goal
int try_again = 1  ;// try the fail goal once again
visualization_msgs::MarkerArray  markerArray=  visualization_msgs::MarkerArray();
void status_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr & msg) 
{
     if(msg->status.status== 3){
        try_again = 1;
        if(add_more_point==0)
        ROS_INFO("Goal reached");
        if(index1 < count)
        {
                  geometry_msgs::PoseStamped  pose = geometry_msgs::PoseStamped();
        
        pose.header.frame_id = "map";
        pose.header.stamp =ros::Time::now();
            pose.pose.position.x = markerArray.markers[index1].pose.position.x;
            pose.pose.position.y = markerArray.markers[index1].pose.position.y;
            pose.pose.orientation.w = 1;
            goal_pub.publish(pose);
        }
         if(index1 ==count)
          add_more_point = 1;
     }
    else{
          ROS_INFO ("Goal cannot reached has some error : %d  try again!!!!",msg->status.status);

           geometry_msgs::PoseStamped    pose = geometry_msgs::PoseStamped();
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = markerArray.markers[index1-1].pose.position.x;
            pose.pose.position.y = markerArray.markers[index1-1].pose.position.y;
            pose.pose.orientation.w = 1;
            goal_pub.publish(pose);
           
        if(try_again == 1)
           try_again = 0;
        else
            if (index1 < markerArray.markers.size())
                index1 += 1;
    }     
}
void click_callback(const geometry_msgs::PoseStamped::ConstPtr & msg) 
{
   visualization_msgs::Marker  marker = visualization_msgs::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = marker.CYLINDER;
    marker.action = marker.ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = msg->pose.position.x;
    marker.pose.position.y = msg->pose.position.y;
    marker.pose.position.z = msg->pose.position.z;

     markerArray.markers.push_back(marker);
    int id = 0;
     for (visualization_msgs::Marker & m  : markerArray.markers)
          m.id = id++;
     mark_pub.publish(markerArray);
    if (count==0||add_more_point)
    {
          geometry_msgs::PoseStamped  pose = geometry_msgs::PoseStamped();
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = msg->pose.position.x;
        pose.pose.position.y = msg->pose.position.y;
        pose.pose.position.z = msg->pose.position.z;
        pose.pose.orientation.x = msg->pose.orientation.x;
        pose.pose.orientation.y = msg->pose.orientation.y;
        pose.pose.orientation.z = msg->pose.orientation.z;
        pose.pose.orientation.w = msg->pose.orientation.w;
        goal_pub.publish(pose);
        index1 += 1;
    }
     count += 1;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_goal_point");
    signal(SIGINT, MySigintHandler);
    ros::NodeHandle handle;
    ROS_INFO(" multi_goal_point ...");
    mark_pub = handle.advertise<visualization_msgs::Marker>("/path_point_array",10);
    goal_pub=handle.advertise<geometry_msgs::PoseStamped >("/move_base_simple/goal",10);
    goal_status_pub=handle.advertise<move_base_msgs::MoveBaseActionResult>("/move_base/result",10);

    goal_sub = handle.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 20,status_callback);
    goal_status_sub = handle.subscribe<geometry_msgs::PoseStamped>("/goal", 20,click_callback);
    ros::spin();
}