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
#include <stdio.h>
#define AR_ID_MAX 20


   

extern void pub_open(int run_time=1000);
extern void pub_close(int run_time=1000);
double CUBE_SIZE = 0.03;
double map_pose[AR_ID_MAX][3] = {{0.18, 0.0, 0.0}};
//float sentry_pose[] = {0.223, 0.0, 0.204};
//float  err_pose[] = {0,0,0};
uint32_t index1=0;
uint32_t id;
//packet  source
//double source_pose[] = {2.1336042881011963,0.8407098054885864, 0.0, 0.0,0.0, 0.6967164380985531, 0.7173466420659297};
double source_pose[] = {1.3498, 0.2309, 0.0, 0.0, 0.0, -0.045680, 0.998956};
double place_pose[3][7] = {{2.0260, -1.7208, 0.0, 0.0, 0.0, -0.994891, 0.100955},\
                                 {2.0260, -1.7208, 0.0, 0.0, 0.0, -0.994891, 0.100955},\
                                 {2.0260, -1.7208, 0.0, 0.0, 0.0, -0.994891, 0.100955}\
                                 };
// double place_pose[3][7] = {{4.1561198234558105,-0.556495726108551, 0.0, 0.0,0.0, -0.003913870799083181, 0.9999923407783522},\
//                                 {0.9858262538909912,0.831580638885498,0.0,0.0,0.0,0.6864743661268362,0.7271540033932004},\
//                                 {}\
//                                 };
// ★ 2026-10-12 状态机开关：默认 true（不设参数时行为与原来完全一样）
//   测试/外部接管时：rosparam set /ar_pick_place_enable false → 状态机不动，指令转发照常
bool ar_enabled()
{
    bool v = true;
    ros::param::param("/ar_pick_place_enable", v, true);
    return v;
}

int pick_place_action = 0;
int try_again = 1;
ros::Publisher *ppub_targetnm, *ppub_targetpose, *ppub_goal;

geometry_msgs::PoseStamped *ptarget_pose;
geometry_msgs::PoseStamped *pnav_pose;
std_msgs::String *pname_pose;

ros::Subscriber sub_ar_mark, sub_res, sub_moveit_result;
   ros::Publisher pub_targetnm, pub_targetpose, pub_goal;
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped nav_pose;

    std_msgs::String name_pose;
void callback(const ar_track_alvar_msgs::AlvarMarkers &msg)
{
    if (!ar_enabled()) return;   // ★ 开关关闭时状态机不动
    static tf::TransformListener tf_listener;
    if(msg.markers.size()==0)
    return;
   // printf(" %d  %d  %d \n",msg.markers.size(),msg.markers[0].id,pick_place_action);
    if (msg.markers.size() == 0 || msg.markers[0].id >= AR_ID_MAX || pick_place_action != 4)
        return;
    char str[20] = {0};
    id = msg.markers[0].id;
    sprintf(str, "/ar_marker_%d", id);
    tf::StampedTransform trans;
    while (1)
    {
        try
        {
            tf_listener.lookupTransform("/arm_base_link", str, ros::Time(0), trans);
             break;
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            ros::Duration(0.5).sleep();
            continue;
        }
       
    }
    //place_pose[0] = map_pose[id][0];
   // place_pose[1] = map_pose[id][1];
   // place_pose[2] = map_pose[id][2];
   // printf("place_pose  %f  %f %f \n", place_pose[0], place_pose[1], place_pose[2]);

    printf(" object_pose%f  %f %f \n",trans.getOrigin().x(),trans.getOrigin().y(),trans.getOrigin().z());
    // ptarget_pose->header.stamp = ros::Time::now();
    // ptarget_pose->pose.position.x = trans.getOrigin().x();
    // ptarget_pose->pose.position.y = trans.getOrigin().y();
    // ptarget_pose->pose.position.z = trans.getOrigin().z() + CUBE_SIZE;
    // ptarget_pose->pose.orientation.x = 0.0;
    // ptarget_pose->pose.orientation.y = 0.0;
    // ptarget_pose->pose.orientation.z = 0.0;
    // ptarget_pose->pose.orientation.w = 1.0;
    // pick_place_action++;
    // ppub_targetpose->publish(*ptarget_pose);
    name_pose.data = "init";
    ppub_targetnm->publish(name_pose);
    pick_place_action=5;

}

void moveit_result_callback(const moveit_msgs::MoveGroupActionResult   &msg)
{
    if (!ar_enabled()) return;   // ★ 开关关闭时状态机不动
      ROS_INFO("msg.status.status  %d  ", msg.status.status);
        if (msg.status.status == 4)
             ppub_targetnm->publish(*pname_pose);
   else  if (msg.status.status == 3)
    {
        ROS_INFO("Pick Step %d Succeed ", pick_place_action);
tag:        switch (pick_place_action)
        {
        case 0:
             pname_pose->data = "init";
            ppub_targetnm->publish(*pname_pose);
              sleep(5);
            break;
        case 1:
            pub_close( );
             pick_place_action++;
            sleep(2);
        case 2:
            pnav_pose->header.frame_id = "map";
             pnav_pose->header.stamp = ros::Time::now();
             pnav_pose->pose.position.x = source_pose[0];
             pnav_pose->pose.position.y = source_pose[1];
             pnav_pose->pose.position.z = source_pose[2];

             pnav_pose->pose.orientation.x = source_pose[3];
            pnav_pose->pose.orientation.y = source_pose[4];
             pnav_pose->pose.orientation.z = source_pose[5];
             pnav_pose->pose.orientation.w = source_pose[6];
             ppub_goal->publish(*pnav_pose);
             break;
        case 4:
           pick_place_action--;
                break;
             
        case 5:
           
            pub_open();
              pick_place_action++;
               sleep(5);
        case 6:
            pname_pose->data = "grap";
            ppub_targetnm->publish(*pname_pose);
              //sleep(5);
            // ptarget_pose->pose.position.z -= CUBE_SIZE * 1.5;
            // ppub_targetpose->publish(*ptarget_pose);
            break;
        case 7:
                 sleep(2);
               pub_close( );
                pick_place_action++;
                 sleep(1);
        case 8:
            // ptarget_pose->pose.position.z += CUBE_SIZE * 2.0;
            // ppub_targetpose->publish(*ptarget_pose);
             pick_place_action++;
            // break;
        case 9:
            pname_pose->data = "init";
            ppub_targetnm->publish(*pname_pose);
             sleep(5);
            break;
        case 10:
             pnav_pose->header.stamp = ros::Time::now();
            pnav_pose->header.frame_id = "map";
             pnav_pose->pose.position.x = place_pose[index1][0];
             pnav_pose->pose.position.y = place_pose[index1][1];
             pnav_pose->pose.position.z = place_pose[index1][2];
             pnav_pose->pose.orientation.x =place_pose[index1][3];
            pnav_pose->pose.orientation.y =place_pose[index1][4];
             pnav_pose->pose.orientation.z =place_pose[index1][5];
             pnav_pose->pose.orientation.w =place_pose[index1][6];
             ppub_goal->publish(*pnav_pose);
            break;
              
        case 12:
            // pname_pose->data = "grap";
            // ppub_targetnm->publish(*pname_pose);
            //   sleep(5);
            // ptarget_pose->header.stamp = ros::Time::now()
            // ptarget_pose->pose.position.x = place_pose[0];
            // ptarget_pose->pose.position.y = place_pose[1];
            // ptarget_pose->pose.position.z = place_pose[2] + CUBE_SIZE * 0.15;
            // ppub_targetpose->publish(*ptarget_pose);
               pub_open( );      
               sleep(5);
                pick_place_action++;
           
        case 13:
             pick_place_action++;
        case 14:
            pick_place_action = 0;
            index1=(index1+1)%3;
           // sleep(2);
            goto tag;
            break;
        default:
            break;
        }
        pick_place_action++;
        usleep(200000);
    }
    else
        ROS_INFO("arm cannot reached %d  ", msg.status.status);
}

void statusCB(const move_base_msgs::MoveBaseActionResult::ConstPtr msg)
{
    if (!ar_enabled()) return;   // ★ 开关关闭时状态机不动
    if (msg->status.status == 3)
    {
        try_again = 1;
        if (pick_place_action == 11)
        {
           // ptarget_pose->header.stamp = ros::Time::now();
           // ptarget_pose->pose.position.x = place_pose[0];
           // ptarget_pose->pose.position.y = place_pose[1];
           // ptarget_pose->pose.position.z = place_pose[2] + 2 * CUBE_SIZE;
            pick_place_action++;
           // ppub_targetpose->publish(*ptarget_pose);
           pname_pose->data = "grap";
            ppub_targetnm->publish(*pname_pose);
        }
        else if (pick_place_action == 3)
        {
            pname_pose->data = "sentry";
            ppub_targetnm->publish(*pname_pose);
            pick_place_action++;
        }
          else if (pick_place_action == 4)
          {
               pick_place_action++;
           // ppub_targetpose->publish(*ptarget_pose);
             pname_pose->data = "grap";
            ppub_targetnm->publish(*pname_pose);
          }
          printf("pick_place_action %d\n", pick_place_action);
    }
    else
    {
        ROS_INFO("Goal cannot reached has some error : %d  try again!!!!", msg->status.status);
        if (try_again--)
            ppub_goal->publish(*pnav_pose);
    }
}
 
void  ar_pick_place( ros::NodeHandle & handle)
{
  
  
    pub_targetnm = handle.advertise<std_msgs::String>("target_named", 5);
    pub_targetpose = handle.advertise<geometry_msgs::PoseStamped>("target_pose", 5);

    pub_goal = handle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    sub_res = handle.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 20, statusCB);
    sub_ar_mark = handle.subscribe("ar_pose_marker", 10, callback);
    sub_moveit_result = handle.subscribe("/move_group/result", 10, moveit_result_callback);
    ros::spinOnce();
    sleep(15);
    ppub_targetnm=&pub_targetnm;
    ppub_targetpose=&pub_targetpose;
    ppub_goal=&pub_goal;
    ptarget_pose=&target_pose;
    pnav_pose=&nav_pose;
    pname_pose=&name_pose;
    pick_place_action = 2;

    printf("ar_pick_place ... \n");
    name_pose.data = "init";
    ppub_targetnm->publish(name_pose);
    // sleep(5);
    // name_pose.data = "snetry";
    // ppub_targetnm->publish(name_pose);
    

}