
#include "ros/ros.h"
#include "wd_arm_moveit_demo/ArmJoint.h"
#include "sensor_msgs/JointState.h"
#include <vector>

ros::Subscriber sub_fkjonint;
ros::Publisher pub_Arm;
std::vector<float> joints{90.0, 90.0, 90.0, 90.0, 90.0, 30.0};
//std::vector<float> joints{-5.0, -112.0, 151.0, -132.0, 0.0, -92.0};

wd_arm_moveit_demo::ArmJoint arm_joint = wd_arm_moveit_demo::ArmJoint();
double  deg2rad=180/3.14;  
double  deg2rad1=  (180-30)/90.0;  

void inline pubArm( int run_time=1000)
{
        arm_joint.joints = joints;
        arm_joint.run_time = run_time;
        pub_Arm.publish(arm_joint);
}
void  pub_close( int run_time=1000)
{
      joints[5]=135;  
      pubArm();
}
void  pub_open( int run_time=1000)
{
    joints[5]=30;  
      pubArm();
}
    
void fkjonint_callbk(const sensor_msgs::JointState& msg )
{
    
        if(msg.position.size()==5)
            for(int i=0;i<5;i++)
            { 
                joints[i]=deg2rad*msg.position[i]+90;
             //    ROS_INFO("fj\n %f %f %f %f %f  ", joints[0] ,joints[1],joints[2],joints[3],joints[4]);
            }
               
        else if(msg.position.size()==1)      
        {
             joints[5]=deg2rad1*(msg.position[0]*deg2rad+90)+30;  
            //  ROS_INFO("fj\n %f ",joints[5]);
            
        }
             
   
    pubArm();

}
void  ar_pick_place( ros::NodeHandle & handle);
void  test2( ros::NodeHandle & handle);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "SimulationToMachine");
   // signal(SIGINT, MySigintHandler);
    ros::NodeHandle handle;
    ROS_INFO("SimulationToMachine  ...");
    sub_fkjonint = handle.subscribe("/move_group/fake_controller_joint_states", 20, fkjonint_callbk);
    
    pub_Arm = handle.advertise<wd_arm_moveit_demo::ArmJoint>("TargetAngle", 10);
 //   test2(handle);
    ar_pick_place(handle);
    //pubArm();
    ros::spin();
}



