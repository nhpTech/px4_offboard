#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ncurses.h>
#include <sstream>
#include <iomanip>
#include <csignal>
#include <cmath>
#include <cstdio>

volatile bool new_msg = false;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

trajectory_msgs::MultiDOFJointTrajectory traj_msg;
void traj_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
    if (!new_msg)
    {
        traj_msg = *msg;
        new_msg = true;
    }
    //std::cout << "New traj ---" << std::endl;
    //for (int i = 0; i < traj_msg.points.size(); i++)
    //{
    //    std::cout << traj_msg.points[i].transforms[0].translation.x << " "
    //              << traj_msg.points[i].transforms[0].translation.y << " "
    //              << traj_msg.points[i].transforms[0].translation.z << " "
    //              << traj_msg.points[i].transforms[0].rotation.x << " " 
    //              << traj_msg.points[i].transforms[0].rotation.y << " "
    //              << traj_msg.points[i].transforms[0].rotation.z << " "
    //              << std::endl;
    //}
}

void sig_handler(int signum)
{
    ros::shutdown();
}

int main(int argc, char **argv)
{
    signal(SIGINT, sig_handler);

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/iris_0/mavros/state", 10, state_cb);
    ros::Subscriber traj_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>
            ("/iris_0/command/trajectory", 100, traj_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/iris_0/mavros/setpoint_position/local", 100);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/iris_0/mavros/setpoint_velocity/cmd_vel", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/iris_0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(60.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    tf2::Quaternion q_tf;
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = -2.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.8;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (current_state.mode != "OFFBOARD")
    {
        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
        }
    } 

    if (!current_state.armed)
    {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Vehicle armed");
        }
    }

    int i = 0;

    while(ros::ok())
    {
        //if (new_msg)
        //{
        //    i = 0;
        //    new_msg = false;
        //    ROS_INFO("New trajectory");
        //}
       
        if (new_msg) 
        {
          if (i < traj_msg.points.size())
          {
              pose.header.stamp = ros::Time::now();
              pose.pose.position.x = traj_msg.points[i].transforms[0].translation.x;
              pose.pose.position.y = traj_msg.points[i].transforms[0].translation.y;
              pose.pose.position.z = traj_msg.points[i].transforms[0].translation.z;
              pose.pose.orientation.x = traj_msg.points[i].transforms[0].rotation.x;
              pose.pose.orientation.y = traj_msg.points[i].transforms[0].rotation.y;
              pose.pose.orientation.z = traj_msg.points[i].transforms[0].rotation.z;
              pose.pose.orientation.w = traj_msg.points[i].transforms[0].rotation.w;
              i++;
          } 
          else
          {
              i = 0;
              new_msg = false;
          }
        }

        local_pos_pub.publish(pose);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
