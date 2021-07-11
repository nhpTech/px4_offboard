#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <csignal>
#include <fstream>
#include <sstream>
#include <cmath>

void sig_handler(int signum)
{
    ros::shutdown();
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

bool poses_match(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2, double margin)
{
    bool ret = ((abs(p1.pose.position.x - p2.pose.position.x) < margin)
        &&  (abs(p1.pose.position.y - p2.pose.position.y) < margin)
        &&  (abs(p1.pose.position.z - p2.pose.position.z) < margin)
        &&  (abs(p1.pose.orientation.x - p2.pose.orientation.x) < margin)
        &&  (abs(p1.pose.orientation.y - p2.pose.orientation.y) < margin)
        &&  (abs(p1.pose.orientation.z - p2.pose.orientation.z) < margin)
        &&  (abs(p1.pose.orientation.w - p2.pose.orientation.w) < margin));
    // std::cout << (abs(p1.pose.position.x - p2.pose.position.x) < margin)
    //     <<  (abs(p1.pose.position.y - p2.pose.position.y) < margin)
    //     <<  (abs(p1.pose.position.z - p2.pose.position.z) < margin)
    //     <<  (abs(p1.pose.orientation.x - p2.pose.orientation.x) < margin)
    //     <<  (abs(p1.pose.orientation.y - p2.pose.orientation.y) < margin)
    //     <<  (abs(p1.pose.orientation.z - p2.pose.orientation.z) < margin)
    //     <<  (abs(p1.pose.orientation.w - p2.pose.orientation.w) < margin) << std::endl;
    return ret;
}

int main(int argc, char **argv)
{
    signal(SIGINT, sig_handler);

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/iris_0/mavros/state", 10, state_cb);
    //ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //        ("/iris_0/mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/iris_0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/iris_0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    // wait for FCU connection
    ROS_INFO("Connecting to FCU...");
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connection established");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    for (int i = 0; i < 100; i++)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    std::string line;
    std::ifstream traj_file("/home/phuc/Projects/Capstone/catkin_ws/src/px4_offboard/trajectory/indoor1.txt");
    if (!traj_file.is_open())
    {
        ROS_ERROR("Failed to open trajectory file");
        return 1;
    }
    else
    {
        ROS_INFO("Found trajectory file");
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (current_state.mode != "OFFBOARD")
    {
        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");    //send a few setpoints before starting
        }
    } 

    if (!current_state.armed)
    {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Vehicle armed");
        }
    }
    
    ros::Time last_update = ros::Time::now();
    rate = ros::Rate(30.0);
    
    while(ros::ok())
    {
        if ((ros::Time::now() - last_update) > ros::Duration(0.4))
        {
            if (getline(traj_file, line))
            {
                std::stringstream traj_ss;

                traj_ss << line;
                traj_ss >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z
                        >> pose.pose.orientation.x >> pose.pose.orientation.y >> pose.pose.orientation.z >> pose.pose.orientation.w;
                last_update = ros::Time::now();
            }
            else
            {
                ROS_INFO("Trajectory finish");
                break;
            }
        }

        local_pos_pub.publish(pose);
        //ROS_INFO("Set destination to [%.1f %.1f %.1f][%.1f %.1f %.1f %.1f]", 
        //        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        //        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
