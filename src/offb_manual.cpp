#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ncurses.h>
#include <sstream>
#include <iomanip>
#include <csignal>
#include <cmath>
#include <cstdio>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void sig_handler(int signum)
{
    ros::shutdown();
}

void update_pose(const geometry_msgs::PoseStamped &pose, double npi)
{
    std::stringstream xyz_ss;
    xyz_ss << std::setprecision(2);
    xyz_ss << std::setw(8) << pose.pose.position.x 
           << std::setw(8) << pose.pose.position.y 
           << std::setw(8) << pose.pose.position.z
           << std::setw(8) << npi; 
    mvaddstr(9, 0, xyz_ss.str().c_str());
}

int main(int argc, char **argv)
{
    signal(SIGINT, sig_handler);

    FILE *traj_file = fopen("/home/phuc/Projects/Capstone/catkin_ws/src/px4_offboard/trajectory/indoor1.txt", "w"); 
    if (traj_file == NULL)
    {
        ROS_ERROR("Fail to open trajectory file");
        return 1;
    }

    WINDOW *win = initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(win, TRUE);

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/iris_0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/iris_0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/iris_0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    tf2::Quaternion q_tf;
    double npi = 0.0;
    double angle_rad = 0.0;
    pose.pose.position.x = 0.;
    pose.pose.position.y = 0.;
    pose.pose.position.z = 0.;
    pose.pose.orientation.x = 0.;
    pose.pose.orientation.y = 0.;
    pose.pose.orientation.z = 0.;
    pose.pose.orientation.w = 1.;

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

    ros::Time last_request = ros::Time::now();
    char cmd;

    if (current_state.mode != "OFFBOARD")
    {
        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
        }
    } 

    mvaddstr(0, 0, "Command set:");
    mvaddstr(1, 0, "g = arm");
    mvaddstr(2, 0, "h = disarm");
    mvaddstr(3, 0, "w/s = increase/decrease x");
    mvaddstr(4, 0, "a/d = increase/decrease y");
    mvaddstr(5, 0, "i/k = increase/decrease z");
    mvaddstr(6, 0, "j/l = rotate yaw left/right");
    mvaddstr(7, 0, "r = reset pose");
    mvaddstr(8, 0, "|   X   ");
    mvaddstr(8, 8, "|   Y   ");
    mvaddstr(8, 16, "|   Z   |");
    mvaddstr(8, 24, "|  k*PI |");
    mvaddstr(11, 0, "Status updates: ");
    move(12, 0);
    refresh();

    while(ros::ok())
    {
        cmd = getch();

        switch (cmd)
        {
        case 'g':
            if (!current_state.armed)
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
            }
            break;
        case 'h':
            if (current_state.armed)
            {
                arm_cmd.request.value = false;
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle disarmed");
                }
            }
            break;
        case 'w':
            //pose.pose.position.x += 0.1;
            pose.pose.position.y += sin(angle_rad) * 0.1;
            pose.pose.position.x += cos(angle_rad) * 0.1;
            update_pose(pose, npi);
            break;
        case 's':
            //pose.pose.position.x -= 0.1;
            pose.pose.position.y -= sin(angle_rad) * 0.1;
            pose.pose.position.x -= cos(angle_rad) * 0.1;
            update_pose(pose, npi);
            break;
        case 'a':
            //pose.pose.position.y += 0.1;
            pose.pose.position.y += cos(angle_rad) * 0.1;
            pose.pose.position.x += -sin(angle_rad) * 0.1;
            update_pose(pose, npi);
            break;
        case 'd':
            //pose.pose.position.y -= 0.1;
            pose.pose.position.y -= cos(angle_rad) * 0.1;
            pose.pose.position.x -= -sin(angle_rad)* 0.1;
            update_pose(pose, npi);
            break;
        case 'i':
            pose.pose.position.z += 0.1;
            update_pose(pose, npi);
            break;
        case 'k':
            pose.pose.position.z -= 0.1;
            update_pose(pose, npi);
            break;
        case 'j':
            npi += 0.1;
            angle_rad = npi * M_PI;
            q_tf.setRPY(0., 0., angle_rad);
            q_tf.normalize();
            pose.pose.orientation.z = q_tf.z();
            pose.pose.orientation.w = q_tf.w();
            update_pose(pose, npi);
            break;
        case 'l':
            npi -= 0.1;
            angle_rad = npi * M_PI;
            q_tf.setRPY(0., 0., angle_rad);
            q_tf.normalize();
            pose.pose.orientation.z = q_tf.z();
            pose.pose.orientation.w = q_tf.w();
            update_pose(pose, npi);
            break;
        case 'r':
            pose.pose.position.x = 0.;
            pose.pose.position.y = 0.;
            pose.pose.position.z = 0.;
            pose.pose.orientation.x = 0.;
            pose.pose.orientation.y = 0.;
            pose.pose.orientation.z = 0.;
            pose.pose.orientation.w = 1.;
            npi = 0.0;
            update_pose(pose, npi);
            break;
        default:
            break;
        }
        move(12, 0);
        local_pos_pub.publish(pose);

        if (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd' || cmd == 'i' || cmd == 'k' || cmd == 'j' || cmd == 'l' || cmd == 'r')
           fprintf(traj_file, "%f %f %f 0 0 %f %f\n",
                   pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                   pose.pose.orientation.z, pose.pose.orientation.w);
        
        ros::spinOnce();
        rate.sleep();
    }

    endwin();
    fclose(traj_file);

    return 0;
}
