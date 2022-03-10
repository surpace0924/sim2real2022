#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>

#include <array>
#include <string>

#include "./trajectory_file_parser.h"


// 各ゾーンのIDと名称
const std::array<std::string, 7> zone_names = {
    "sz",
    "mz1",
    "mz2",
    "mz3",
    "mz4",
    "mz5",
    "es",
};

uint8_t now_goal_id = 0;
void callbackGoalId(const std_msgs::UInt8 &msg)
{
    now_goal_id = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim2real_path_planner_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    ////////// Subscriber //////////
    ros::Subscriber sub_goal_id = nh.subscribe("goal_id", 1000, callbackGoalId);
    
    ////////// Publisher //////////
    ros::Publisher pub_trajectory = nh.advertise<nav_msgs::Path>("/trajectory", 1000);

    TrajectoryFileParser tfp;
    tfp.setTrajectoryFrameId("/map");

    while (nh.ok())
    {
        // 経路データ読み出して配信
        tfp.setFilePath("~/catkin_ws/src/sim2real_path_planner/trajectories/sz_" + zone_names[now_goal_id] + ".csv");
        nav_msgs::Path trajectory_msg = tfp.parse();
        pub_trajectory.publish(trajectory_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
