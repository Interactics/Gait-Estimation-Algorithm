#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "openpose_ros_msgs/BoundingBox.h"
#include "openpose_ros_msgs/OpenPoseHuman.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/PointWithProb.h"

float Dist = 0.0;
bool Data_receving = false;

void Callback(const std_msgs::Float32& msg);
float speedOfRobot(const float d);

int main(int argc, char **argv){
    ros::init(argc, argv, "Follower");
    ros::NodeHandle nh;

    ros::Subscriber Follow = nh.subscribe("distance_msg", 1, Callback);
    ros::Publisher Spd_Robot = nh.advertise<std_msgs::Float32>("SPD", 1);
    ros::Rate loop_rate(10);
    std_msgs::Float32 spd;

    ROS_INFO("Follower Node in Online");

    while (ros::ok()){
        ros::spinOnce();        
        if(Data_receving == true){
            spd.data = speedOfRobot(Dist);
            Spd_Robot.publish(spd);
            Data_receving = false;
        }
    }
    ROS_INFO("Follower Node OFF");
    return 0;
}

void Callback(const std_msgs::Float32& msg){
    Dist = msg.data;
    Data_receving = true;
}

float speedOfRobot(const float d){
    float speed;
    if (d < 0) 
        return 0;

    speed = d * 2;

    return speed;
}