#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "openpose_ros_msgs/BoundingBox.h"
#include "openpose_ros_msgs/OpenPoseHuman.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/PointWithProb.h"

/***************************************************
 *  Class SpdCtrl
 ***************************************************/
class SpdCtrl{
private:
    float Dist = 0.0;
    bool Data_receving = false;
    ros::Subscriber Follow;
    ros::Publisher Spd_Robot;
    std_msgs::Float32 spd;
public:
    SpdCtrl(ros::NodeHandle* nh);
    ~SpdCtrl(){}
    void Callback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg);
    float speedOfRobot(const float d);
    void PubSpd();
};

//Class Method Define
SpdCtrl::SpdCtrl(ros::NodeHandle* nh){
    Follow  = nh -> subscribe("distance_msg", 1, &SpdCtrl::Callback, this);
    Spd_Robot = nh -> advertise<std_msgs::Float32>("SPD", 1);
    Dist = 0.0;
    Data_receving = false;
    ros::Rate loop_rate(10);
    ROS_INFO("Follower Node in Online");
}
void SpdCtrl::Callback(const openpose_ros_msgs::OpenPoseHumanList::ConstPtr& msg){
    if (msg->num_humans == 0 ){
        Data_receving = false;
        return;
    }
    Dist = msg->human_list[0].body_key_points_with_prob[10].x;
    Data_receving = true;
}
float SpdCtrl::speedOfRobot(const float d){
    float speed;
    if (d < 0) 
        return 0;
    speed = d * 2;
    return speed;
}
void SpdCtrl::PubSpd(){
    if(Data_receving == true){
        spd.data = speedOfRobot(Dist);
        Spd_Robot.publish(spd);
        Data_receving = false;
    }
}
//**************************************************
// END Class SpdCtrl
//================================================

int main(int argc, char **argv){
    ros::init(argc, argv, "Follower");
    ros::NodeHandle nh;
    SpdCtrl Spdcontrol(&nh);
    while (ros::ok()){
        ros::spinOnce();    
        Spdcontrol.PubSpd();    
    }
    ROS_INFO("Follower Node OFF");
    return 0;
}