#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from openpose_ros_msgs.msg import BoundingBox
from openpose_ros_msgs.msg import OpenPoseHuman
from openpose_ros_msgs.msg import OpenPoseHumanList
from openpose_ros_msgs.msg import PointWithProb

# {0,  "Nose"},
# {1,  "Neck"},
# {2,  "RShoulder"},
# {3,  "RElbow"},
# {4,  "RWrist"},
# {5,  "LShoulder"},
# {6,  "LElbow"},
# {7,  "LWrist"},
# {8,  "MidHip"},
# {9,  "RHip"},      <-
# {10, "RKnee"},     <-
# {11, "RAnkle"},    <-
# {12, "LHip"},      <-
# {13, "LKnee"},     <- 2
# {14, "LAnkle"},    <-
# {15, "REye"},
# {16, "LEye"},
# {17, "REar"},
# {18, "LEar"},
# {19, "LBigToe"},
# {20, "LSmallToe"},
# {21, "LHeel"},
# {22, "RBigToe"},
# {23, "RSmallToe"},
# {24, "RHeel"},
# {25, "Background"}


Dist = 0.0
Data_receving = False

#rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, callback)
#Spd_Robot = rospy.Publisher("get_speed", Float32, queue_size = 1)
global Spd_Robot

def callback (msg) :
    global Spd_Robot
    try:
        Dist = msg.human_list[0].body_key_points_with_prob[10].x
    except :
        Dist = 0

    spd = speedOfRobot(Dist)
    Spd_Robot.publish(spd)


def speedOfRobot(d) : 
    speed = 1.2
    # if (d <= 0 ) :
    #     return 0
    # speed = ( 320 - d ) / 160 

    if (speed < 0) :
        speed = 0
    return speed

def main() :
    global Spd_Robot
    rospy.init_node('Follower', anonymous = False)
    rospy.loginfo('Follower Node Start')

    rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, callback)
    Spd_Robot = rospy.Publisher("get_speed", Float32, queue_size = 1)

    rospy.Rate(10)
    rospy.spin()


if __name__ == '__main__' :
    try :
        main()
    except rospy.ROSInterruptException:
        pass
