#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from openpose_ros_msgs.msg import OpenPoseHumanList
from openpose_ros_msgs.msg import PointWithProb
from openpose_ros_msgs.msg import OpenPoseHuman
from openpose_ros_msgs.msg import BoundingBox

height1 = 480
width1 = 640

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
# {13, "LKnee"},     <- 
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


global img_np

def callback1(data) :
    keypointList = data.human_list[0].body_key_points_with_prob
    print (point_to_depth(keypointList, 0))

def point_to_depth(keypointList, n_body) :
    global img_np

    x_pix = keypointList[n_body].x
    y_pix = keypointList[n_body].y

    dist =  img_np[int(y_pix),int(x_pix)]
    x_pt, y_pt = getCoord(x_pix, y_pix, dist)

    return int(x_pt), int(y_pt), dist

    
def callback2(img_msg) :
    global img_np

    img = msg_to_numpy(img_msg)
    img_np = np.asarray(img)

    ## type : numpy.ndarray
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(img_np, alpha = 0.03), cv2.COLORMAP_JET)
    
    cv2.imshow('rs', depth_colormap)
    cv2.waitKey(1)


def get_keypoint() :
    rospy.init_node('get_keypoint', anonymous = False)
    rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList,callback1)
    rospy.Subscriber("depth_image", Image, callback2)
    rospy.spin()

def msg_to_numpy(data):
        bridge = CvBridge()
        try:
            raw_img = bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as err:
            print("err")

        return raw_img 

def getCoord(x,y,distance):
    X = getHorizontalCoordinate(x, distance)
    Y = getVerticalCoordinate(y, distance)
    return X,Y

def getVerticalCoordinate(y, distance):
    #rs RGB : FOV 69.4 x 42.5 x 77 (HxVxD)
    #rs Depth : FOV 73 x 58 x 95 (HxVxD)
    VFov2 = math.radians(42.5/2)
    VSize = math.tan(VFov2) * 2
    VCenter = (height1-1)/2
    VPixel = VSize/(height1 - 1)
    VRatio = (VCenter - y) * VPixel
    return distance * VRatio

def getHorizontalCoordinate(x, distance):
    #rs RGB : FOV 69.4 x 42.5 x 77 (HxVxD)
    #rs Depth : FOV 73 x 58 x 95 (HxVxD)
    HFov2 = math.radians(64.4/2)
    HSize = math.tan(HFov2) * 2
    HCenter = (width1-1)/2
    HPixel = HSize/(width1 - 1)
    HRatio = (x - HCenter) * HPixel
    return distance * HRatio


if __name__ == '__main__' :
    get_keypoint()