#!/usr/bin/env python
## ROS
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from openpose_ros_msgs.msg import OpenPoseHumanList
from openpose_ros_msgs.msg import PointWithProb
from openpose_ros_msgs.msg import OpenPoseHuman
from openpose_ros_msgs.msg import BoundingBox

## Calculation
import numpy as np
import cv2
import math

import tensorflow as tf
## Machine Learning
import keras
import keras.backend as K 

from keras.layers import LSTM 
from keras.models import Sequential 
from keras.layers import Dense 
from keras.layers import Flatten
from keras.models import load_model
from keras.models import model_from_json

#Filter
from scipy import signal


# from tensorflow.python.keras.backend import set_session


# sess = tf.Session()
# graph = tf.get_default_graph()

# set_session(sess)


print('--Program Online--')
# global graph,model
# graph = tf.get_default_graph()

# session = keras.backend.get_session()
# init = tf.global_variables_initializer()
# session.run(init)

# K.clear_session()
# model = Sequential() # Sequeatial Model 
# model.add(LSTM(70, input_shape=(30, 1))) # (timestep, feature)
# model.add(Dense(1)) # output = 1 
# model.compile(loss='mean_squared_error', optimizer='adam')

# print('--modle loading--')
# json_file = open("/home/wowmecha/Desktop/model100.json", "r")
# loaded_model_json = json_file.read() 
# json_file.close() 
# model = model_from_json(loaded_model_json)

# model.load_weights('/home/wowmecha/Desktop/Gait_LSTM_NODE12_70model100W.h5')
# model.compile(loss='mean_squared_error', optimizer='adam')
# print(model.summary())
# model._make_predict_function()
# print('--KERAS Model is loaded--')

## LSTM Model Setting
global graph
global sess
sess = tf.Session()
from keras.backend import set_session

graph = tf.get_default_graph()  
set_session(sess)
model = load_model('/home/wowmecha/Desktop/Gait_LSTM_NODE12_70model100.h5')
model.compile(loss='mean_squared_error', optimizer='adam')

print(model.summary())

### Scaler setting
from sklearn.preprocessing import MinMaxScaler
sc_x = MinMaxScaler(feature_range=(0, 1))
sc_y = MinMaxScaler(feature_range=(0, 1))
Xsc_ = [[76.68918234,   67.08620117, -1496.91313308],
        [ 178.85618721, 152.72581815, 1201.76893437]]
Ysc_ = [[-2.24800694], 
        [4.12432266]]
sc_x.fit(np.array(Xsc_))
sc_y.fit(np.array(Ysc_))
###

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
# {8,  "MidHip"},    <-
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

Time_flag = False
time_sec = 0
global img_np
firstCheck = True

global PubSpd
global theta_leftknee_prev
global time_now
global time_prev
global data_pred
global data_vel

file = open("./skltData.txt", 'w')

def callback1(data) :
    #try:
        keypointList = data.human_list[0].body_key_points_with_prob
        timeInfo = data.header.stamp
        pointDepthXYZ(keypointList, 0)
        printLegPoint(keypointList,timeInfo)
    #except:
        #print('nobody keypoint detected')

    # print (pointDepthXYZ(keypointList, 0))

#LegKeypoint Detecting and Printing.
def printLegPoint(keyPoint, timeInfo) :
    global firstCheck
    global theta_leftknee_prev
    global time_now
    global time_prev
    global data_pred
    global data_vel

    
    node_size = 12
    
    time_now = timeWriter(timeInfo)
    TIME = time_now
    time_now = float(time_now)
    MidHip = pointDepthXYZ(keyPoint, 8)

    RHip = pointDepthXYZ(keyPoint, 9)
    RKnee = pointDepthXYZ(keyPoint, 10)
    RAnkle = pointDepthXYZ(keyPoint, 11)

    LHip = pointDepthXYZ(keyPoint, 12)
    LKnee = pointDepthXYZ(keyPoint, 13)
    LAnkle = pointDepthXYZ(keyPoint, 14)
    
    if (firstCheck == True) :
        time_prev = 0
        prev_MidHip = MidHip
        prev_RHip = RHip
        prev_RKnee = RKnee
        prev_RAnkle = RAnkle
        prev_LHip = LHip
        prev_LKnee = LKnee
        prev_LAnkle = LAnkle
        firstCheck = False
        theta_leftknee_prev = 0
        data_pred = np.array([])
        data_vel = np.array([])
        
    if ( -1000 < LHip[0] < 1000 and -1000 < LAnkle[0] < 1000 )  :

        ###
        ###theta of LeftKnee
        ###
        left_shin_x     = int(LAnkle[0]) - int(LKnee[0])
        left_shin_y     = int(LAnkle[1]) - int(LKnee[1])
        left_shin_z     = int(LAnkle[2]) - int(LKnee[2])
        left_thigh_x    =   int(LHip[0]) - int(LKnee[0])
        left_thigh_y    =   int(LHip[1]) - int(LKnee[1])
        left_thigh_z    =   int(LHip[2]) - int(LKnee[2])

        costheta_left = \
            ((left_shin_x * left_thigh_x) + (left_shin_y * left_thigh_y) + (left_shin_z * left_thigh_z)) / \
            (np.sqrt(left_shin_x**2 + left_shin_y**2 + left_shin_z**2) * np.sqrt(left_thigh_x**2 + left_thigh_y**2 + left_thigh_z**2)) 

        theta_leftknee = np.arccos(costheta_left) / np.pi * 180

        ###
        ### Theta of LeftShin to ground
        ###
                
        shin_x = -left_shin_x
        shin_y = -left_shin_y
        shin_z = -left_shin_z
        
        costheta_Knee_vs_ground = ((shin_x * 1) + (shin_y * 0) + (shin_z * 0)) / \
        (np.sqrt(shin_x**2 + shin_y**2 + shin_z**2) * np.sqrt(1**2 + 0**2 + 0**2)) 

        theta_shin_vs_ground = np.arccos(costheta_Knee_vs_ground) / np.pi * 180

        ###
        ### Make Angle velocity of knee
        ###


        velocity_knee_left =  (theta_leftknee - theta_leftknee_prev) / (time_now - time_prev)


        ###
        ### prev declare
        ### 

        time_prev = time_now
        theta_leftknee_prev = theta_leftknee

        data_arr = np.array([theta_leftknee, theta_shin_vs_ground, velocity_knee_left])
        print(data_arr)
        data_arr = sc_x.transform(data_arr.reshape((1,3))).squeeze()
        data_pred = np.hstack([data_pred, data_arr])

        if (data_pred.shape[0] == node_size * 3) :

            INPUT_D = data_pred.reshape((1, node_size * 3, 1))

            global graph
            global sess
            with graph.as_default():  
                set_session(sess)
                velocity = (model.predict(INPUT_D))
            velocity = sc_y.inverse_transform(velocity)[0,0]
            print('----')
            print(velocity)

            data_pred = data_pred[3:]
            data_vel = np.hstack([data_vel, velocity])

            if (data_vel.shape[0] >= 13) : 
                b, a = signal.butter(3, 0.01) ## Butterworth Filter 
                filterd_y = signal.filtfilt(b, a, data_vel) ## Filter
                filterd_y1 = signal.filtfilt(b, a, data_vel[-13:]) ## Filter

                speed = filterd_y[-1]
                #data_vel = data_vel[:-1]
                print('Filtered Speed: ', speed)
                print('Filtered Speed: ', filterd_y1[-1])

                PubSpd.publish(speed)
                            
    # dataStr = str(TIME) + ' ' + str(MidHip) + ' ' + \
    # str(RHip) + ' ' + str(RKnee) + ' ' + str(RAnkle) + ' ' + \
    # str(LHip) + ' ' + str(LKnee) + ' ' + str(LAnkle) + ' ' + str(mSPD) + '\n'

    dataStr = str(TIME) + ' ' + str(MidHip) + ' ' + \
    str(RHip) + ' ' + str(RKnee) + ' ' + str(RAnkle) + ' ' + \
    str(LHip) + ' ' + str(LKnee) + ' ' + str(LAnkle) + ' '  + '\n'

## Selecting Left Leg
    # if (RKnee[2] <= LKnee[2]) : 
    #     dataStr = str(TIME) + ' ' + str(MidHip) + ' ' + \
    #     str(RHip) + ' ' + str(RKnee) + ' ' + str(RAnkle) + ' ' + \
    #     str(LHip) + ' ' + str(LKnee) + ' ' + str(LAnkle) + '\n'

    # if (RKnee[2] > LKnee[2]) :
    #     dataStr = str(TIME) + ' ' + str(MidHip) + ' ' + \
    #     str(LHip) + ' ' + str(LKnee) + ' ' + str(LAnkle) + ' ' + \
    #     str(RHip) + ' ' + str(RKnee) + ' ' + str(RAnkle) + '\n'

    file.write(dataStr)

# Returning Time of system's running.
    

def timeWriter(timeInfo) : 
    global Time_flag, time_sec
    timeStr = ''

    #Setting 0sec
    if (Time_flag == False) :
        time_sec = timeInfo.secs
        Time_flag = True
    time = timeInfo.secs - time_sec  

    #Solve nsecs err.
    time_size = len(str(timeInfo.nsecs))
    if time_size < 9 :
        for i in range(0,9-time_size):
            #file.write(str('0'))
            timeStr = '0' + timeStr
        timeStr = timeStr + str(timeInfo.nsecs)
    else :
        timeStr = str(timeInfo.nsecs)
    
    timeStr = str(time)+ '.' + timeStr

    return timeStr


def pointDepthXYZ(keypointList, n_body) :
    global img_np

    x_pix = keypointList[n_body].x
    y_pix = keypointList[n_body].y

    dist =  img_np[int(y_pix),int(x_pix)]
    x_pt, y_pt = getCoord(x_pix, y_pix, dist)

    return int(x_pt), int(y_pt), dist

#inputing DepthImg    
def callback2(img_msg) :
    global img_np

    img = msg_to_numpy(img_msg)
    img_np = np.asarray(img)

    ## type : numpy.ndarray
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(img_np, alpha = 0.03), cv2.COLORMAP_JET)
    
    cv2.imshow('rs', depth_colormap)
    cv2.waitKey(1)


def get_keypoint() :
    global PubSpd
    rospy.init_node('get_keypoint', anonymous = False)
    rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList,callback1)
    rospy.Subscriber("depth_image", Image, callback2)
    PubSpd = rospy.Publisher("get_speed", Float32, queue_size=1)
    
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