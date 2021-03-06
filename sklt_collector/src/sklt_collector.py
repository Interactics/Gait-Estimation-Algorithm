#!/usr/bin/env python
import roslib
import rospy
import tf
from tf2_msgs.msg import TFMessage

flag = False
time_sec = 0
file = open("./sklt_Data.txt", 'w')

def sklt_collector():
    rospy.init_node('sklt_collector', anonymous=False)
    rospy.Subscriber("/tf", TFMessage,callback)
    rospy.spin()
 
def callback(data):
    #rospy.loginfo("Pose : %s", data)
    global flag, time_sec
    print(data.transforms[0].header.stamp.nsecs)
    if (flag == False) :
        time_sec = data.transforms[0].header.stamp.secs
        flag = True

    time = data.transforms[0].header.stamp.secs - time_sec

    print (time)
    if (data.transforms[0].child_frame_id == 'torso_1') : 
        print(str(time) + '.' + str(data.transforms[0].header.stamp.nsecs))
        time_size = len(str(data.transforms[0].header.stamp.nsecs))
        file.write(str(time)+ '.')
        if time_size < 9 :
            for i in range(0,9-time_size):
                file.write(str('0'))
            file.write(str(data.transforms[0].header.stamp.nsecs))
        else :
            file.write(str(data.transforms[0].header.stamp.nsecs))

        print ("torso")
        print(data.transforms[0].transform.translation)
        #file.write(str(time)+ '.'+ str(data.transforms[0].header.stamp.nsecs)
        file.write(" " + str(data.transforms[0].transform.translation.x) + " " + str(data.transforms[0].transform.translation.y) + " " + str(data.transforms[0].transform.translation.z))

    if (data.transforms[0].child_frame_id == 'left_knee_1') :
        print ("left knee")
        print(data.transforms[0].transform.translation)
        file.write(" " + str(data.transforms[0].transform.translation.x) + " " +
        str(data.transforms[0].transform.translation.y) + " " +str(data.transforms[0].transform.translation.z))
    
    if (data.transforms[0].child_frame_id == 'right_knee_1') :
        print ("right knee : ")
        print(data.transforms[0].transform.translation)
        file.write(" " + str(data.transforms[0].transform.translation.x) + " " +
        str(data.transforms[0].transform.translation.y) + " " +str(data.transforms[0].transform.translation.z))

    if (data.transforms[0].child_frame_id == 'left_hip_1') :
        print ("left hip : ")
        print(data.transforms[0].transform.translation)
        file.write(" " + str(data.transforms[0].transform.translation.x) + " " +
        str(data.transforms[0].transform.translation.y) + " " +str(data.transforms[0].transform.translation.z))
    if (data.transforms[0].child_frame_id == 'right_hip_1') :
        print ("right hip : ")
        print(data.transforms[0].transform.translation)
        file.write(" " + str(data.transforms[0].transform.translation.x) + " " +
        str(data.transforms[0].transform.translation.y) + " " +str(data.transforms[0].transform.translation.z))
    if (data.transforms[0].child_frame_id == 'left_foot_1') :
        print ("left foot : ")
        print(data.transforms[0].transform.translation)
        file.write(" " + str(data.transforms[0].transform.translation.x) + " " +
        str(data.transforms[0].transform.translation.y) + " " +str(data.transforms[0].transform.translation.z))
    if (data.transforms[0].child_frame_id == 'right_foot_1') :
        print ("right foot : ")
        print(data.transforms[0].transform.translation)
        file.write(" " + str(data.transforms[0].transform.translation.x) + " " +
        str(data.transforms[0].transform.translation.y) + " " +str(data.transforms[0].transform.translation.z) +"\n")

        

    #print(data.transforms[0].transform.translation)

if __name__ == '__main__' :
    sklt_collector()