import pandas as pd
import numpy as np

FileName = 'sklt_Data3'
dataFile = '/content/' + FileName + '.txt'

name_arr = ['time',
            'torso_x', 'torso_y', 'torso_z',
            'left_hip_x', 'left_hip_y', 'left_hip_z',
            'left_knee_x', 'left_knee_y', 'left_knee_z',
            'left_foot_x', 'left_foot_y', 'left_foot_z',
            'right_hip_x', 'right_hip_y', 'right_hip_z',
            'right_knee_x', 'right_knee_y', 'right_knee_z',
            'right_foot_x', 'right_foot_y', 'right_foot_z']

rawData = pd.read_csv(dataFile, index_col=False, sep=' ', names=name_arr)
pd.options.display.float_format = '{:.4f}'.format
rawData.info()

rawData.head(10)

velocity_torso = (rawData['torso_x'][1:].to_numpy() - rawData['torso_x'][:-1].to_numpy()) / (
        rawData['time'][1:].to_numpy() - rawData['time'][:-1].to_numpy())
velocity_torso = np.insert(velocity_torso, 0, 0)  # insert '0' to array[0]

rawData["velocity_torso"] = velocity_torso

# Make Size_Of_left_shin
size_of_shin = np.sqrt(
    (rawData['left_knee_x'] - rawData['left_foot_x']) ** 2 + (rawData['left_knee_y'] - rawData['left_foot_y']) ** 2 + (
            rawData['left_knee_z'] - rawData['left_foot_z']) ** 2
)

rawData["size_of_shin"] = size_of_shin

# vector_(knee,foot)_left

left_shin_x = rawData["left_foot_x"] - rawData["left_knee_x"]
left_shin_y = rawData["left_foot_y"] - rawData["left_knee_y"]
left_shin_z = rawData["left_foot_z"] - rawData["left_knee_z"]
right_shin_x = rawData["right_foot_x"] - rawData["right_knee_x"]
right_shin_y = rawData["right_foot_y"] - rawData["right_knee_y"]
right_shin_z = rawData["right_foot_z"] - rawData["right_knee_z"]

left_thigh_x = rawData["left_hip_x"] - rawData["left_knee_x"]
left_thigh_y = rawData["left_hip_y"] - rawData["left_knee_y"]
left_thigh_z = rawData["left_hip_z"] - rawData["left_knee_z"]
right_thigh_x = rawData["right_hip_x"] - rawData["right_knee_x"]
right_thigh_y = rawData["right_hip_y"] - rawData["right_knee_y"]
right_thigh_z = rawData["right_hip_z"] - rawData["right_knee_z"]

# cosine(theta) = dot(v1,v2)/(norm(v1)*norm(v2))
# cosine(theta_left)
costheta_left = ((left_shin_x * left_thigh_x) + (left_shin_y * left_thigh_y) + (left_shin_z * left_thigh_z)) / \
                (np.sqrt(left_shin_x ** 2 + left_shin_y ** 2 + left_shin_z ** 2) * np.sqrt(
                    left_thigh_x ** 2 + left_thigh_y ** 2 + left_thigh_z ** 2))

# cosine(theta_right)
costheta_right = ((right_shin_x * right_thigh_x) + (right_shin_y * right_thigh_y) + (right_shin_z * right_thigh_z)) / (
        np.sqrt(right_shin_x ** 2 + right_shin_y ** 2 + right_shin_z ** 2) * np.sqrt(
    right_thigh_x ** 2 + right_thigh_y ** 2 + right_thigh_z ** 2))

# calculate theta of knee

rawData['theta_leftknee'] = np.arccos(costheta_left) / np.pi * 180
rawData['theta_rightknee'] = np.arccos(costheta_right) / np.pi * 180

# Make Angle velocity of knee

##left knee
velocity_knee_left = (rawData['theta_leftknee'][1:].to_numpy() - rawData['theta_leftknee'][:-1].to_numpy()) / (
        rawData['time'][1:].to_numpy() - rawData['time'][:-1].to_numpy())
velocity_knee_left = np.insert(velocity_knee_left, 0, 0)  # insert '0' to array[0]
rawData["velocity_knee_left"] = velocity_knee_left

##right knee
velocity_knee_right = (rawData['theta_rightknee'][1:].to_numpy() - rawData['theta_rightknee'][:-1].to_numpy()) / (
        rawData['time'][1:].to_numpy() - rawData['time'][:-1].to_numpy())
velocity_knee_right = np.insert(velocity_knee_right, 0, 0)  # insert '0' to array[0]
rawData["velocity_knee_right"] = velocity_knee_right

# store Data

rawData.to_csv(FileName + 'inform' + '.csv')
