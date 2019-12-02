#!/usr/bin/env python

import rospy
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
def multiply_transforms((trans1, rot1), (trans2, rot2)):
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)
    
    return (trans3, rot3)


def add_to_neighbors(map, idx1, idx2, nb):
    try:
        if map[idx1+1][idx2] == -1:
           map[idx1+1][idx2] = nb + 1
        if map[idx1-1][idx2] == -1:
           map[idx1-1][idx2] = nb + 1
        if map[idx1][idx2+1] == -1:
           map[idx1][idx2+1] = nb + 1
        if map[idx1][idx2-1] == -1:
           map[idx1][idx2-1] = nb + 1
    except:
        # TODO: edge cases
        pass


def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)
    
    mapOfWorld[occupancyGrid==100] = 1 # obstacles
    mapOfWorld[occupancyGrid==-1] = -1  # unknowns
    mapOfWorld[occupancyGrid==0] = -1 

    #import pdb; pdb.set_trace()
    num = 1
    while -1 in mapOfWorld:
        for i in range(mapOfWorld.shape[0]):
            for j in range(mapOfWorld.shape[1]):
                if mapOfWorld[i][j] == num:
                    add_to_neighbors(mapOfWorld, i, j, num)
        num += 1      
    
    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    return mapOfWorld


def brushfire_inv(occupancyGrid):
    brushfire_map = brushfire(occupancyGrid)
    maximum = np.amax(brushfire_map)

    if maximum > 0:
        for i in range(brushfire_map.shape[0]):
            for j in range(brushfire_map.shape[1]):
                brushfire_map[i][j] = maximum - brushfire_map[i][j]
    return brushfire_map


