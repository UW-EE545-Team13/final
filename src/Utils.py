#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, \
    PoseWithCovarianceStamped, PointStamped
import tf.transformations
import tf
import matplotlib.pyplot as plt


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


def world_to_map(pose, map_info):
    # equivalent to map_to_grid(world_to_map(pose))
    # operates in place
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)
    config = [0.0, 0.0]
    # translation
    config[0] = (1.0 / float(scale)) * (pose[0] - map_info.origin.position.x)
    config[1] = (1.0 / float(scale)) * (pose[1] - map_info.origin.position.y)

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(config[0])
    config[0] = int(c * config[0] - s * config[1])
    config[1] = int(s * temp + c * config[1])

    return config


def map_to_world(pose, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)

    # rotate
    config = np.array([pose[0], map_info.height - pose[1], pose[2]])
    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(config[0])
    config[0] = c * config[0] - s * config[1]
    config[1] = s * temp + c * config[1]

    # scale
    config[:2] *= float(scale)

    # translate
    config[0] += map_info.origin.position.x
    config[1] += map_info.origin.position.y
    config[2] += angle

    return config

''' 
Convert array of poses in the world to pixel locations in the map image 
  pose: The poses in the world to be converted. Should be a nx3 numpy array
  map_info: Info about the map (returned by get_map)
'''    
def world_to_map_pixel(poses, map_info):
   
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)

    # Translation
    poses[:,0] -= map_info.origin.position.x
    poses[:,1] -= map_info.origin.position.y

    # Scale
    poses[:,:2] *= (1.0/float(scale))

    # Rotation
    c, s = np.cos(angle), np.sin(angle)
    
    # Store the x coordinates since they will be overwritten
    temp = np.copy(poses[:,0])
    poses[:,0] = c*poses[:,0] - s*poses[:,1]
    poses[:,1] = s*temp       + c*poses[:,1]
    poses[:,2] += angle
