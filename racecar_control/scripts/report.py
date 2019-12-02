#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from libcontrol import *


def get_points(fname):
    points = []
    with open(fname, 'r') as f:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.split(' ')
            x, y = float(line[0]), float(line[1])

            points.append((x, y))
    return points


def create_report(folder_path, map_2d):
    # points = get_points(folder_path + 'points.txt')
    # test
    points = [(4, 6)]
    origin = (0, 0)

    grid = np.reshape(map_2d.map.data, [map_2d.map.info.height, map_2d.map.info.width])
    brushfire_map = brushfire_inv(grid)
    
    import pdb; pdb.set_trace()

    # TODO

    f = open(folder_path + "report.txt","w+")
    
    for p in points:
        pass
        # f.write("x y :" + str(x[i])+ "," + str(y[i]) + " " + str(image[i]) + " " + str(bitmap[i])+ "\n" )
    f.close()


def main():
    rospy.init_node('report')
    prefix = "racecar"
    rospy.wait_for_service(prefix + '/get_map')
    try:
        get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
        response = get_map()
    except (rospy.ServiceException) as e:
        print "Service call failed: %s"%e
        return
    
    create_report('./report/', response)

if __name__== "__main__":
  main()