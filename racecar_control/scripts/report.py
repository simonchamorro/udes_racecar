#!/usr/bin/env python
import rospy
import cv2
import copy
import numpy as np
from nav_msgs.srv import GetMap
from libcontrol import *

ROBOT_ORIGIN = (0, 0)

def get_points(fname):
    points = []
    with open(fname, 'r') as f:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.split(' ')
            if not len(line) == 2:
                break
            x, y = float(line[0]), float(line[1])

            points.append((x, y))
    return points


def m_cost(node_a, node_b, grid):
    dist_x = abs(node_a[0] - node_b[0])
    dist_y = abs(node_a[1] - node_b[1])
    assert dist_x <= 1 and dist_y <= 1, "m_cost : %s is not a neighbor of %s"%(str(node_a), str(node_b))
    return grid[node_b[0]][node_b[1]]


def m_neighbors_8(node, obs_map):
    ns = []
    x = node[0]
    y = node[1]
    lx = obs_map.shape[0] - 1
    ly = obs_map.shape[1] - 1
    
    min_x = -1 if (x > 0) else 0
    min_y = -1 if (y > 0) else 0
    max_x = 2 if (x < lx) else 1
    max_y = 2 if (y < ly) else 1

    maximum = np.amax(obs_map)
    
    for dx in range(min_x, max_x):
        for dy in range(min_y, max_y):
            if ((dx == 0) and (dy == 0)):
                continue
            n = (x+dx, y+dy)
            if (obs_map[n] != maximum):
                ns.append(n)
            
    return ns


def astar(start, goal, c_fun, n_fun, h_fun):
    search_set = [start] 
    g = {}
    g[start] = 0
    
    f = {} 
    f[start] = g[start] + h_fun(start, goal)
   
    from_node = {}
    from_node[start] = None
    
    while (len(search_set) > 0):
        min_f = float('inf')
        min_n = None
        for n in search_set:
            assert n in f, "Error : %s not in F!"%(str(n))
            if f[n] < min_f:
                min_f = f[n]
                min_n = n
        
        current = min_n
        print("Current: %s, F(%s) = %d"%(current, current, min_f))
        search_set.remove(min_n)

        if (current == goal):
            path_r = [goal]
            previous = from_node[goal]
            while (previous is not None):
                path_r.append(previous)
                previous = from_node[previous]             
            path_r.reverse() 
            return (path_r, g[goal])
        
        ns = n_fun(current) 
        for n in ns:
            g_n = g[current] + c_fun(current, n)
            f_n = g_n + h_fun(n, goal)

            print("G(%s) = %d, F(%s) = %d"%(n, g_n, n, f_n))
            if ((n not in g) or (g_n < g[n])):
                from_node[n] = current
                g[n] = g_n
                f[n] = f_n
                if (n not in search_set):
                    print("Adding %s"%(str(n)))
                    search_set.append(n)
    return ([], -1)


def m_h(node_a, node_b):
    from math import sqrt
    (ax, ay) = node_a
    (bx, by) = node_b
    return sqrt((ax-bx)**2 + (ay-by)**2)


def find_path(grid, goal, origin, map_origin, map_resolution):
    goal_x = goal[0] - map_origin.position.x
    goal_y = goal[1] - map_origin.position.y
    origin_x = origin[0] - map_origin.position.x
    origin_y = origin[1] - map_origin.position.y

    goal_idx = (int(goal_y // map_resolution), int(goal_x // map_resolution))
    origin_idx = (int(origin_y // map_resolution), int(origin_x // map_resolution))

    # Permet de definir une nouvelle fonction a un seul parametre. Ainsi, m_n(node) correspond a m_neighbors_8(node, obs_map)
    m_n = lambda node : m_neighbors_8(node, grid)
    m_c = lambda node_a, node_b : m_cost(node_a, node_b, grid)
    (seq, cost) = astar(origin_idx, goal_idx, m_c, m_n, m_h)

    grid[goal_idx[0]][goal_idx[1]] = 0
    grid[origin_idx[0]][origin_idx[1]] = 0

    for idx in seq:
        grid[idx[0]][idx[1]] = 0


def draw_path(grid_map, fname):
    maximum = np.amax(grid_map)
    mask_obs = grid_map == maximum
    map_draw = grid_map.astype(float) / float(maximum) *225.0
    map_draw[mask_obs] = 255

    # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    cv2.imwrite(fname, cv2.transpose(cv2.flip(map_draw, -1)))
    rospy.loginfo("Exported " + fname)


def create_report(folder_path, map_2d):
    points = get_points(folder_path + 'points.txt')
    origin = ROBOT_ORIGIN

    grid = np.reshape(map_2d.map.data, [map_2d.map.info.height, map_2d.map.info.width])
    brushfire_map = brushfire_inv(grid)    

    f = open(folder_path + "report.txt","w+")
    
    obj_num = 1
    for p in points:
        cost_map = copy.copy(brushfire_map)
        map_fname = folder_path + 'trajectory_object_' + str(obj_num) + '.bmp'
        find_path(cost_map, p, origin, map_2d.map.info.origin, map_2d.map.info.resolution)
        draw_path(cost_map, map_fname)
        f.write(str(p[0]) + " " + str(p[1]) + " photo_object_" + str(obj_num) + \
            ".png trajectory_object_" + str(obj_num) + ".bmp\n" )
        obj_num += 1
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