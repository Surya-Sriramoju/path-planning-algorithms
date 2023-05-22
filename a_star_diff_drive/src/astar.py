
import numpy as np
import heapq as heap
import cv2
import math


def dist_between_nodes(node1, node2):
    return np.sqrt(np.square(node1[0]-node2[0])+np.square(node1[1]-node2[1]))

def node_calc(current_node, actions, step_size, radius, wheel_d, free_points):
    temp = 0
    t_step = 0.1
    X1 = current_node[0]
    Y1 = current_node[1]
    move1 = actions[0]
    move2 = actions[1]
    angle = np.pi * current_node[2]/180
    cost = 0
    while temp<step_size:
        temp = temp + t_step
        T_X = 0.5 * radius * (move1+move2) * np.cos(angle) * t_step
        T_Y = 0.5 * radius * (move1+move2) * np.sin(angle) * t_step
        X1 = X1 + T_X
        Y1 = Y1 + T_Y

        if (round(X1), round(Y1)) not in free_points:
            return (-1,-1,-1), -1
        angle += (radius/wheel_d) * (move2-move1) * t_step
        cost = cost + np.sqrt(np.square((0.5*radius*(move1 + move2)* np.cos(angle)*t_step))+ np.square((0.5*radius*(move1 + move2)* np.sin(angle)*t_step)))
    angle = 180 * angle/np.pi
    return (round(X1), round(Y1), round(angle)), cost


def populate_nodes(current_node, free_points, step_size, actions, radius, wheel_d):
    nodes = []
    for action in actions:
        new_node, cost = node_calc(current_node, action, step_size, radius, wheel_d, free_points)
        x = new_node[0]
        y = new_node[1]
        if (x,y) in free_points:
            nodes.append((new_node, action, cost))
    return nodes


def node_visited(thresh_nodes, current_node, threshold):
    x = current_node[0]//threshold
    y = current_node[1]//threshold
    # print(x,y)
    # print('set: ',thresh_nodes)
    try:
        if (x, y) in thresh_nodes:
            return True
    except:
        return False

def calculate_cost(new_node, current_node, node_cost, queue, parent_nodes, cost, goal_point, threshold, new_action):
    dist = dist_between_nodes(new_node, goal_point)
    new_cost = node_cost[current_node] + cost + dist
    t_cost = node_cost.get(new_node)

    if not t_cost or (t_cost>new_cost):
        node_cost[new_node] = new_cost
        parent_nodes[new_node] = (current_node, new_action)
        heap.heappush(queue, (new_cost, new_node))
    if dist < threshold:
        return True, node_cost, queue, parent_nodes
    return False, node_cost, queue, parent_nodes


def astar(start_point, goal_point, free_points, step_size, threshold, actions, radius, wheel_d):
    visited = []
    queue = []
    node_cost = {}
    parent_nodes = {}
    goal_reached = False
    node_cost[start_point] = 0
    thresh_nodes = set()
    heap.heappush(queue, (0, start_point))

    
    if dist_between_nodes(start_point, goal_point) < threshold:
        goal_reached = True
        parent_nodes[goal_point] = start_point[0:2]
    
    while not goal_reached and queue:
        current_cost, current_node = heap.heappop(queue)
        if node_visited(thresh_nodes, current_node, threshold):
            continue
        visited.append(current_node)
        thresh_nodes.add((current_node[0]//threshold, current_node[1]//threshold))
        # print('set: ',thresh_nodes)
        new_nodes = populate_nodes(current_node, free_points, step_size, actions, radius, wheel_d)
        for new_node, new_action, cost in new_nodes:
            if node_visited(current_node, new_node, threshold):
                continue
            status, node_cost, queue, parent_nodes = calculate_cost(new_node, current_node, node_cost, queue, parent_nodes, cost, goal_point, threshold, new_action)
            print('checking for node: ', new_node[0:2])
            if status:
                thresh_nodes.add((current_node[0]//threshold, current_node[1]//threshold))
                visited.append(new_node)
                goal_reached = True
                break
    return goal_reached, parent_nodes, visited

def getPath(parent_nodes, start_point, goal_point, visited):

    current_node = visited[-1]
    path = [(current_node, (0, 0))]
    while not current_node[0:2] == start_point[0:2]:
        (parent_node, move) = parent_nodes[current_node]
        path.append((parent_node, move))
        current_node = parent_node
    return path[::-1]