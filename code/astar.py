import numpy as pb
import matplotlib.pyplot as plt
from queue import PriorityQueue
from obstacle_plot import *


class Astar:
    """Class for AStar
    """

    def __init__(self, position, cost, parent):
        self.position = position
        self.cost = cost
        self.parent = parent


def verify_node(node, clearance):
    """Check for borders with robot and obstacle clearance

    Args:
        node: Current Node
        clearance (int): Total Clearance

    Returns:
        Bool: boolean value
    """
    px = node[0]
    py = node[1]

    if px < clearance:
        return False
    if py < clearance:
        return False
    if px >= (400 - clearance):
        return False
    if py >= (250 - clearance):
        return False
    return True


def movem30(th):
    """Angle to move 30 degree clockwise

    Args:
        th (int): angle

    Returns:
        int : new angle
    """
    new_th = th - 30
    if new_th <= -360:
        new_th = 360 + new_th
    return new_th


def move30(th):
    """Angle to move 30 degree counter-clockwise

    Args:
        th (int): angle

    Returns:
        int : new angle
    """
    new_th = th + 30
    if new_th >= 360:
        new_th = new_th - 360
    return new_th


def movem60(th):
    """Angle to move 60 degree clockwise

    Args:
        th (int): angle

    Returns:
        int : new angle
    """
    new_th = th - 60
    if new_th <= -360:
        new_th = 360 + th
    return new_th


def move60(th):
    """Angle to move 60 degree counter-clockwise

    Args:
        th (int): angle

    Returns:
        int : new angle
    """
    new_th = th + 60
    if new_th >= 360:
        new_th = new_th - 360
    return new_th


def actions(px, py, th, step_size):
    """Explore paths

    Args:
        px (float): current node x position
        py (float): current node y position
        th (int): current node angle
        step_size (int): step size

    Returns:
        float array: Explored paths
    """
    '''Explore paths'''
    actions = [
        (px + (step_size * pb.cos(pb.radians(movem30(th)))),
         py + (step_size * pb.sin(pb.radians(movem30(th)))),
         movem30(th)),
        (px + (step_size * pb.cos(pb.radians(move30(th)))),
         py + (step_size * pb.sin(pb.radians(move30(th)))),
         move30(th)),
        (px + (step_size * pb.cos(pb.radians(movem60(th)))),
         py + (step_size * pb.sin(pb.radians(movem60(th)))),
         movem60(th)),
        (px + (step_size * pb.cos(pb.radians(move60(th)))),
         py + (step_size * pb.sin(pb.radians(move60(th)))),
         move60(th)),
        (px + (step_size * pb.cos(pb.radians(th))),
         py + (step_size * pb.sin(pb.radians(th))),
         th)]
    return actions


def astar(start, goal, map, step_size, clearance, animation):
    """ AStar Algorithm
    Args:
        start : Start position
        goal : Goal position
        map : Updated Map
        step_size (int): step size
        clearance (int): Total clearance
        animation (bool): To visualize
    """
    narray = pb.array([[[pb.inf for k in range(12)]
                        for j in range(int(250 / 0.5))]
                       for i in range(int(400 / 0.5))])
    eplr_x = []
    eplr_y = []
    # Goal tolerance
    goal_tolerance = 1.5
    pque = PriorityQueue()
    start_node = Astar(start, 0, None)
    goal_node = Astar(goal, 0, None)
    uniquebits = 0
    pque.put((start_node.cost, uniquebits, start_node))
    while not pque.empty():
        current_node = pque.get()
        node = current_node[2]
        if goal_reached(node, goal_node, goal_tolerance):
            print("Goal Reached !!")
            goal_node.parent = node.parent
            goal_node.cost = node.cost
            break
        else:
            explortion = planning(node, map, goal_node, step_size, clearance)
            for i in explortion:
                explore_pos = i.position
                if visited_nodes(i, goal_node, narray):
                    narray[int((round(2 * explore_pos[0]) / 2) / 0.5),
                           int((round(2 * explore_pos[1]) / 2) / 0.5),
                           int((round(2 * explore_pos[2]) / 2) / 30)] = i.cost + (
                        heuristic(explore_pos, goal_node))
                    eplr_x.append([node.position[0], node.position[1]])
                    eplr_y.append(
                        [(explore_pos[0] - node.position[0]),
                         (explore_pos[1] - node.position[1])])

                    ncost = i.cost + heuristic(explore_pos, goal_node)
                    pque.put((ncost, uniquebits, i))
                    uniquebits += 1

    if animation == str(True):
        print("Showing Exploration")
        for i in range(len(eplr_x)):
            plt.arrow(eplr_x[i][0],
                      eplr_x[i][1],
                      eplr_y[i][0],
                      eplr_y[i][1],
                      head_width=1,
                      head_length=1,
                      fc='k',
                      ec='red',
                      )
            plt.pause(0.00000000000000001)
        plt.arrow(
            int(node.parent.position[0]),
            int(node.parent.position[1]),
            (goal_node.position[0] - int(node.parent.position[0])),
            (goal_node.position[1] - int(node.parent.position[1])),
            head_width=0.5,
            head_length=0.001,
            fc='k',
            ec='red',
        )
    goal_node = goal_node
    path_x = []
    path_y = []
    while goal_node:
        path_x.append(goal_node.position[0])
        path_y.append(goal_node.position[1])
        goal_node = goal_node.parent
        plt.plot(path_x, path_y, "-k", markersize=5)
        plt.pause(0.00001)
    plt.show()
