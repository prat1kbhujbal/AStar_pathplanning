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
