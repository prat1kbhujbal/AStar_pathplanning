import argparse
import numpy as pb
from obstacle_plot import *
from astar import *


def solvable(start_node, goal_node, map):
    """Check if goal/start node inside the obstacle or outside of map"""
    if start_node[0] > 400 or start_node[0] < 0 or start_node[1] > 250 or start_node[
            1] < 0 or goal_node[0] > 400 or goal_node[0] < 0 or goal_node[1] > 250 or goal_node[1] < 0:
        print("Start Node/Goal Node is outside the Map!! Please provide valid nodes.")
        return False
    if map[start_node[0],
           start_node[1]] == 1 or map[goal_node[0],
                                      goal_node[1]] == 1:
        print("Start Node/Goal Node is inside the obstacle!! Please provide valid nodes.")
        return False
    return True


def main():
    """Main Function"""
    Parser = argparse.ArgumentParser()
    Parser.add_argument(
        '--start', nargs='+', type=int, default=[16, 16, 0],
        help='start node. Default: [16, 16, 0]')
    Parser.add_argument(
        '--goal', nargs='+', type=int, default=[317, 51, 0],
        help='goal node. Default: [317, 51, 0]')
    Parser.add_argument(
        '--robot_radius', type=int, default=10,
        help='Robot Radius. Default: 10')
    Parser.add_argument(
        '--clearance', type=int, default=5,
        help='Obstacle clearance. Default: 5')
    Parser.add_argument(
        '--step_size', type=int, default=10,
        help='Step Size. Default: 10')
    Parser.add_argument(
        "--visualize",
        default=True,
        choices=('True', 'False'),
        help="Shows visualization after goal reached. Default: True.")

    Args = Parser.parse_args()
    start_node = Args.start
    goal_node = Args.goal
    show_animation = str(Args.visualize)
    robot_radius = Args.robot_radius
    clearance = Args.clearance
    step_size = Args.step_size

    # Map Grid
    map_grid = [400, 250]
    map = pb.zeros((map_grid[0], map_grid[1]), pb.uint8)
    total_clearance = robot_radius + clearance
    plot_grid(start_node, goal_node, map_grid)
    map = obstacle(map, total_clearance)
    # Check for valid goal and start position
    if solvable(start_node, goal_node, map):
        # Perfrom Astar
        astar(
            start_node,
            goal_node,
            map,
            step_size,
            total_clearance,
            show_animation)


if __name__ == "__main__":
    main()
