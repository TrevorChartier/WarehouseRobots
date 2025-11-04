import copy
from queue import PriorityQueue, Queue
import time
from visualize import display_grid

class InvalidActionError(Exception):
    """Robot performs an invalid action"""
    pass


class Problem(object):
    def __init__(self):
        self.create_state_space()
        self.set_initial_state()
        self.create_actions()

    def create_state_space(self):
        """
        This defines the static information about the environment needed to complete the task.
        """
        self.grid_size = (10, 15)  # (rows, cols)
        # Static locations the robots cannot move to
        self.obstacles = [(4, 3), (3, 8), (5, 11), (7,7), (7,8), (6,13), (6,14)]
        # Static goal locations for target shelves
        self.packing_stations = [(0, 3), (0, 10)]
        # ID's of the shelves to be brought to packing stations
        self.target_shelves = [6, 14]

    def set_initial_state(self):
        """This function defines the initial state of the system"""
        self.initial_state = {
            'robot1': {
                'position': (3, 0),
                'shelf_carried': None,
            },
            'robot2': {
                'position': (7, 0),
                'shelf_carried': None,
            },
            'shelf_positions': {
                1: (7, 3), 2: (7, 4), 3: (7, 5), 4: (7, 6),
                5: (9, 3), 6: (9, 4), 7: (9, 5), 8: (9, 6),
                9: (7, 9), 10: (7, 10), 11: (7, 11), 12: (7, 12),
                13: (9, 9), 14: (9, 10), 15: (9, 11), 16: (9, 12)
            }
        }

    def create_actions(self):
        """ Defines the set of possible actions """
        self.actions = {
            'robot1': ['up', 'down', 'left', 'right', 'pickup', 'putdown', 'none'],
            'robot2': ['up', 'down', 'left', 'right', 'pickup', 'putdown', 'none'],
        }

    def goal_test(self, state):
        """
        Returns true if the state passed is a goal state.

        This occurs when both target shelves are at a packing station
        """
        return (
            state['shelf_positions'][self.target_shelves[0]
                                     ] in self.packing_stations
            and state['shelf_positions'][self.target_shelves[1]] in self.packing_stations
        )

    def is_goal(self, state):
        """
        Returns true if the state arg is a goal state.

        This occurs when both target shelves are at a packing station
        """
        return self.goal_test(state)

    def get_cost(self, action):
        """ Uniform cost of 2 per action """
        return 2

    def heuristic(self, state, ucs_flag=False):
        if ucs_flag:
            return 0
        else:
            return 2.3 * self.your_heuristic_function(state)

    def your_heuristic_function(self, state):
        robot1_pos = state['robot1']['position']
        robot2_pos = state['robot2']['position']
        shelf1_pos = state['shelf_positions'][self.target_shelves[0]]
        shelf2_pos = state['shelf_positions'][self.target_shelves[1]]

        heuristic_val = (
            min(
                self.__manhatt_dist(robot1_pos, shelf1_pos)
                + self.__manhatt_dist(robot2_pos, shelf2_pos),

                self.__manhatt_dist(robot2_pos, shelf1_pos)
                + self.__manhatt_dist(robot1_pos, shelf2_pos)
            )  # Shortest way to get both robots to the target shelves
            + min(
                self.__manhatt_dist(shelf1_pos, self.packing_stations[0])
                + self.__manhatt_dist(shelf2_pos, self.packing_stations[1]),

                self.__manhatt_dist(shelf2_pos, self.packing_stations[0])
                + self.__manhatt_dist(shelf1_pos, self.packing_stations[1]),
            )  # Shortest way to get both target shelves to packing stations
        )
        
        pickups_needed = 2
        shelves_carried = [state['robot1']['shelf_carried'], state['robot2']['shelf_carried']]
        if self.target_shelves[0] in shelves_carried:
            pickups_needed -= 1
        if self.target_shelves[1] in shelves_carried:
            pickups_needed -= 1

        heuristic_val += pickups_needed
        
        return heuristic_val

    def get_successors(self, state):
        """ Return a list of all possible successive states from the given state """
        successors = []
        for robot1_action in self.actions['robot1']:
            for robot2_action in self.actions['robot2']:
                try:
                    result_state = self.__get_result_state(
                        state, robot1_action, "robot1")
                    result_state = self.__get_result_state(
                        result_state, robot2_action, "robot2")
                    action_str = f"Robot 1: {robot1_action}\nRobot 2: {robot2_action}"
                    successors.append((result_state, action_str))
                except InvalidActionError:
                    continue # Skip actions that cannot be performed
        return successors


    def __get_result_state(self, state, robot_action, robot_id):
        """
        Returns the state that results from the robot with specified ID performing given action

        Throws a InvalidActionError if the action cannot be performed.
        """
        result_state = copy.deepcopy(state)
        if robot_action == 'none':
            return result_state

        if robot_action == 'pickup':
            if state[robot_id]['shelf_carried'] is not None:
                raise InvalidActionError(
                    "Attempting to pickup shelf while already carrying one")

            # Check for a shelf at the robots current position
            for shelf_id, pos in state['shelf_positions'].items():
                if pos == state[robot_id]['position']:
                    result_state[robot_id]['shelf_carried'] = shelf_id
                    return result_state
            raise InvalidActionError(
                "Attempting to pickup a shelf while not located under one")

        if robot_action == 'putdown':
            if state[robot_id]['shelf_carried'] is None:
                raise InvalidActionError(
                    "Attempting to put down a shelf, but the robot is not carrying one")
            result_state[robot_id]['shelf_carried'] = None
            return result_state

        else:  # Remaining actions are move actions
            result_state[robot_id]['position'] = self.__apply_move(
                robot_action, state[robot_id]['position'])

            # If carrying a shelf, update that shelf's position too
            shelf_carried = state[robot_id]['shelf_carried']
            if shelf_carried is not None:
                result_state['shelf_positions'][shelf_carried] = self.__apply_move(
                    robot_action, state['shelf_positions'][shelf_carried])

            if not self.__is_valid(result_state):
                raise InvalidActionError(
                    "Attempting a Move that Results in an Invalid State")
            return result_state

    def __apply_move(self, move_direction, curr_position):
        match move_direction:
            case 'up':
                return (curr_position[0] - 1, curr_position[1])
            case 'right':
                return (curr_position[0], curr_position[1] + 1)
            case 'down':
                return (curr_position[0] + 1, curr_position[1])
            case 'left':
                return (curr_position[0], curr_position[1] - 1)

    def __manhatt_dist(self, pos1, pos2):
        return abs(pos2[0] - pos1[0]) + abs(pos2[1] - pos1[1])

    def __is_valid(self, state):
        """Returns a boolean indicating if the specified state is allowed"""
        robot1_pos = state['robot1']['position']
        robot2_pos = state['robot2']['position']
        # Both robots need to be in the grid
        if not (self.__in_bounds(robot1_pos) and self.__in_bounds(robot2_pos)):
            return False

        # Robots cannot be in the same cell
        if robot1_pos == robot2_pos:
            return False

        # Robots cannot be in the same cell as an obstacle
        if (robot1_pos in self.obstacles or robot2_pos in self.obstacles):
            return False

        # Two shelves cannot be in same cell simultaneously
        positions_list = list(state['shelf_positions'].values())
        if len(positions_list) != len(set(positions_list)):
            return False

        return True

    def __in_bounds(self, coordinate):
        """ Returns a boolean indicating whether the coordinate (row, col) tuple is 
        in the grid"""

        num_rows, num_cols = self.grid_size
        
        if 0 <= coordinate[0] < num_rows and 0 <= coordinate[1] < num_cols:
            return True
        return False


class Node(object):
    def __init__(self, state, parent_node, action):
        ''' Feel free to add any additional arguments you need'''
        self.parent_node = parent_node
        self.action = action # Action required to get to node from parent node
        self.state = state
        if parent_node is None:
            self.total_cost = 0
        else:
            self.total_cost = parent_node.get_path_cost() + 2

    def get_plan(self):
        ''' Return the plan to reach self from the start state'''
        plan = []
        curr_node = self
        while curr_node is not None:
            plan.insert(0, curr_node.state)  # insert state at front
            curr_node = curr_node.parent_node
        return plan


    def get_path_cost(self):
        ''' Return the path cost to reach self from the start state'''
        return self.total_cost

    def expand(self, problem):
        ''' Return all children nodes'''
        successor_states = problem.get_successors(self.state)

        return [Node(state[0], self, state[1]) for state in successor_states]


def astar_graph_search(problem, ucs_flag=False):
    counter = 0
    start_node = Node(problem.initial_state, None, "Start")
    fringe = PriorityQueue()
    closed = set()
    fringe.put((problem.heuristic(start_node.state, ucs_flag), counter, start_node))
    counter += 1
    while not fringe.empty():
        curr_node = fringe.get()[2]
        
        if problem.is_goal(curr_node.state):
            return curr_node
        if get_state_id(curr_node.state) not in closed:
            closed.add(get_state_id(curr_node.state))
            for child_node in curr_node.expand(problem):
                f = child_node.get_path_cost() + problem.heuristic(child_node.state, ucs_flag)
                fringe.put((f, counter, child_node))
                counter += 1
    print("Error: Could Not Find Goal")

def get_state_id(state):
    return (
        state['robot1']['position'],
        state['robot2']['position'],
        frozenset(state['shelf_positions'].items()),
        state['robot1']['shelf_carried'],
        state['robot2']['shelf_carried']
    )

if __name__ == "__main__":
    problem = Problem()
    node = astar_graph_search(problem)
    plan = node.get_plan()


    # Animate
    display_grid(plan[0], problem)
    time.sleep(5)
    for state in plan:
        display_grid(state, problem)
        time.sleep(1.5)

