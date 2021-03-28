import sys

sys.path.insert(0, "../")
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
from A_Star import AStar


class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return str((self.x, self.y))


class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location.x) + str(self.location.y))

    def is_equal_except_time(self, state):
        return self.location == state.location

    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    VERTEX = 1
    EDGE = 2

    def __init__(self):
        self.time = -1
        self.type = -1

        self.robot_1 = ""
        self.robot_2 = ""

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return (
            "("
            + str(self.time)
            + ", "
            + self.robot_1
            + ", "
            + self.robot_2
            + ", "
            + str(self.location_1)
            + ", "
            + str(self.location_2)
            + ")"
        )


class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location))

    def __str__(self):
        return "(" + str(self.time) + ", " + str(self.location) + ")"


class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2

    def __eq__(self, other):
        return (
            self.time == other.time
            and self.location_1 == other.location_1
            and self.location_2 == other.location_2
        )

    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))

    def __str__(self):
        return (
            "("
            + str(self.time)
            + ", "
            + str(self.location_1)
            + ", "
            + str(self.location_2)
            + ")"
        )


class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return (
            "VC: "
            + str([str(vc) for vc in self.vertex_constraints])
            + "EC: "
            + str([str(ec) for ec in self.edge_constraints])
        )


class Environment(object):
    def __init__(self, dimension, robots, obstacles, robots_time):
        self.dimension = dimension
        self.obstacles = obstacles
        self.robots_time = robots_time

        self.robots = robots
        self.robot_dict = {}

        self.make_robot_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y + 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y - 1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x - 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x + 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors

    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for robot_1, robot_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(robot_1, solution, t)
                state_2 = self.get_state(robot_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.robot_1 = robot_1
                    result.robot_2 = robot_2
                    return result

            for robot_1, robot_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(robot_1, solution, t)
                state_1b = self.get_state(robot_1, solution, t + 1)

                state_2a = self.get_state(robot_2, solution, t)
                state_2b = self.get_state(robot_2, solution, t + 1)

                if state_1a.is_equal_except_time(
                    state_2b
                ) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.robot_1 = robot_1
                    result.robot_2 = robot_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.robot_1] = constraint
            constraint_dict[conflict.robot_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(
                conflict.time, conflict.location_1, conflict.location_2
            )
            e_constraint2 = EdgeConstraint(
                conflict.time, conflict.location_2, conflict.location_1
            )

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.robot_1] = constraint1
            constraint_dict[conflict.robot_2] = constraint2

        return constraint_dict

    def get_state(self, robot_name, solution, t):
        if t < len(solution[robot_name]):
            return solution[robot_name][t]
        else:
            return solution[robot_name][-1]

    def state_valid(self, state):
        return (
            state.location.x >= 0
            and state.location.x < self.dimension[0]
            and state.location.y >= 0
            and state.location.y < self.dimension[1]
            and VertexConstraint(state.time, state.location)
            not in self.constraints.vertex_constraints
            and (state.location.x, state.location.y) not in self.obstacles
        )

    def transition_valid(self, state_1, state_2):
        return (
            EdgeConstraint(state_1.time, state_1.location, state_2.location)
            not in self.constraints.edge_constraints
        )

    def is_solution(self, robot_name):
        pass

    def admissible_heuristic(self, state, robot_name):
        goal = self.robot_dict[robot_name]["Final"]
        return fabs(state.location.x - goal.location.x) + fabs(
            state.location.y - goal.location.y
        )

    def is_at_goal(self, state, robot_name):
        goal_state = self.robot_dict[robot_name]["Final"]
        return state.is_equal_except_time(goal_state)

    def make_robot_dict(self):
        for robot in self.robots:

            start_state = State(
                self.robots_time[robot["Name"]],
                Location(robot["Initial"][0], robot["Initial"][1]),
            )
            goal_state = State(
                self.robots_time[robot["Name"]],
                Location(robot["Final"][0], robot["Final"][1]),
            )

            self.robot_dict.update(
                {robot["Name"]: {"Initial": start_state, "Final": goal_state}}
            )

    def compute_solution(self):
        solution = {}
        for robot in self.robot_dict.keys():
            self.constraints = self.constraint_dict.setdefault(robot, Constraints())
            local_solution = self.a_star.search(robot)
            if not local_solution:
                return False
            solution.update({robot: local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])


class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)):
            return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost


class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()

    def search(self):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for robot in self.env.robot_dict.keys():
            start.constraint_dict[robot] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = {}
            if not conflict_dict:
                print("Result: ")

                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for robot in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[robot].add_constraint(constraint_dict[robot])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution):
        plan = {}
        for robot, path in solution.items():
            path_dict_list = [
                {"t": state.time, "x": state.location.x, "y": state.location.y}
                for state in path
            ]
            plan[robot] = path_dict_list
        return plan


def main():
    with open(r"D:\AIFA Assignment 1\Input.yaml") as param_file: #Change location of Input file
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["Map"]["Dimension"]
    obstacles = param["Map"]["Obstacles"]
    robots = param["Robots"]

    new_robots = []
    robot_name = []
    remaining_robots = []
    robot_time = {}
    time = 0
    k = 1

    while len(robots) > 0:
        for robot in robots:
            if robot["Name"] not in robot_name:
                new_robots.append(robot)
                robot_name.append(robot["Name"])
                if robot["Name"] not in robot_time:
                    robot_time[robot["Name"]] = 0
            else:
                remaining_robots.append(robot)

        robots = remaining_robots.copy()

        print("Working: ")

        print(new_robots)
        print(robot_name)
        print(robots)

        env = Environment(dimension, new_robots, obstacles, robot_time)

        cbs = CBS(env)
        solution = cbs.search()
        if not solution:
            print("Result Not Found: ")
            return

        with open(r"D:\AIFA Assignment 1\Input.yaml") as output_yaml:  #Change location of Input file
            try:
                output = yaml.load(output_yaml, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        output["Schedule"] = solution
        output["Cost"] = env.compute_solution_cost(solution)
        time = time + output["Cost"]

        for robot_end in output["Schedule"]:
            robot_time[robot_end] = output["Schedule"][robot_end][-1]["t"]

        print(output)

        revised_output = output["Schedule"]

        if k == 1:
            with open(r"D:\AIFA Assignment 1\Output.yaml", "w") as output_yaml:  #Change location of Output file
                yaml.safe_dump(revised_output, output_yaml)
        else:
            with open(r"D:\AIFA Assignment 1\Output.yaml", "a") as output_yaml:  #Change location of Output file
                yaml.safe_dump(revised_output, output_yaml)

        new_robots.clear()
        robot_name.clear()
        remaining_robots.clear()
        k += 1


if __name__ == "__main__":
    main()
