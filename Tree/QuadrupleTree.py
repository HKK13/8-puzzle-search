import numpy as np
import heapq


class Node:
    def __init__(self, matrix, parent, blank_coordinates, depth=0, heuristic=0):
        self.matrix = matrix
        self.blank_coordinates = blank_coordinates
        self.parent = parent
        self.depth = depth
        self.heuristic = heuristic

    def get_cost(self):
        depth = self.parent.depth if self.parent else 0
        return depth + self.heuristic

    def __hash__(self):
        flat_copy = self.matrix.flatten()
        flat_matrix_string = "".join(str(x) for x in flat_copy)
        return hash(flat_matrix_string)

    def __eq__(self, other):
        return np.array_equal(self.matrix, other.matrix)

    def __lt__(self, other):
        return (self.depth + self.heuristic) < (other.depth + other.heuristic)


class QuadrupleTree:
    def __init__(self, root, goal_state):
        self.root = root
        self.goal_state = goal_state
        self.frontier = []
        self.explored = {}

    def add_explored(self, node, cost=0):
        self.explored[node] = cost

    def is_explored(self, node):
        return node in self.explored

    def is_goal(self, node):
        keys = list(dict.keys(self.goal_state))
        goal_matrix = np.reshape(keys, (-1, 3))
        return np.array_equal(node.matrix, goal_matrix)

    def get_heuristic(self, node):
        total_manhattan_distance = 0
        for y in range(3):
            for x in range(3):
                goal_x, goal_y = self.goal_state[node.matrix[y][x]]
                total_manhattan_distance += abs(x - goal_x) + abs(y - goal_y)
        return total_manhattan_distance

    def move_blank(self, node, blank_coordinate, destination_coordinate):
        blank_x, blank_y = blank_coordinate
        destination_x, destination_y = destination_coordinate

        new_matrix = np.copy(node.matrix)
        value = new_matrix[destination_y, destination_x]

        new_matrix[blank_y, blank_x] = value
        new_matrix[destination_y, destination_x] = 0

        return Node(new_matrix, node, destination_coordinate)

    def add_neighbours_to_frontier(self, node, state_counter):
        # Store the coordinates of the blank tile AKA 0
        blank_x, blank_y = node.blank_coordinates
        for position in [0, 1]:
            for item in [-1, 1]:
                next_x = blank_x
                next_y = blank_y

                if position == 0 and 3 > blank_x + item >= 0:
                    next_x = blank_x + item
                elif position == 1 and 3 > blank_y + item >= 0:
                    next_y = blank_y + item
                else:
                    continue

                next_node = self.move_blank(node, node.blank_coordinates, (next_x, next_y))
                next_node.depth = next_node.parent.depth + 1

                if next_node in self.explored:
                    continue

                self.frontier.append(next_node)
                state_counter += 1

        return state_counter

    def add_neighbours_to_frontier_queue(self, node, state_counter):
        # Store the coordinates of the blank tile AKA 0
        blank_x, blank_y = node.blank_coordinates
        for position in [0, 1]:
            for item in [-1, 1]:
                next_x = blank_x
                next_y = blank_y

                if position == 0 and 3 > blank_x + item >= 0:
                    next_x = blank_x + item
                elif position == 1 and 3 > blank_y + item >= 0:
                    next_y = blank_y + item
                else:
                    continue

                next_node = self.move_blank(node, node.blank_coordinates, (next_x, next_y))

                next_node.depth = next_node.parent.depth + 1
                next_node.heuristic = self.get_heuristic(next_node)

                if next_node in self.explored:
                    old_cost = self.explored[next_node]
                    if next_node.get_cost() >= old_cost:
                        continue

                """
                if next_node in [pair[1] for pair in self.frontier]:
                    node_tuple = [item for item in self.frontier if item[1] == next_node]
                    old_cost, node = node_tuple[0]

                    if next_node.get_cost() >= old_cost:
                        continue
                """

                priority = int(next_node.get_cost())
                heapq.heappush(self.frontier, (priority, next_node))
                state_counter += 1

        return state_counter

