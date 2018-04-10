from Tree.QuadrupleTree import QuadrupleTree
import heapq


class GreedyBestSearch(QuadrupleTree):
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
                    if next_node.heuristic >= old_cost:
                        continue

                """
                if next_node in [pair[1] for pair in self.frontier]:
                    node_tuple = [item for item in self.frontier if item[1] == next_node]
                    old_cost, node = node_tuple[0]

                    if next_node.get_cost() >= old_cost:
                        continue
                """

                priority = int(next_node.heuristic)
                heapq.heappush(self.frontier, (priority, next_node))
                state_counter += 1

        return state_counter

    def search(self):
        # Initialize search
        heapq.heappush(self.frontier, (0, self.root))
        result = None

        state_counter = 0
        explored_counter = 0

        while len(self.frontier) > 0:
            # Get head item from queue
            cost, node = heapq.heappop(self.frontier)

            if node in self.explored:
                continue

            self.add_explored(node)
            explored_counter += 1

            if self.is_goal(node):
                result = node
                break

            state_counter = self.add_neighbours_to_frontier_queue(node, state_counter)

        print('GBS added: %i \t explored: %i \t depth: %i' % (state_counter, explored_counter, result.depth))
        return result


class AStarSearch(QuadrupleTree):

    explored = {}

    def is_cheaper(self, node, old_node=None):
        if not old_node:
            return False
        return old_node.get_cost() > node.get_cost()

    def add_explored(self, node, cost=0):
        self.explored[node] = cost

    def search(self):
        # Initialize search
        heapq.heappush(self.frontier, (0, self.root))
        result = None

        state_counter = 0
        explored_counter = 0

        while len(self.frontier) > 0:
            # Get head item from queue
            cost, node = heapq.heappop(self.frontier)

            if self.is_goal(node):
                result = node
                break

            self.add_explored(node, node.get_cost())
            explored_counter += 1

            state_counter = self.add_neighbours_to_frontier_queue(node, state_counter)

        print('A*S added: %i \t explored: %i \t depth: %i' % (state_counter, explored_counter, result.depth))
        return result
