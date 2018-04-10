from Tree.QuadrupleTree import QuadrupleTree
import heapq


class BreadthFirstSearch (QuadrupleTree):

    def search(self):
        # Initialize search
        self.frontier.append(self.root)
        result = None

        state_counter = 0
        explored_counter = 0

        while len(self.frontier) > 0:
            # Get head item from queue
            node = self.frontier.pop(0)

            self.add_explored(node)
            explored_counter += 1

            if self.is_goal(node):
                result = node
                break

            state_counter = self.add_neighbours_to_frontier(node, state_counter)

        print('BFS added: %i \t explored: %i \t depth: %i' % (state_counter, explored_counter, result.depth))
        return result


class UniformCostSearch (QuadrupleTree):
    def get_heuristic(self, node):
        return 0

    def search(self):
        # Initialize search
        heapq.heappush(self.frontier, (0, self.root))
        result = None

        state_counter = 0
        explored_counter = 0

        while len(self.frontier) > 0:
            # Get head item from queue
            cost, node = heapq.heappop(self.frontier)

            self.add_explored(node)
            explored_counter += 1

            if self.is_goal(node):
                result = node
                break

            state_counter = self.add_neighbours_to_frontier_queue(node, state_counter)

        print('UFC added: %i \t explored: %i \t depth: %i' % (state_counter, explored_counter, result.depth))
        return result


class DepthFirstSearch (QuadrupleTree):

    def search(self):
        self.frontier.append(self.root)
        result = None

        state_counter = 0
        explored_counter = 0

        while len(self.frontier) > 0:
            node = self.frontier.pop()

            self.add_explored(node)
            explored_counter += 1

            if self.is_goal(node):
                result = node
                break

            state_counter = self.add_neighbours_to_frontier(node, state_counter)

        print('DFS added: %i \t explored: %i \t depth: %i' % (state_counter, explored_counter, result.depth))
        return result


class IterativeDeepeningSearch (QuadrupleTree):

    def depth_first(self, depth, explored_counter, state_counter):
        result = None

        while len(self.frontier) > 0:
            node = self.frontier.pop()

            if node.depth > depth:
                continue

            self.add_explored(node)
            explored_counter += 1

            if self.is_goal(node):
                result = node
                break

            state_counter = self.add_neighbours_to_frontier(node, state_counter)

        return result, explored_counter, state_counter

    def search(self):
        result = None

        state_counter = 0
        explored_counter = 0
        depth = 0

        for depth in range(500):
            self.frontier = []
            self.root.depth = 0
            self.frontier.append(self.root)
            self.explored = {}

            result, explored_counter, state_counter = self.depth_first(depth, explored_counter, state_counter)

            if result:
                break

        print('IDF added: %i \t explored: %i \t depth: %i' % (state_counter, explored_counter, depth))
        return result
