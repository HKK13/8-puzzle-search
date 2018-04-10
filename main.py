import numpy as np
import random
from Tree.QuadrupleTree import Node
from Search.TreeSearch import \
    BreadthFirstSearch, \
    DepthFirstSearch, \
    IterativeDeepeningSearch, \
    UniformCostSearch
from Search.SupervisedTreeSearch import \
    GreedyBestSearch, \
    AStarSearch
import copy


def is_solvable(root):
    inversion_count = 0
    flat_list = root.matrix.flatten().tolist()
    flat_list.remove(0)

    for i in range(8):
        for j in range(i, 8):
            if flat_list[i] > flat_list[j]:
                inversion_count += 1

    return inversion_count % 2 == 0


def main():

    problem_1 = np.array([
        [6, 7, 2],
        [4, 1, 3],
        [0, 8, 5]
    ])
    case_1 = Node(problem_1, None, (0, 2))

    problem_2 = np.array([
        [0, 2, 3],
        [7, 4, 8],
        [1, 5, 6]
    ])
    case_2 = Node(problem_2, None, (0, 0))

    random_numbers = random.sample(range(0, 9), 9)
    initial_problem = np.reshape(random_numbers, (-1, 3))
    y, x = np.where(initial_problem == 0)

    root = Node(initial_problem, None, (x, y))

    while not is_solvable(root):
        random_numbers = random.sample(range(0, 9), 9)
        initial_problem = np.reshape(random_numbers, (-1, 3))
        y, x = np.where(initial_problem == 0)

        root = Node(initial_problem, None, (x, y))

    goal_state = {
        1: (0, 0),
        2: (1, 0),
        3: (2, 0),
        4: (0, 1),
        5: (1, 1),
        6: (2, 1),
        7: (0, 2),
        8: (1, 2),
        0: (2, 2),
    }

    print(root.matrix.flatten().tolist())
    print(root.matrix, is_solvable(root))

    bfs = BreadthFirstSearch(copy.deepcopy(root), goal_state)
    result = bfs.search()
    print('Nope for BFS') if not result else None

    dfs = DepthFirstSearch(copy.deepcopy(root), goal_state)
    result1 = dfs.search()
    print('Nope for DFS') if not result1 else None

    idfs = IterativeDeepeningSearch(copy.deepcopy(root), goal_state)
    result2 = idfs.search()
    print('Nope for IDFS') if not result2 else None

    ucs = UniformCostSearch(copy.deepcopy(root), goal_state)
    result3 = ucs.search()
    print('Nope for UCS') if not result3 else None

    gbs = GreedyBestSearch(copy.deepcopy(root), goal_state)
    result4 = gbs.search()
    print('Nope for GBS') if not result4 else None

    a_s = AStarSearch(copy.deepcopy(root), goal_state)
    result5 = a_s.search()
    print('Nope for A*S') if not result5 else None


if __name__ == '__main__':
    main()
