import math
testing_graph = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
testing_step = int(math.sqrt(25))
A = [0, 2]
B = [3, 0]


def a_star_algorithm(grid, step, start, goal):
    # open_list is a list of nodes which have been visited, but who's neighbors
    # haven't all been inspected, starts off with the start node
    # closed_list is a list of nodes which have been visited
    # and who's neighbors have been inspected
    def heuristic(vertex):
        return abs(vertex[0] - goal[0]) + abs(vertex[1] - goal[1]) \
            + abs(grid[vertex[0]][vertex[1]] - grid[goal[0]][goal[1]])
    start = tuple(start)
    goal = tuple(goal)
    open_list = set([start])
    closed_list = set([])

    # g contains current distances from start_node to all other nodes
    # the default value (if it's not found in the map) is +infinity
    distance_to_start = {}

    distance_to_start[start] = 0

    # parents contains an adjacency map of all nodes
    parents = {}
    parents[start] = start

    while len(open_list) > 0:
        vertex = None

        # find a node with the lowest value of f() - evaluation function
        for candidate in open_list:
            if not vertex or distance_to_start[candidate] \
                    + heuristic(candidate) < distance_to_start[vertex] + heuristic(vertex):
                vertex = candidate

        if vertex == goal:
            reconst_path = []

            while parents[vertex] != vertex:
                reconst_path.append(vertex)
                vertex = parents[vertex]

            reconst_path.append(start)
            reconst_path.reverse()
            return reconst_path

        candidates = []
        if vertex[0] > 0:
            candidates.append(
                [(vertex[0]-1, vertex[1]), grid[vertex[0]-1][vertex[1]]])
        if vertex[1] > 0:
            candidates.append(
                [(vertex[0], vertex[1]-1), grid[vertex[0]][vertex[1]-1]])
        if vertex[0] < len(grid)-1:
            candidates.append(
                [(vertex[0]+1, vertex[1]), grid[vertex[0]+1][vertex[1]]])
        if vertex[1] < len(grid[0])-1:
            candidates.append(
                [(vertex[0], vertex[1]+1), grid[vertex[0]][vertex[1]+1]])
        # for all neighbors of the current node do
        for (candidate, height) in candidates:
            # if the current node isn't in both open_list and closed_list
            # add it to open_list and note n as it's parent
            if candidate not in open_list and candidate not in closed_list:
                open_list.add(candidate)
                parents[candidate] = vertex
                distance_to_start[candidate] = distance_to_start[vertex] + \
                    math.sqrt(step**2+(height-grid[vertex[0]][vertex[1]])**2)

            # otherwise, check if it's quicker to first visit n, then m
            # and if it is, update parent data and g data
            # and if the node was in the closed_list, move it to open_list
            else:
                if distance_to_start[candidate] > distance_to_start[vertex] \
                        + math.sqrt(step**2+(height-grid[vertex[0]][vertex[1]])**2):
                    distance_to_start[candidate] = distance_to_start[vertex] + \
                        math.sqrt(
                            step**2+(height-grid[vertex[0]][vertex[1]])**2)
                    parents[candidate] = vertex

                    if candidate in closed_list:
                        closed_list.remove(candidate)
                        open_list.add(candidate)

        # remove n from the open_list, and add it to closed_list
        # because all of his neighbors were inspected
        open_list.remove(vertex)
        closed_list.add(vertex)

    print('Path does not exist!')
    return None


if __name__ == '__main__':
    print(a_star_algorithm(testing_graph, testing_step, A, B))
