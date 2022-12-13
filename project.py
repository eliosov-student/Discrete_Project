'''
Contains an implementation of the A* algorithm and a few helper functions
'''
import math
testing_graph = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]]
testing_step = int(math.sqrt(25))
A = [0, 2]
B = [3, 0]

def read_from_csv(filename):
    with open(filename , 'r') as file:
        text = ([i for i in file])
        step = int(text[0].split(' ')[1].replace('\n' , ''))
        x = text[1].split(' ')
        newx = x[1].replace('\n' , '')
        x[1] = newx
        y = text[2].split(' ')
        newy = y[1].replace('\n' , '')
        y[1] = newy
        graph = text[3:]
        for i in range(len(graph)):
            graph[i] = graph[i].replace('\n' , '')
            graph[i] = graph[i].split(' ')
            graph[i] = list(map(float , graph[i]))
        x = list(map(int , x))
        y = list(map(int , y))
        return graph , step , x , y 

def heuristic(grid, goal, vertex):
    '''
    A heuristic function using absolute distance difference
    '''
    return abs(vertex[0] - goal[0]) + abs(vertex[1] - goal[1]) \
        + abs(grid[vertex[0]][vertex[1]] - grid[goal[0]][goal[1]])


def get_neighbors(grid, vertex):
    '''
    A function that gets all neighbors of a vertex
    '''
    neighbors = []
    if vertex[0] > 0:
        neighbors.append(
            [(vertex[0]-1, vertex[1]),
                grid[vertex[0]-1][vertex[1]]])
    if vertex[1] > 0:
        neighbors.append(
            [(vertex[0], vertex[1]-1),
                grid[vertex[0]][vertex[1]-1]])
    if vertex[0] < len(grid)-1:
        neighbors.append(
            [(vertex[0]+1, vertex[1]),
                grid[vertex[0]+1][vertex[1]]])
    if vertex[1] < len(grid[0])-1:
        neighbors.append(
            [(vertex[0], vertex[1]+1),
                grid[vertex[0]][vertex[1]+1]])
    return neighbors


def a_star_algorithm(grid, step, start, goal):
    '''
    Uses the A* algorithm
    '''
    # open_list is a list of vertices which have been visited, but who's neighbors
    # haven't all been inspected, starts with the start vertex
    # closed_list is a list of vertices which have been visited
    # and who's neighbors have been inspected

    start = tuple(start)
    goal = tuple(goal)
    open_list = set([start])
    closed_list = set([])

    # distance_to_start contains currently known minimum distances from start to other vertices
    distance_to_start = {}

    distance_to_start[start] = 0

    # parents is an (incomplete) vector of vertices
    parents = {}
    parents[start] = start

    while len(open_list) > 0:
        chosen_candidate = None

        # find a vertex with the lowest value of heuristic() - evaluation function
        for candidate in open_list:
            if not chosen_candidate or distance_to_start[candidate] \
                + heuristic(grid, goal, candidate) < distance_to_start[chosen_candidate] +\
                    heuristic(grid, goal, chosen_candidate):
                chosen_candidate = candidate

        if chosen_candidate == goal:
            reconst_path = []
            while parents[chosen_candidate] != chosen_candidate:
                reconst_path.append(chosen_candidate)
                chosen_candidate = parents[chosen_candidate]
            reconst_path.append(start)
            reconst_path.reverse()
            return reconst_path

        # a list of [point, height]
        neighbors = get_neighbors(grid, chosen_candidate)
        for (neighbor, height) in neighbors:
            # if a neighbor isn't in open_list and closed_list
            # add it to open_list and note chosen_candidate as its parent
            if neighbor not in open_list and neighbor not in closed_list:
                open_list.add(neighbor)
                parents[neighbor] = chosen_candidate
                distance_to_start[neighbor] = distance_to_start[chosen_candidate] + \
                    math.sqrt(
                        step**2+(height-grid[chosen_candidate[0]][chosen_candidate[1]])**2)

            # otherwise, check if it's quicker to first visit chosen_candidate, then neighbor
            # and if it is, update parents and distance_to_start
            # and if the node was in the closed_]list, move it to open_list
            else:
                if distance_to_start[neighbor] > distance_to_start[chosen_candidate] \
                        + math.sqrt(step**2 +
                                    (height-grid[chosen_candidate[0]][chosen_candidate[1]])**2):
                    distance_to_start[neighbor] = distance_to_start[chosen_candidate] + \
                        math.sqrt(
                            step**2+(height-grid[chosen_candidate[0]][chosen_candidate[1]])**2)
                    parents[neighbor] = chosen_candidate

                    if neighbor in closed_list:
                        closed_list.remove(neighbor)
                        open_list.add(neighbor)

        # remove chosen_candidate from the open_list, and add it to closed_list
        # because all of its neighbors were inspected
        open_list.remove(chosen_candidate)
        closed_list.add(chosen_candidate)
    print('Path does not exist!')
    return None


if __name__ == '__main__':
    print(a_star_algorithm(testing_graph, testing_step, A, B))
