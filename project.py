'''
Contains an implementation of Dijkstra's algorithm and a few helper functions
'''
import math
import heapq


def read_from_csv(filename):
    '''
    Reads grid, step, start, goal from a csv file
    '''
    with open(filename, 'r', encoding='utf-8') as file:
        text = list(file)
        step = int(text[0].split(' ')[1].replace('\n', ''))
        start = text[1].split(' ')
        start[1] = start[1].replace('\n', '')
        goal = text[2].split(' ')
        goal[1] = goal[1].replace('\n', '')
        graph = text[3:]
        for i, row in enumerate(graph):
            row = row.replace('\n', '').split(' ')
            graph[i] = list(map(float, row))
        start = list(map(int, start))
        goal = list(map(int, goal))
        return graph, step, start, goal


def get_neighbors(grid, vertex):
    '''
    A function that gets all neighbors of a vertex as [(x,y), height]
    '''
    length = len(grid)-1
    neighbors = []
    if vertex[0] > 0:
        neighbors.append(
            [(vertex[0]-1, vertex[1]),
                grid[vertex[0]-1][vertex[1]]])
    if vertex[1] > 0:
        neighbors.append(
            [(vertex[0], vertex[1]-1),
                grid[vertex[0]][vertex[1]-1]])
    if vertex[0] < length:
        neighbors.append(
            [(vertex[0]+1, vertex[1]),
                grid[vertex[0]+1][vertex[1]]])
    if vertex[1] < length:
        neighbors.append(
            [(vertex[0], vertex[1]+1),
                grid[vertex[0]][vertex[1]+1]])
    return neighbors


def dijkstra_algorithm_optimized(grid, step, start, goal):
    '''
    Uses Dijkstra's algorithm with a shortcut
    '''
    start = tuple(start)
    goal = tuple(goal)
    # queue is a priority queue of ALL known (distance to start, vertex) ordered by the distance
    # only contains vertices with non-final distances to start
    queue = [(0, start)]
    # distance_to_start is a dictionary of the currently known shortest distances to start
    distance_to_start = {start: 0}
    # parents is an (incomplete) vector of vertices
    parents = {}
    parents[start] = start
    while queue:
        # gets the vertex with the smallest distance to start and removes it from queue
        (current_distance, current_vertex) = heapq.heappop(queue)
        # removes (distance to start, vertex) where the distance is not the shortest known
        # this is done because heappop(queue) is considerably more efficient than
        # calling queue.remove((old_distance, current_vertex)) and heapify(queue)
        if current_distance != distance_to_start[current_vertex]:
            continue
        # shortcut
        if current_vertex == goal:
            break

        # find the shortest distances to start for the vertices adjacent to current_vertex
        neighbors = get_neighbors(grid, current_vertex)
        for neighbor, height in neighbors:
            distance = current_distance + \
                math.sqrt(
                    step**2 + (height
                               - grid[current_vertex[0]][current_vertex[1]])**2)
            if neighbor not in distance_to_start or \
                    distance < distance_to_start[neighbor]:
                # adds the newly found shortest distance to start to queue and distance_to_start
                heapq.heappush(queue, (distance, neighbor))
                distance_to_start[neighbor] = distance
                parents[neighbor] = current_vertex

    # reconstructs the path from goal to start
    reconst_path = []
    while parents[goal] != goal:
        reconst_path.append(goal)
        goal = parents[goal]
    reconst_path.append(start)
    reconst_path.reverse()
    return reconst_path


if __name__ == '__main__':
    grid1, step1, x1, y1 = read_from_csv('testing.csv')
    print(dijkstra_algorithm_optimized(grid1, step1, x1, y1))
