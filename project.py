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
    queue = [(0, start)]
    distance_to_start = {start: 0}
    parents = {}
    parents[start] = start

    while queue:
        (current_distance, current_vertex) = heapq.heappop(queue)

        if current_distance != distance_to_start[current_vertex]:
            continue

        if current_vertex == goal:
            break


        neighbors = get_neighbors(grid, current_vertex)
        for neighbor, height in neighbors:
            distance = current_distance + \
                math.sqrt(
                    step**2 + (height
                               - grid[current_vertex[0]][current_vertex[1]])**2)
            if neighbor not in distance_to_start or \
                    distance < distance_to_start[neighbor]:

                heapq.heappush(queue, (distance, neighbor))
                distance_to_start[neighbor] = distance
                parents[neighbor] = current_vertex

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
