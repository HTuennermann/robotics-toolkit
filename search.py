grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']

delta_name = {(-1, 0): '^', (0, -1): '<', (1, 0): 'v', (0, 1): '>'}
import Queue


def expand(grid, init, goal, cost):
    expand = [[(0, 0) for row in range(len(grid[0]))] for col in range(len(grid))]
    visited = set()

    pq = Queue.PriorityQueue()
    pq.put((0, init[0], init[1], 0, 0))

    while not (pq.empty()):
        index, currenty, currentx, dy, dx = pq.get()

        if (currenty, currentx) not in visited:
            visited.add((currenty, currentx))
            expand[currenty][currentx] = (dy, dx) #add to expand list

            if (currenty, currentx) == goal:
                return expand

            for dy, dx in delta: # look around
                y, x = currenty + dy, currentx + dx

                if (x >= 0 and x < len(grid[0])) and (y >= 0 and y < len(grid)):
                    if ((y, x) not in visited) and (grid[y][x] != 1):
                        pq.put((index + cost, y, x, dy, dx))
    return expand


def search(grid, init, goal, cost):
    e = expand(grid, init, goal, cost)
    way = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    print e

    if e[goal[0]][goal[1]] == (0, 0):
        return way

    y, x = goal
    way[y][x] = '*'

    while e[y][x] != (0, 0):
        n = delta_name[e[y][x]]

        yn = y - e[y][x][0]
        xn = x - e[y][x][1]
        x, y = xn, yn
        way[y][x] = n

    print e[goal[0]][goal[1]]
    return way

print search(grid, init, goal, cost)