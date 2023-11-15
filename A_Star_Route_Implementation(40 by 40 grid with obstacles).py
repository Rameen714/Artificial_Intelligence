from matplotlib import pyplot
from matplotlib import colors
import numpy as np
import math


# for random generation of map 
# data = [
#     # [random.randint(a=0,b=1) for x in range(0,8) ],
#     # [random.randint(a=0,b=1) for x in range(0,8) ]
# ]

def generate_grid_with_obst(grid_size,num_obstacles):
    grid = np.zeros(grid_size, dtype=int)
    start = (0,0)
    end = ( grid_size[0] - 1, grid_size[1] - 1) # 39,39
    grid[start] = 1
    grid[end] = 1

    # for random generation of obstacles 

    # for _ in range(num_obstacles):
    #     while True:
    #         ob_x = np.random.randint(grid_size[0])
    #         ob_y = np.random.randint(grid_size[1])

    #         if (ob_x, ob_y) not in [start, end]:
    #             grid[ob_x, ob_y] = -1
    #             break


    #static random 50 obstacles
    obstacle_coordinates = [
        # (3,7), (2,5), (1,4), (3,3), (6,2)
        (3, 10), (5, 15), (7, 7), (8, 22), (11, 30),
        (12, 9), (15, 17), (17, 31), (20, 2), (22, 5),
        (25, 28), (26, 13), (29, 24), (32, 20), (34, 14),
        (37, 18), (4, 33), (6, 27), (9, 35), (13, 25),
        (16, 12), (18, 34), (21, 3), (24, 6), (27, 29),
        (30, 8), (33, 19), (36, 16), (2, 38), (14, 36),
        (23, 23), (10, 26), (19, 21), (28, 11), (31, 7),
        (35, 4), (39, 1), (1, 39), (38, 37), (37, 0),
        (15, 2), (6, 39), (2, 0), (18, 0), (23, 32),
        (17, 7), (30, 38), (24, 35), (39, 11), (9, 19)
    ]
    for coord in obstacle_coordinates:
        x, y = coord
        grid[x, y] = -1  # Obstacle

    

    return grid


class Node:
    def __init__(self, pos, parent, cost, h):
        self.pos = pos
        self.parent = parent
        self.cost = cost
        self.h = h

    def __lt__(self, other):
        return (self.cost + self.h) < (other.cost + other.h)

     
def A_search(grid, start, end):
    open_list = []
    closed_list = []

    start_node = Node(start, None,0,cal_heuristic(start,end))
    open_list.append(start_node)
    # print("This is node",start_node.pos)

    while open_list:
        current_node = min(open_list, key=lambda node: node.cost + node.h )
        # print("This is node",current_node.pos)
        open_list.remove(current_node)
        closed_list.append(current_node)

        if current_node.pos == end:
            return reconstruct_path(grid,current_node)
        
        for neighbor in get_neighbors(current_node.pos,grid):
            if neighbor in closed_list:
                continue
            cost = current_node.cost + 1
            if neighbor not in open_list or cost < neighbor.cost:
                # if not already explored or reached from a longer path earlier
                neighbor_node = Node(neighbor,current_node,cost, cal_heuristic(neighbor,end))
                if neighbor not in open_list:
                    open_list.append(neighbor_node)
    
    return None


def reconstruct_path(grid,node):
    cost = 0
    path = []
    while node:
        path.insert(0,node.pos)
        x,y = node.pos
        grid[x,y] = 2
        cost = 1 + cost
        # print("This is cost",cost)
        node = node.parent
    return cost


def get_neighbors(position, grid):
    x, y = position
    neighbors = []

    # Define possible movement directions (up, down, left, right, and diagonals)
    directions = [
        (0, 1), (0, -1), (1, 0), (-1, 0),
        (1, 1), (-1, -1), (1, -1), (-1, 1)
    ]

    for dx, dy in directions:
        new_x, new_y = x + dx, y + dy

        # Check if the new position is within the grid bounds
        if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]):
            # Check if the new position is not an obstacle (you can customize this condition)
            if grid[new_x][new_y] != -1:
                neighbors.append((new_x, new_y))

    return neighbors


def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def cal_heuristic(current, goal):
    return euclidean_distance(current, goal)



# --------------------------------------------------------------------------------------------------------

#initilaizing 
grid_size = (40,40)
num_obstacles = 50
final_grid = generate_grid_with_obst(grid_size, num_obstacles)

#showing simulation 
colormap = colors.ListedColormap(["red", "grey", "green", "blue"])
fig, ax = pyplot.subplots()


start = (0,0)
end = ( grid_size[0] - 1, grid_size[1] - 1)

cost = A_search(final_grid,start,end)

final_grid[start] = 1
final_grid[end] = 1

# fig for additional things such as below
fig.suptitle('Grid Visualization with Obstacles')
# ax for axes - coordinate system where u can creaet 2D grid visualization
cax = ax.matshow(final_grid, cmap=colormap)
#cax for setting colour replated properties to grid


# adding black boundries to cells
for i in range(grid_size[0]):
    for j in range(grid_size[1]):
        ax.add_patch(pyplot.Rectangle((j - 0.5, i - 0.5), 1, 1, fill=False, edgecolor='black'))
# j and i are indics indicating a particular cell
# -0.5 centers a rectangle within a cell
# 1,1 recatngle height width which baiscally means a cell
# and then we are making edges black of the particular rectangle (cell)
# in loop all cells ony by one treated as rectangles and then boundary set to black.

pyplot.text(40,45,f'Cost: {cost: }',fontsize=12, color='black')

pyplot.show()
