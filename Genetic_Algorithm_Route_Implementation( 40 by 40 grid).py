from matplotlib import pyplot
from matplotlib import colors
import numpy as np
import math
import random

# for random generation of map 
# data = [
#     # [random.randint(a=0,b=1) for x in range(0,8) ],
#     # [random.randint(a=0,b=1) for x in range(0,8) ]
# ]

cost= 0

def generate_grid_with_obst(grid_size,num_obstacles):
    grid = np.zeros(grid_size, dtype=int)
    start = (0,0)
    end = ( grid_size[0] - 1, grid_size[1] - 1) # 39,39
    grid[start] = 1
    grid[end] = 1

    # grid[20, 20] = -1

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
    # for coord in obstacle_coordinates:
    #     x, y = coord
    #     grid[x, y] = -1  # Obstacle

    # grid[20,20] = -1

    return grid


class Node:
    def __init__(self, pos, parent, cost, h):
        self.pos = pos
        self.parent = parent
        self.cost = cost
        self.h = h

    def __lt__(self, other):
        return (self.cost + self.h) < (other.cost + other.h)

     
def A_search(grid, start, end,signal):
    open_list = []
    closed_list = []


    start_node = Node(start, None,0,cal_heuristic(start,end,grid))
    open_list.append(start_node)
    # print("This is node",start_node.pos)

    while open_list:
        current_node = min(open_list, key=lambda node: node.cost + node.h )
        # print("This is node",current_node.pos)
        open_list.remove(current_node)
        closed_list.append(current_node)

        if current_node.pos == end:
            return reconstruct_path(grid,current_node,signal)
        
        for neighbor in get_neighbors(current_node.pos,grid):
            if neighbor in closed_list:
                continue
            cost = current_node.cost + 1
            if (neighbor!=-1) & (neighbor not in open_list or cost < neighbor.cost):
                # if not already explored or reached from a longer path earlier
                neighbor_node = Node(neighbor,current_node,cost, cal_heuristic(neighbor,end,grid))
                if neighbor not in open_list:
                    open_list.append(neighbor_node)
    
    return None


def reconstruct_path(grid,node,signal):
    # cost = 0
    path = []
    while node:
        path.insert(0,node.pos)
        if(signal == 1):
            x,y = node.pos
            grid[x,y] = 2
        # cost = 1 + cost
        # print("This is cost",cost)
        node = node.parent
    #return cost
    return path


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

# function to calculate euclidean distance btw two points/nodes/cells
def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# eculcidean distncae from start to end
def cal_heuristic(current, goal,grid):
    x_current, y_current = current
    x_goal, y_goal = goal
    
    # Euclidean distance between current position and goal
    euclidean_dist = euclidean_distance(current, goal)
    
    # Check if the current position is an obstacle
    if grid[x_current][y_current] == -1:
        # Return a very high heuristic value if the current position is an obstacle
        return float('inf')
    
    # Calculate the number of obstacles between the current position and the goal
    obstacle_count = 0
    for i in range(min(x_current, x_goal), max(x_current, x_goal) + 1):
        for j in range(min(y_current, y_goal), max(y_current, y_goal) + 1):
            if grid[i][j] == -1:
                obstacle_count += 1
    
    # Apply a penalty for the number of obstacles in the path
    penalty = obstacle_count * 10  # Adjust the penalty factor as needed
    
    # Return the sum of the Euclidean distance and the penalty
    return euclidean_dist + penalty
    # return euclidean_distance(current, goal)

# takes a choromosome, join all waypoints through A* and returns total cost
def cal_fitness(chromosome, grid,signal):
    cost = 0
    path = []
    j = 0
    chromosome = sorted(chromosome)
    for i in range(0, len(chromosome) - 1, 1):
        p = A_search(grid, chromosome[i], chromosome[i + 1],signal) # connecting waypoints through A star
        path.append(p)  
        cost += len(path[j])  # total cost through all points
        j += 1

    if None in path:
        return float('inf')  # Return a high cost for an invalid path

    # Calculate the number of obstacles encountered
    # obstacle_count = 0
    # for subpath in path:
    #     for point in subpath:
    #         x, y = point
    #         if grid[x, y] == -1:
    #             obstacle_count += 1
            # else:
            #     grid[x, y] = 2

    # Return the fitness value, combining cost and obstacle count
    return cost #+ obstacle_count


def roulette_wheel_selection(population, fitness_values, num_parents):

    selected_parents = []

    # Calculate total fitness
    total_fitness = sum(fitness_values)

    # generating parents accoridng to input provided as "num_parents"
    for _ in range(num_parents):

        # Generating a random value between 0 and the total fitness
        random_value = random.uniform(0, total_fitness)

        cumulative_fitness = 0
        parent = None

        # Select a parent whose cumulative fitness exceeds the random value
        for i, chromosome in enumerate(population):
            cumulative_fitness += fitness_values[i]
            if cumulative_fitness >= random_value:
                parent = chromosome
                break

        selected_parents.append(parent)

    return selected_parents

def one_point_crossover(parent1, parent2):
    # Select a random crossover point
    crossover_point = random.randint(1, len(parent1) - 1)

    # Create offspring by combining the genetic material of parents
    offspring1 = parent1[:crossover_point] + parent2[crossover_point:]
    offspring2 = parent2[:crossover_point] + parent1[crossover_point:]

    return offspring1, offspring2

def two_point_crossover(parent1, parent2):
    # Two random crossover points
    crossover_point1 = random.randint(0, len(parent1) - 1)
    crossover_point2 = random.randint(0, len(parent1) - 1)

    # Crossover_point1 is before crossover_point2
    if crossover_point1 > crossover_point2:
        crossover_point1, crossover_point2 = crossover_point2, crossover_point1

    # Two empty offspring
    offspring1 = [-1] * len(parent1)
    offspring2 = [-1] * len(parent2)

    # Copy the genetic material from parents to offspring
    offspring1[crossover_point1:crossover_point2 + 1] = parent1[crossover_point1:crossover_point2 + 1]
    offspring2[crossover_point1:crossover_point2 + 1] = parent2[crossover_point1:crossover_point2 + 1]

    # Remaining genes from the other parent
    remaining_genes1 = [gene for gene in parent2 if gene not in offspring1]
    remaining_genes2 = [gene for gene in parent1 if gene not in offspring2]

    for i in range(len(parent1)):
        if offspring1[i] == -1:
            offspring1[i] = remaining_genes1.pop(0)
        if offspring2[i] == -1:
            offspring2[i] = remaining_genes2.pop(0)

    return offspring1, offspring2


def is_goal_met(fitness_pairs, goal_val, fitness_tolerance=5):
    
    fittest_fitness = fitness_pairs[0]
    
    print("fittes:",fittest_fitness)
    return goal_val <= fittest_fitness <= (goal_val + fitness_tolerance)

def is_obstacle(point, grid):
    x, y = point
    return grid[x][y] == -1

def mutate(chromosome, grid, mutation_rate):
    mutated_chromosome = chromosome[:]  # Create a copy of the chromosome to avoid modifying the original

    for i in range(1, len(mutated_chromosome) - 1):
        if random.random() < mutation_rate:
            x, y = mutated_chromosome[i]
            if grid[x][y] == -1:
                # Regenerate the gene if it's an obstacle
                new_gene = (random.randint(0, 39), random.randint(0, 39))
                while grid[new_gene[0]][new_gene[1]] == -1:
                    new_gene = (random.randint(0, 39), random.randint(0, 39))
                mutated_chromosome[i] = new_gene

    return mutated_chromosome

def init_pop(population,fitness,start_point,end_point,chromosome_size,pop_size,grid):

    for i in range(pop_size):

        # Initialize a chromosome with random genes

        genes = []
        for _ in range(chromosome_size):
            gene = (random.randint(0, 39), random.randint(0, 39))
            while is_obstacle(gene, grid):
                # Regenerate the gene if it's an obstacle
                gene = (random.randint(0, 39), random.randint(0, 39))
            genes.append(gene)

        # Create the complete chromosome , add start and end points
        chromosome = [start_point] + genes + [end_point]
        population.append(chromosome)
        fitness.append(0)

    return population,fitness

def GA(grid,goal_val,cost):
    
    # ----------------------------STEP 1: Initilaize Population--------------------

    # Defined the size of the population and chromosome
    pop_size = 60
    num_parents = int(pop_size*0.3)
    chromosome_size = 8  # Excluding start and end points
    population = []
    fitness = []
    start_point = (0, 0)  # Start point
    end_point = (39, 39)  # End point

    population,fitness = init_pop(population,fitness,start_point,end_point,chromosome_size,pop_size,grid)
    print("pop initialized")
    print("This is initial population")
    for i in range(0,pop_size,1):
        print(population[i],'\n')


    max_iterations = 30
    iteration = 0
    goal_not_met = True

    while goal_not_met and iteration < max_iterations:

        print("Iteration:",iteration)

        # ----------------------------STEP 2: Calculate fitness for the population--------------------
        for i in range(0,pop_size,1):
            fitness[i] = cal_fitness(population[i],grid,0)
            #print('Fitness values calculated for \n',i,": ",fitness[i],'\n')

        # ----------------------------STEP 3: Select some members as parents according to fitness--------------------
        # quarter of population selected to produce offsprings
        parents = roulette_wheel_selection(population, fitness, num_parents)

        # ----------------------------STEP 4: Produce offspring---------------------------------------
        # diversity - exploration
        # elitism - exploitation
        # Out of parents ratio exploit 60% by crossover and explore 40% by mutation

        #currently u have proportionate ranking applied , TRY LINEAR RANKING 

        offspring = []
        exploit_parents = int(num_parents * 0.6)
        explore_parents = int(num_parents * 0.4)
        print(exploit_parents)
        print(explore_parents)
        print(len(population))
        print(len(fitness))

        for i in range(0,explore_parents,2):
            parent1 = parents[i]
            parent2 = parents[i + 1]
            child1 = []
            child2 = []
            child1 = mutate(parent1,final_grid, 0.4)
            child2 = mutate(parent2,final_grid, 0.4)
            offspring.append(child1)
            offspring.append(child2)
            fitness.append( cal_fitness(child1,grid,0))
            fitness.append( cal_fitness(child2,grid,0))
        population = population + offspring
        print(len(population))
        print(len(fitness))

        

        offspring = []
        for i in range(explore_parents, exploit_parents, 2):
            parent1 = parents[i]
            parent2 = parents[i + 1]
            child1 = []
            child2 = []
            if(i % 2 == 0):
                child1, child2 = one_point_crossover(parent1, parent2)
            else:
                child1, child2 = two_point_crossover(parent1, parent2)
            offspring.append(child1)
            offspring.append(child2)
            fitness.append( cal_fitness(child1,grid,0))
            fitness.append( cal_fitness(child2,grid,0))
        population = population + offspring 
        print(len(population))
        print(len(fitness))

        # ----------------------------STEP 5: Select fitest population( replace some memebrs with the new offspirngs )--------------------

        # Create a list of (individual, fitness) tuples for sorting
        individual_fitness_pairs = [(population[i], fitness[i]) for i in range(len(population))]

        # Sort the population based on fitness 
        sorted_population = sorted(individual_fitness_pairs, key=lambda x: x[1])

        # Select the top individuals ( 80% fittest  and 20% less fit )
        fit_mem_sel = int(pop_size*0.8)
        lessfit_mem_sel = int(pop_size*0.2)
        
        selected_individuals = [pair[0] for pair in sorted_population[:fit_mem_sel]]
        selected_individuals += [pair[0] for pair in sorted_population[-lessfit_mem_sel:]]
        # print("Inidviduals selected for next generation ")
        # for i in range(0,len(selected_individuals),1):
        #     print(i,": ",selected_individuals[i],'\n')
        
        selected_sorted_fitness = [pair[1] for pair in sorted_population[:pop_size]]
        print("This are fittnesses ")
        for i in range(0,len(selected_sorted_fitness),1):
            print(i,": ",selected_sorted_fitness[i],'\n')

        # ----------------------------STEP 6: Check if optimal obtained--------------------

        goal_not_met = not is_goal_met(selected_sorted_fitness, goal_val)

        iteration += 1

        cost = selected_sorted_fitness[0]
        solution = selected_individuals[0]

    cal_fitness(solution,grid,1)

    return cost,solution


# --------------------------------------------------------------------------------------------------------
cost = 0

#initilaizing 
grid_size = (40,40)
num_obstacles = 50
final_grid = generate_grid_with_obst(grid_size, num_obstacles)

#showing simulation 
colormap = colors.ListedColormap([ "white", "green", "blue"])
fig, ax = pyplot.subplots()


start = (0,0)
end = ( grid_size[0] - 1, grid_size[1] - 1)

path = A_search(final_grid,start,end,0)
goal_val = len(path)

cost, path2 = GA(final_grid,goal_val + 5,cost)

path2 = sorted(path2)

for i in range(10):
    final_grid[path2[i]] = 2
    print(path2[i])

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
