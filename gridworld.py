import matplotlib.pyplot as plt
import numpy as np

# Params 
GRID_HEIGHT = 100
GRID_WIDTH = 100
MAX_OBJECT_SIZE = 20
MIN_OBJECT_SIZE = 5
NUM_OBJECTS = 10

# create objects
# Randomly sample NUM_OBJECTS circuar objects with random radius between MAX_OBJECT_SIZE and MIN_OBJECT_SIZE and center
objects = []
for i in range(NUM_OBJECTS):
    object_size = np.random.randint(MIN_OBJECT_SIZE, MAX_OBJECT_SIZE)
    object_center = (np.random.randint(GRID_HEIGHT), np.random.randint(GRID_WIDTH))
    objects.append((object_center, object_size))

# Randomly sample start and goal states
start_state = (np.random.randint(GRID_HEIGHT), np.random.randint(GRID_WIDTH))
goal_state = (np.random.randint(GRID_HEIGHT), np.random.randint(GRID_WIDTH))

# display grid with objects
plt.figure()
plt.xlim(0, GRID_WIDTH)
plt.ylim(0, GRID_HEIGHT)
for object in objects:
    object_center, object_size = object
    plt.gca().add_patch(plt.Circle(object_center, object_size, color='r', fill=True))
plt.gca().add_patch(plt.Circle(start_state, 1, color='g', fill=True))
plt.gca().add_patch(plt.Circle(goal_state, 1, color='b', fill=True))

''' temp debug'''
#All nodes
nodes_x = [0,8,69,50,82,15,6,78,75,0,38,36,22,50,29,17,48,16,95,9,93,92,23,22,81,48,82,50,29,7,82,57,2,47,44,74,85,89,94,11,38,98,96,77,53,36,9,65,71,60,25,5,5,70,10,78,45,20,77,71,34,18,52,6,52,44,4,87,48,13,95,58,68,33,28,32,63,57,37,44,33,18,41,51,19,99,57,54,88,75,96,30,9,31,6,40,67,80,68,50,97,]
nodes_y = [0,65,17,92,74,15,47,71,58,98,8,79,92,51,29,99,25,17,18,77,76,99,23,50,74,71,85,4,53,58,24,30,26,97,8,37,19,60,63,27,11,20,0,25,84,35,45,11,64,68,86,10,46,99,33,48,76,93,97,82,7,95,35,76,18,77,1,62,9,58,92,40,78,97,56,62,50,68,15,89,40,89,57,89,65,14,57,77,39,16,7,11,39,98,75,91,58,9,32,87,2,]

#Path
path_x = [92,93,82,50,8,0,]
path_y = [99,76,74,92,65,0,]
plt.scatter(nodes_x,nodes_y)
plt.plot(path_x,path_y)


'''temp debug'''
plt.show()