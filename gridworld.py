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


plt.show()