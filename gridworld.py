import matplotlib.pyplot as plt
import numpy as np
import build.PathPlanner as planner

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
    objects.append((*object_center, object_size))


# Randomly sample start and goal states
start_state = (np.random.randint(GRID_HEIGHT), np.random.randint(GRID_WIDTH))
goal_state = (np.random.randint(GRID_HEIGHT), np.random.randint(GRID_WIDTH))

# display grid with objects
plt.figure()
plt.xlim(0, GRID_WIDTH)
plt.ylim(0, GRID_HEIGHT)
for object in objects:
    object_x, object_y, object_size = object
    plt.gca().add_patch(plt.Circle((object_x,object_y), object_size, color='cyan', fill=True))
plt.gca().add_patch(plt.Circle(start_state, 1, color='g', fill=True))
plt.gca().add_patch(plt.Circle(goal_state, 1, color='r', fill=True))


# Call RRT
print("Calling RRT")
start = planner.RobotConfig(*start_state)
end = planner.RobotConfig(*goal_state)
result = planner.planPath(start, end, GRID_HEIGHT, GRID_WIDTH, objects)
print("RRT Done")
# Display all nodes, connected to their parents
metadata = result.metadata.data
for node_metadata in metadata:
    node, parent_idx = node_metadata
    print(node)
    if parent_idx != -1:
        parent = metadata[parent_idx][0]
        plt.plot([node.x, parent.x], [node.y, parent.y], 'b--')

# Display path
path = result.path
for i in range(len(path) - 1):
    plt.plot([path[i].x, path[i+1].x], [path[i].y, path[i+1].y], 'g-')
   
 
plt.show()