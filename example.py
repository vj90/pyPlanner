import build.PathPlanner as planner

print(planner.add(1, 2))

planner.callRRT()

start = planner.RobotConfig(2, 3)
 
end = planner.RobotConfig(44, 55)


result = planner.planPath(start, end, 100, 100)
#print(result.path)

#print(result.metadata.data)

path_sample_list = [start,end]

obstacle = planner.CircularObstacle(2,2,5)

print(obstacle.getCenter())

print(obstacle.collision(path_sample_list))

