import build.PathPlanner as planner

print(planner.add(1, 2))

planner.callRRT()

start = planner.RobotConfig(2, 3)
 
end = planner.RobotConfig(44, 55)


result = planner.callRRT2(start, end, 100, 100)

print(result.path)

print(result.metadata)