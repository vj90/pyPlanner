import build.example as example

print(example.add(1, 2))

example.callRRT()

start = example.RobotConfig(2, 3)

end = example.RobotConfig(44, 55)


example.callRRT2(start, end, 100, 100)