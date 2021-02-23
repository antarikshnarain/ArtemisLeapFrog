import os
import random

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self):
        return (str) (self.x) + ", " + (str) (self.y) + ", " + (str) (self.z)

start = Point(0, 0, 0)
os.system("echo start: " + (str)(start) +" > /dev/ttys001")
xmax = 10
ymax = 10
zmax = 10

goal = Point(random.randrange(xmax), random.randrange(ymax), random.randrange(zmax))
os.system("echo goal: " + (str)(goal) +" > /dev/ttys001")

current = start
while (current.x != goal.x or current.y != goal.y or current.z != goal.z):
    if (goal.x < current.x):
        current.x -= 1
    elif (goal.x > current.x):
        current.x += 1
    if (goal.y < current.y):
        current.y -= 1
    elif (goal.y > current.y):
        current.y += 1
    if (goal.z < current.z):
        current.z -= 1
    elif (goal.z > current.z):
        current.z += 1
    os.system("echo current: " + (str)(current) +" > /dev/ttys001")
os.system("echo goal reached! > /dev/ttys001")
