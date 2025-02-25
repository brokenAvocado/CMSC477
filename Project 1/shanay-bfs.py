import csv
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time

startTime = time.time()

def importCSV(filename):
    map = []
    with open(filename, newline='') as file:
        reader = csv.reader(file)
        for row in reader:
            map.append([int(cell) for cell in row])
    return map

def findStartAndEnd(map):
    for row in range(len(map)):
        for col in range(len(map[0])):
            if map[row][col] == 2:
                start = (row, col)
            if map[row][col] == 3:
                end = (row, col)
    return start, end

def bfsAlgorithm(map, start, end):
    movements = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    queue = [start]
    visited = {start}
    parent = {start: None}
    numOperations = 0

    row, col = start
    while (row, col) != end:
        row, col = queue.pop(0)

        for movementRow, movementCol in movements:
            newRow, newCol = row + movementRow, col + movementCol
            if (map[newRow][newCol] != 1 and (newRow, newCol) not in visited): 
                queue.append((newRow, newCol))
                visited.add((newRow, newCol))
                parent[(newRow, newCol)] = (row, col)
                numOperations +=1

    # backtrack to get the shortest path
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = parent.get(current)
    path.reverse()  # reverse to start from the robot's start rather than the goal
    
    return path, numOperations, visited

def animate(map, start, end, visited):
    plt.imshow(np.array(map), cmap="gray", origin="upper")
    plt.show()
    return None


map = importCSV("Map1.csv")
start, end = findStartAndEnd(map)
# print(start)
# print(end)
path, numOperations, visited = bfsAlgorithm(map, start, end)


print("Started at: " + str(start))
print("Ended at: " + str(end))
print("Took " + str(numOperations) + " operations")
print("Shortest Path:" + str(path))
print(len(path))
print("Elapsed time: " + str(time.time()-startTime))