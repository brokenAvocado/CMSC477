import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import ast
import os
import numpy as np

class DJI:
    def __init__(self, givenFilepath):
        self.filepath = givenFilepath
        self.matrix = []

        # Finding the start and end
        self.startRow = 0
        self.startCol = 0
        self.createMatrix()

        # Plotting variables
        self.xs = []
        self.ys = []
        self.start = []
        self.end = []

        # Result and path variables
        self.resultx = []
        self.resulty = []

        self.pathx = []
        self.pathy = []

        self.interpx = np.array([])
        self.interpy = np.array([])

    def createMatrix(self):
        currDir = os.getcwd()
        path = os.path.join(currDir, self.filepath)
        with open(path, mode="r") as file:
            for lines in csv.reader(file):
                self.matrix.insert(0,lines)

    def giveStart(self):
        for row in self.matrix:
            for element in row:
                if element == '2':
                    self.startRow = self.matrix.index(row)
                    self.startCol = row.index(element)
                    return (self.startRow, self.startCol)

    def giveEnd(self):
        for row in self.matrix:
            for element in row:
                if element == '3':
                    self.startRow = self.matrix.index(row)
                    self.startCol = row.index(element)
                    return (self.startRow, self.startCol)
                
    def dist(self, x1, x0):
        return float(((x1[0]-x0[0])**2+(x1[1]-x0[1])**2)**0.5)
    
    def initPlot(self):
        self.xs = []
        self.ys = []
        self.start = []
        self.end = []
        
        for row in range(0, len(self.matrix[0])):
            for col in range(0, len(self.matrix)):
                if self.matrix[col][row] == '2':
                    self.start.append(row)
                    self.start.append(col)
                elif self.matrix[col][row] == '3':
                    self.end.append(row)
                    self.end.append(col)
                elif self.matrix[col][row] != '0':
                    self.xs.append(row)
                    self.ys.append(col)
    
    def interp(self, elems = 50):
        self.interpy = np.linspace(self.giveStart[0], self.giveEnd[0], elems)
        self.interpx = np.interp(self.interpy, self.pathy, self.pathx)

    def plot(self, pathBool=False):
        fig, ax = plt.subplots()

        self.graph = ax.plot(self.xs,self.ys,'ko')
        self.graph = ax.plot(self.start[0],self.start[1],'go')
        self.graph = ax.plot(self.end[0],self.end[1],'ro')
        self.graph = ax.plot([],[], 'bx', markersize = 3)[0]

        ani = animation.FuncAnimation(fig=fig, func=self.animate, frames=500, interval=0)
        if pathBool:
            ax.plot(self.pathx, self.pathy, 'ro', markersize=3)
        # ani.save("dfs_map1.gif", writer="pillow", fps = 30)

    def search(self):
        unvisited = {}
        visited = {}
        map = {}
        path = []

        for row in range(len(self.matrix)):
            for ind, element in enumerate(self.matrix[row]):
                if element == '0' or element == '3':
                    location = (row, ind)
                    unvisited[str(location)] = float('inf')
        
        unvisited[str(self.giveStart())] = 0
        start = time.time() 

        while unvisited:
            unvisited = dict(sorted(unvisited.items(), key=lambda item: item[1]))

            look = ast.literal_eval(next(iter(unvisited)))
            shortest = unvisited[str(look)]

            if self.matrix[look[0]][look[1]] != '1':
                adjacent = [(look[0]-1, look[1]), (look[0]+1, look[1]), (look[0], look[1]-1), (look[0], look[1]+1), (look[0]-1, look[1]-1), (look[0]+1, look[1]+1), (look[0]+1, look[1]-1), (look[0]-1, look[1]+1)]
                for adj in adjacent:
                    if (str(adj) in unvisited) and adj[0] >= 0 and adj[1] >= 0:
                        distance = self.dist(adj, look)+unvisited[str(look)]
                        if distance < unvisited[str(adj)]:
                            unvisited[str(adj)] = distance
                        
                        if not(str(adj) in map):
                            map[str(adj)] = str(look)
                
                visited[str(look)] = unvisited[str(look)]
                del unvisited[str(look)]

            if self.matrix[look[0]][look[1]] == '3':
                print("Found goal: " + str(look))
                print("Steps taken: " + str(len(visited)))
                print("Shortest Path: " + str(shortest))
                print("Time taken: " + str(time.time()-start))
                break
        
        curr = str(self.giveEnd())
        while (curr != None):
            try:
                path.append(curr)
                curr = map[curr]
            except KeyError: 
                break

        for point in visited:
            point = point[1:-1].split(", ")
            self.resultx.append(int(point[1]))
            self.resulty.append(int(point[0]))

        for nodes in path:
            nodes = nodes[1:-1].split(", ")
            self.pathx.append(int(nodes[1]))
            self.pathy.append(int(nodes[0]))

    def animate(self, frame):
        self.graph.set_xdata(self.resultx[:frame*20])
        self.graph.set_ydata(self.resulty[:frame*20])
        