import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import ast

class DJI():
    def __init__(self, givenFilepath):
        self.filepath = givenFilepath
        self.matrix = []

        self.startRow = 0
        self.startCol = 0
        self.createMatrix()

        self.resultx = []
        self.resulty = []

        self.pathx = []
        self.pathy = []

    def dist(self, x1, x0):
        return float(((x1[0]-x0[0])**2+(x1[1]-x0[1])**2)**0.5)

    def search(self):
        unvisited = {}
        visited = {}
        map = {}
        path = []

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
            self.resultx.append(int(point[0]))
            self.resulty.append(int(point[1]))

        for nodes in path:
            nodes = nodes[1:-1].split(", ")
            self.pathx.append(int(nodes[0]))
            self.pathy.append(int(nodes[1]))

        self.plot()

if __name__=="__main__":
    map1.search()