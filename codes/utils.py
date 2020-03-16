"""
 *  MIT License
 *
 *  Copyright (c) 2019 Arpit Aggarwal Shantam Bajpai
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
"""


# header files
import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop


# class for AStar
class AStar(object):
    # init function
    def __init__(self, start, goal, clearance, radius, stepSize):
        self.start = start
        self.goal = goal
        self.numRows = 200
        self.numCols = 300
        self.clearance = clearance
        self.radius = radius
        self.distance = {}
        self.path = {}
        self.costToCome = {}
        self.costToGo = {}
        self.visited = {}
        self.step_size = stepSize
        
        for row in range(1, 2 * self.numRows + 1):
            for col in range(1, 2 * self.numCols + 1):
                for theta in range(0, 360, 30):            
                    self.visited[(row, col, theta)] = False
    
    # move is valid 
    def IsValid(self, currRow, currCol):
        return (currRow >= (1 + self.radius + self.clearance) and currRow <= (self.numRows - self.radius - self.clearance) and currCol >= (1 + self.radius + self.clearance) and currCol <= (self.numCols - self.radius - self.clearance))

    # checks for an obstacle
    def IsObstacle(self, row, col):
        # constants
        sum_of_c_and_r = self.clearance + self.radius
        sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r
        
        # check circle
        dist1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225)) - ((25 + sum_of_c_and_r) * (25 + sum_of_c_and_r))
        
        # check eclipse
        dist2 = ((((row - 100) * (row - 100)) / ((20 + sum_of_c_and_r) * (20 + sum_of_c_and_r))) + (((col - 150) * (col - 150)) / ((40 + sum_of_c_and_r) * (40 + sum_of_c_and_r)))) - 1
        
        # check triangles
        (x1, y1) = (120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r))
        (x2, y2) = (150 - sqrt_of_c_and_r, 50)
        (x3, y3) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist3 = 1
        if(first <= 0 and second <= 0 and third <= 0):
            dist3 = 0
            
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        (x3, y3) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.714))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist4 = 1
        if(first >= 0 and second >= 0 and third >= 0):
            dist4 = 0
        
        # check rhombus
        (x1, y1) = (10 - sqrt_of_c_and_r, 225)
        (x2, y2) = (25, 200 - sqrt_of_c_and_r)
        (x3, y3) = (40 + sqrt_of_c_and_r, 225)
        (x4, y4) = (25, 250 + sqrt_of_c_and_r)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist5 = 1
        dist6 = 1
        if(first >= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist5 = 0
            dist6 = 0
        
        # check square
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (120 - sqrt_of_c_and_r, 75)
        (x3, y3) = (150, 100 + sqrt_of_c_and_r)
        (x4, y4) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.714))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist7 = 1
        dist8 = 1
        if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
            dist7 = 0
            dist8 = 0
        
        # check rod
        first = ((col - 95) * (8.66 + sqrt_of_c_and_r)) - ((5 + sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        second = ((col - 95) * (37.5 + sqrt_of_c_and_r)) - ((-64.95 - sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        third = ((col - 30.05 + sqrt_of_c_and_r) * (8.65 + sqrt_of_c_and_r)) - ((5.45 + sqrt_of_c_and_r) * (row - 67.5))
        fourth = ((col - 35.5) * (-37.49 - sqrt_of_c_and_r)) - ((64.5 + sqrt_of_c_and_r) * (row - 76.15 - sqrt_of_c_and_r))
        dist9 = 1
        dist10 = 1
        if(first <= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist9 = 0
            dist10 = 0
        
        if(dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
            return True
        return False
    
    # action move one
    def ActionMoveOne(self, currRow, currCol, theta):
        newRow = currRow + (np.sin(theta * (np.pi / 180)) / 0.5)
        newCol = currCol + (np.cos(theta * (np.pi / 180)) / 0.5)
        newTheta = (theta + 0) % 360

        if(self.IsValid(newRow, newCol) and self.IsObstacle(newRow, newCol) == False and self.visited[(int(round(2.0 * newRow)), int(round(2.0 * newCol)), newTheta)] == False):
            return True
        return False

    # action move two
    def ActionMoveTwo(self, currRow, currCol, theta):
        newRow = currRow + (np.sin((theta + 30) * (np.pi / 180)) / 0.5)
        newCol = currCol + (np.cos((theta + 30) * (np.pi / 180)) / 0.5)
        newTheta = (theta + 30) % 360
 
        if(self.IsValid(newRow, newCol) and self.IsObstacle(newRow, newCol) == False and self.visited[(int(round(2.0 * newRow)), int(round(2.0 * newCol)), newTheta)] == False):
            return True
        return False

    # action move three
    def ActionMoveThree(self, currRow, currCol, theta):
        newRow = currRow + (np.sin((theta + 60) * (np.pi / 180)) / 0.5)
        newCol = currCol + (np.cos((theta + 60) * (np.pi / 180)) / 0.5)
        newTheta = (theta + 60) % 360

        if(self.IsValid(newRow, newCol) and self.IsObstacle(newRow, newCol) == False and self.visited[(int(round(2.0 * newRow)), int(round(2.0 * newCol)), newTheta)] == False):
            return True
        return False

    # action move four
    def ActionMoveFour(self, currRow, currCol, theta):
        newRow = currRow + (np.sin((theta - 30) * (np.pi / 180)) / 0.5)
        newCol = currCol + (np.cos((theta - 30) * (np.pi / 180)) / 0.5)
        newTheta = (theta - 30) % 360

        if(self.IsValid(newRow, newCol) and self.IsObstacle(newRow, newCol) == False and self.visited[(int(round(2.0 * newRow)), int(round(2.0 * newCol)), newTheta)] == False):
            return True
        return False

    # action move five
    def ActionMoveFive(self, currRow, currCol, theta):
        newRow = currRow + (np.sin((theta - 60) * (np.pi / 180)) / 0.5)
        newCol = currCol + (np.cos((theta - 60) * (np.pi / 180)) / 0.5)
        newTheta = (theta - 60) % 360

        if(self.IsValid(newRow, newCol) and self.IsObstacle(newRow, newCol) == False and self.visited[(int(round(2.0 * newRow)), int(round(2.0 * newCol)), newTheta)] == False):
            return True
        return False
    
    # generate video
    def generate_video(path):
        files = glob.glob(str(path) + "/*")
        files = np.sort(files)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter("simulation.avi", fourcc, 20.0, (300, 200))
        for file in files:
            image = cv2.imread(file)
            image = cv2.resize(image, (300, 200))
            out.write(image)
        out.release()

    # animate path
    def animate(self, explored_states, backtrack_states, path):
        startX = []
        startY = []
        endX = []
        endY = []
        explored_startX = []
        explored_startY = []
        explored_endX = []
        explored_endY = []
        fig, ax = plt.subplots()
        plt.grid()
        ax.set_aspect('equal')
        plt.xlim(0, 300)
        plt.ylim(0, 200)
        count = 0

        for index in range(1, len(explored_states)):
            explored_startX.append(explored_states[index-1][1])
            explored_startY.append(explored_states[index-1][0])
            explored_endX.append(explored_states[index][1])
            explored_endY.append(explored_states[index][0])    
            #if(count % 500 == 0):
            #    plt.quiver(np.array((explored_startX)), np.array((explored_startY)), np.array((explored_endX)), np.array((explored_endY)), units = 'xy', scale = 10, color = 'g', headwidth = 0.5, headlength = 0)
            #    plt.savefig("output1/sample" + str(count) + ".png")
            count = count + 1
    
        if(len(backtrack_states) > 0):
            for index in range(1, len(backtrack_states)):
                startX.append(backtrack_states[index-1][1])
                startY.append(backtrack_states[index-1][0])
                endX.append(backtrack_states[index][1])
                endY.append(backtrack_states[index][0])    
                #if(count % 1 == 0):
                #    plt.quiver(np.array((startX)), np.array((startY)), np.array((endX)), np.array((endY)), units = 'xy', scale = 10, color = 'r', headwidth = 0.5, headlength = 0)
                #    plt.savefig("output1/sample" + str(count) + ".png")
                count = count + 1

        plt.quiver(np.array((explored_startX)), np.array((explored_startY)), np.array((explored_endX)), np.array((explored_endY)), units = 'xy', scale = 10, color = 'g', headwidth = 0.5, headlength = 0)
        if(len(backtrack_states) > 0):
            plt.quiver(np.array((startX)), np.array((startY)), np.array((endX)), np.array((endY)), units = 'xy', scale = 10, color = 'r', headwidth = 0.5, headlength = 0)
        plt.show()
        plt.close()
        
    # euc heuristic
    def euc_heuristic(self, row, col):
        return np.sqrt(((self.goal[0] - row)**2) + ((self.goal[1] - col)**2))
    
    # a-star algo
    def search(self):
        # mark source node and create a queu
        explored_states = []
        queue = []
        heappush(queue, (0, self.start))
        self.distance[self.start] = 0
        self.costToCome[self.start] = 0
        self.costToGo[self.start] = 0
        backtrackNode = None
        flag = 0
        steps = 0
        
        # run a-star
        while(len(queue) > 0):
            # get current node
            _, current_node = heappop(queue)
            self.visited[(int(round(2.0 * current_node[0])), int(round(2.0 * current_node[1])), current_node[2])] = True
            explored_states.append(current_node)
            steps = steps + 1
            
            # if goal node then break
            # Using the distance formula
            if(np.square(np.abs(current_node[0] - self.goal[0])) + np.square(np.abs(current_node[1] - self.goal[1])) < 2.25):
                flag = 1
                backtrackNode = current_node
                break
               
            # break if steps greater than 10000000
            if(steps > 50000000):
                break

            # traverse the edges
            if(self.ActionMoveOne(current_node[0], current_node[1], current_node[2])):
                newRow = current_node[0] + (np.sin(current_node[2] * (np.pi / 180)) / 0.5)
                newCol = current_node[1] + (np.cos(current_node[2] * (np.pi / 180)) / 0.5)
                newTheta = (current_node[2] + 0) % 360
                new_cost_to_come = self.costToCome[current_node] + 1
                new_cost_to_go = self.euc_heuristic(newRow, newCol)
                new_distance = new_cost_to_come + new_cost_to_go

                if(self.distance.get((newRow, newCol, newTheta)) == None):
                    self.distance[(newRow, newCol, newTheta)] = float('inf')                    

                if(self.distance[(newRow, newCol, newTheta)] > new_distance):
                    self.distance[(newRow, newCol, newTheta)] = new_distance
                    self.costToCome[(newRow, newCol, newTheta)] = new_cost_to_come
                    self.costToGo[(newRow, newCol, newTheta)] = new_cost_to_go
                    self.path[(newRow, newCol, newTheta)] = current_node
                    heappush(queue, (new_distance, (newRow, newCol, newTheta)))
            
            if(self.ActionMoveTwo(current_node[0], current_node[1], current_node[2])):
                newRow = current_node[0] + (np.sin((current_node[2] + 30) * (np.pi / 180)) / 0.5)
                newCol = current_node[1] + (np.cos((current_node[2] + 30) * (np.pi / 180)) / 0.5)
                newTheta = (current_node[2] + 30) % 360
                new_cost_to_come = self.costToCome[current_node] + 1.3
                new_cost_to_go = self.euc_heuristic(newRow, newCol)
                new_distance = new_cost_to_come + new_cost_to_go
                
                if(self.distance.get((newRow, newCol, newTheta)) == None):
                    self.distance[(newRow, newCol, newTheta)] = float('inf')

                if(self.distance[(newRow, newCol, newTheta)] > new_distance):
                    self.distance[(newRow, newCol, newTheta)] = new_distance
                    self.costToCome[(newRow, newCol, newTheta)] = new_cost_to_come
                    self.costToGo[(newRow, newCol, newTheta)] = new_cost_to_go
                    self.path[(newRow, newCol, newTheta)] = current_node
                    heappush(queue, (new_distance, (newRow, newCol, newTheta)))
                    
            if(self.ActionMoveThree(current_node[0], current_node[1], current_node[2])):
                newRow = current_node[0] + (np.sin((current_node[2] + 60) * (np.pi / 180)) / 0.5)
                newCol = current_node[1] + (np.cos((current_node[2] + 60) * (np.pi / 180)) / 0.5)
                newTheta = (current_node[2] + 60) % 360
                new_cost_to_come = self.costToCome[current_node] + 1.6
                new_cost_to_go = self.euc_heuristic(newRow, newCol)
                new_distance = new_cost_to_come + new_cost_to_go
                
                if(self.distance.get((newRow, newCol, newTheta)) == None):
                    self.distance[(newRow, newCol, newTheta)] = float('inf')

                if(self.distance[(newRow, newCol, newTheta)] > new_distance):
                    self.distance[(newRow, newCol, newTheta)] = new_distance
                    self.costToCome[(newRow, newCol, newTheta)] = new_cost_to_come
                    self.costToGo[(newRow, newCol, newTheta)] = new_cost_to_go
                    self.path[(newRow, newCol, newTheta)] = current_node
                    heappush(queue, (new_distance, (newRow, newCol, newTheta)))
                    
            if(self.ActionMoveFour(current_node[0], current_node[1], current_node[2])):
                newRow = current_node[0] + (np.sin((current_node[2] - 30) * (np.pi / 180)) / 0.5)
                newCol = current_node[1] + (np.cos((current_node[2] - 30) * (np.pi / 180)) / 0.5)
                newTheta = (current_node[2] - 30) % 360
                new_cost_to_come = self.costToCome[current_node] + 1.3
                new_cost_to_go = self.euc_heuristic(newRow, newCol)
                new_distance = new_cost_to_come + new_cost_to_go
                
                if(self.distance.get((newRow, newCol, newTheta)) == None):
                    self.distance[(newRow, newCol, newTheta)] = float('inf')

                if(self.distance[(newRow, newCol, newTheta)] > new_distance):
                    self.distance[(newRow, newCol, newTheta)] = new_distance
                    self.costToCome[(newRow, newCol, newTheta)] = new_cost_to_come
                    self.costToGo[(newRow, newCol, newTheta)] = new_cost_to_go
                    self.path[(newRow, newCol, newTheta)] = current_node
                    heappush(queue, (new_distance, (newRow, newCol, newTheta)))
                    
            if(self.ActionMoveFive(current_node[0], current_node[1], current_node[2])):
                newRow = current_node[0] + (np.sin((current_node[2] - 60) * (np.pi / 180)) / 0.5)
                newCol = current_node[1] + (np.cos((current_node[2] - 60) * (np.pi / 180)) / 0.5)
                newTheta = (current_node[2] - 60) % 360
                new_cost_to_come = self.costToCome[current_node] + 1.6
                new_cost_to_go = self.euc_heuristic(newRow, newCol)
                new_distance = new_cost_to_come + new_cost_to_go
                
                if(self.distance.get((newRow, newCol, newTheta)) == None):
                    self.distance[(newRow, newCol, newTheta)] = float('inf')

                if(self.distance[(newRow, newCol, newTheta)] > new_distance):
                    self.distance[(newRow, newCol, newTheta)] = new_distance
                    self.costToCome[(newRow, newCol, newTheta)] = new_cost_to_come
                    self.costToGo[(newRow, newCol, newTheta)] = new_cost_to_go
                    self.path[(newRow, newCol, newTheta)] = current_node
                    heappush(queue, (new_distance, (newRow, newCol, newTheta)))
                    
        # return if no optimal path
        if(flag == 0):
            return (explored_states, [], float('inf'))
        
        # backtrack path
        backtrack_states = []
        node = backtrackNode
        while(node != self.start):
            backtrack_states.append(node)
            node = self.path[node]
        backtrack_states.append(self.start)
        backtrack_states = list(reversed(backtrack_states))      
        return (explored_states, backtrack_states, self.distance[backtrackNode])
