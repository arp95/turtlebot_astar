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
import matplotlib.lines as mlines
from heapq import heappush, heappop


# class for AStar
class AStar(object):
    # init function
    def __init__(self, start, goal, wheelRPM, clearance):
        self.start = start
        self.goal = goal
        self.xLength = 300
        self.yLength = 200
        self.innerXLength = 300
        self.innerYLength = 200
        self.wheelRPM = wheelRPM
        self.clearance = clearance + 1
        self.radius = 1
        self.wheelDistance = 1
        self.wheelRadius = 1
        self.distance = {}
        self.path = {}
        self.costToCome = {}
        self.costToGo = {}
        self.visited = {}
        
        for x in range(1, 2 * self.innerXLength + 1):
            for y in range(1, 2 * self.innerYLength + 1):
                for theta in range(0, 360, 1):            
                    self.visited[(x, y, theta)] = False
    
    # move is valid 
    def IsValid(self, currX, currY):
        return (currX >= (self.radius + self.clearance) and currX <= (self.innerXLength - self.radius - self.clearance) and currY >= (self.radius + self.clearance) and currY <= (self.innerYLength - self.radius - self.clearance))

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
    
    # return updated position by taking into account non-holonomic constraint of robot
    def GetNewPositionOfRobot(self, currentNode, leftRPM, rightRPM):
        leftWheelVelocity = (6.0 * leftRPM) * self.wheelRadius
        rightWheelVelocity = (6.0 * rightRPM) * self.wheelRadius
        dx = (self.wheelRadius / 2.0) * (leftWheelVelocity + rightWheelVelocity) * np.cos(currentNode[2] * (np.pi / 180.0))
        dy = (self.wheelRadius / 2.0) * (leftWheelVelocity + rightWheelVelocity) * np.sin(currentNode[2] * (np.pi / 180.0))
        dtheta = (self.wheelRadius / self.wheelDistance) * (rightWheelVelocity - leftWheelVelocity)
        return (currentNode[0] + (dx / 0.5), currentNode[1] + (dy / 0.5), (currentNode[2] + dtheta) % 360)

    # action move one
    def ActionMoveRobot(self, currentNode, leftRPM, rightRPM):
        # update position
        (newX, newY, newTheta) = self.GetNewPositionOfRobot(currentNode, leftRPM, rightRPM)

        # check obstacle
        if(self.IsValid(newX, newY) and self.IsObstacle(newX, newY) == False and self.visited[(int(round(2.0 * newX)), int(round(2.0 * newY)), int(round(newTheta)))] == False):
            return (True, newX, newY, newTheta)
        return (False, newX, newY, newTheta)

    # update action
    def UpdateAction(self, currentNode, weight, newX, newY, newTheta, action):
        newCostToCome = self.costToCome[currentNode] + weight
        newCostToGo = self.euc_heuristic(newX, newY)
        newDistance = newCostToCome + newCostToGo

        if(self.distance.get((newX, newY, newTheta)) == None):
            self.distance[(newX, newY, newTheta)] = float('inf')                    
        if(self.distance[(newX, newY, newTheta)] > newDistance):
            self.distance[(newX, newY, newTheta)] = newDistance
            self.costToCome[(newX, newY, newTheta)] = newCostToCome
            self.costToGo[(newX, newY, newTheta)] = newCostToGo
            self.path[(newX, newY, newTheta)] = (currentNode, action)
            return True
        return False

    # animate path
    def animate(self, exploredStates, backtrackStates, path):
        startX = []
        startY = []
        endX = []
        endY = []
        explored_startX = []
        explored_startY = []
        explored_endX = []
        explored_endY = []
        fig, ax = plt.subplots()
        plt.xlabel("x-coordinate")
        plt.ylabel("y-coordinate")
        plt.grid()
        ax.set_aspect('equal')
        plt.xlim(0, 300)
        plt.ylim(0, 200)
        count = 0
        
        # obstacle space
        obstacleX = []
        obstacleY = []
        size = []
        for index1 in range(0, 300):
            for index2 in range(0, 200):
                if(self.IsObstacle(index2/0.5, index1/0.5)):
                    obstacleX.append(index1/0.5)
                    obstacleY.append(index2/0.5)     
                    size.append(15)      
        obstacleX = np.array(obstacleX)
        obstacleY = np.array(obstacleY)
        plt.scatter(obstacleX, obstacleY, color='b', s=size)

        # explore node space
        for index in range(1, len(exploredStates)):
            parentNode = self.path[exploredStates[index]]
            explored_startX.append(parentNode[1])
            explored_startY.append(parentNode[0])
            explored_endX.append(exploredStates[index][1] - parentNode[1])
            explored_endY.append(exploredStates[index][0] - parentNode[0])    
            #if(count % 500 == 0 or (index > 11000 and index % 100 == 0) or index > 12600):
            #    plt.quiver(np.array((explored_startX)), np.array((explored_startY)), np.array((explored_endX)), np.array((explored_endY)), units = 'xy', scale = 1, color = 'g', label='Explored region')
            #    plt.savefig("output1/sample" + str(count) + ".png", dpi=1700)
            count = count + 1
    
        # backtrack space
        if(len(backtrackStates) > 0):
            for index in range(1, len(backtrackStates)):
                startX.append(backtrackStates[index-1][1])
                startY.append(backtrackStates[index-1][0])
                endX.append(backtrackStates[index][1] - backtrackStates[index-1][1])
                endY.append(backtrackStates[index][0] - backtrackStates[index-1][0])    
                #if(count % 2 == 0):
                #    plt.quiver(np.array((startX)), np.array((startY)), np.array((endX)), np.array((endY)), units = 'xy', scale = 1, color = 'r', label='Backtrack path')
                #    plt.savefig("output1/sample" + str(count) + ".png", dpi=1700)
                count = count + 1

        plt.quiver(np.array((explored_startX)), np.array((explored_startY)), np.array((explored_endX)), np.array((explored_endY)), units = 'xy', scale = 1, color = 'g', label='Explored region')
        if(len(backtrackStates) > 0):
            plt.quiver(np.array((startX)), np.array((startY)), np.array((endX)), np.array((endY)), units = 'xy', scale = 1, color = 'r', label='Backtrack path')
        plt.savefig("sample.png", dpi=1700)
        plt.legend()
        plt.show()
        plt.close()
        
    # eucledian heuristic (becomes weighted a-star when weight made greater than 1.0)
    def euc_heuristic(self, currX, currY, weight = 1.0):
        return weight * (np.sqrt(((self.goal[0] - currX) ** 2) + ((self.goal[1] - currY) ** 2)) / 0.5)
    
    # a-star algo
    def search(self):
        # mark source node and create a queue
        exploredStates = []
        queue = []
        self.costToCome[self.start] = 0
        self.costToGo[self.start] = self.euc_heuristic(self.start[0], self.start[1])
        self.distance[self.start] = self.costToCome[self.start] + self.costToGo[self.start]
        heappush(queue, (self.distance[self.start], self.costToCome[self.start], self.start))
        backtrackNode = None
        flag = 0
        steps = 0
        
        # run a-star
        while(len(queue) > 0):
            # get current node
            _, _, currentNode = heappop(queue)
            self.visited[(int(round(2.0 * currentNode[0])), int(round(2.0 * currentNode[1])), int(round(currentNode[2])))] = True
            exploredStates.append(currentNode)
            steps = steps + 1
            
            # if goal node then break, using the distance formula
            if(np.square(np.abs(currentNode[0] - self.goal[0])) + np.square(np.abs(currentNode[1] - self.goal[1])) < 0.5625):
                backtrackNode = currentNode
                flag = 1
                break
               
            # break if steps greater than 5000000
            if(steps > 5000000):
                break

            # traverse the edges
            # action 1
            (moveOnePossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, 0, self.wheelRPM[0])
            if(moveOnePossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[0], newX, newY, newTheta, (0, self.wheelRPM[0]))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
            
            # action 2
            (moveTwoPossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, self.wheelRPM[0], 0)
            if(moveTwoPossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[0], newX, newY, newTheta, (self.wheelRPM[0], 0))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
                    
            # action 3
            (moveThreePossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, self.wheelRPM[0], self.wheelRPM[0])
            if(moveThreePossible):
                updateHeap = self.UpdateAction(currentNode, (self.wheelRPM[0] * 1.4142), newX, newY, newTheta, (self.wheelRPM[0], self.wheelRPM[0]))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
              
            # action 4
            (moveFourPossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, 0, self.wheelRPM[1])      
            if(moveFourPossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[1], newX, newY, newTheta, (0, self.wheelRPM[1]))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
                    
            # action 5
            (moveFivePossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, self.wheelRPM[1], 0)
            if(moveFivePossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[1], newX, newY, newTheta, (self.wheelRPM[1], 0))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))

            # action 6
            (moveSixPossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, self.wheelRPM[1], self.wheelRPM[1])
            if(moveSixPossible):
                updateHeap = self.UpdateAction(currentNode, (self.wheelRPM[1] * 1.4142), newX, newY, newTheta, (self.wheelRPM[1], self.wheelRPM[1]))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
                    
            # action 7
            (moveSevenPossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, self.wheelRPM[0], self.wheelRPM[1])
            if(moveSevenPossible):
                updateHeap = self.UpdateAction(currentNode, max((self.wheelRPM[0] * 1.4142), (self.wheelRPM[1] * 1.4142)), newX, newY, newTheta, (self.wheelRPM[0], self.wheelRPM[1]))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))

            
            # action 8
            (moveEightPossible, newX, newY, newTheta) = self.ActionMoveRobot(currentNode, self.wheelRPM[1], self.wheelRPM[0])
            if(moveEightPossible):
                updateHeap = self.UpdateAction(currentNode, max((self.wheelRPM[0] * 1.4142), (self.wheelRPM[1] * 1.4142)), newX, newY, newTheta, (self.wheelRPM[1], self.wheelRPM[0]))
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))

        # return if no optimal path
        if(flag == 0):
            return (exploredStates, [], float('inf'))
        
        # backtrack path
        backtrackStates = []
        actions = []
        node = backtrackNode
        action = None
        while(node != self.start):
            backtrackStates.append(node)
            if(action != None):
                actions.append(action)
            node, action = self.path[node]
        backtrackStates.append(self.start)
        actions.append(action)
        backtrackStates = list(reversed(backtrackStates))  
        actions = list(reversed(actions))    
        return (exploredStates, backtrackStates, actions, self.distance[backtrackNode])