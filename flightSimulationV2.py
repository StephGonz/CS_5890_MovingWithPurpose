## Stephanie Gonzales ##
###### A00979467 ######

import json
import time
import sys, pygame, math, time
import matplotlib.pyplot as plt

# Multiple that acts as a 'zoom' so that we can actually see everything
SCREEN_SCALE = 10
# Frames per Second
FPS = 100
# Background Color: WHITE
WHITE = [255,255,255]
# SIZE of the board (1m representation)
SIZE = width, height = 100*SCREEN_SCALE, 100*SCREEN_SCALE
# Render for the game display
SCREEN_RENDER = pygame.display.set_mode(SIZE)
# Length of the robot
ROBO_LENGTH = 3
# Turn radius for the wheels
ALPHA = 32

class HolonomicSim:
    def __init__(self, originX, originY, originTheta, destX, destY, destTheta, maxVelocity):
        self.originX = originX
        self.originY = originY
        self.originTheta = originTheta
        self.currentX = originX
        self.currentY = originY
        self.currentTheta = originTheta
        self.destX = destX
        self.destY = destY
        self.destTheta = destTheta
        self.maxVelocity = maxVelocity
        self.xPoints = []
        self.yPoints = []

        def getRise(self, originY, destY):
            return destY - originY
        
        def getRun(self, originX, destX):
            return destX - originX
        
        def getC(self, run, rise):
            return math.sqrt((run*run) + (rise*rise))
        
        def getUnit(self, run, rise, c):
            newRun = (run/2)*(run/2)
            newRise = (rise/2)*(rise/2)
            newValue = newRun + newRise
            newC = (c/2)*(c/2)
            return newC / newValue
        
        def metersPerFrame(maxVelocity):
            return maxVelocity / FPS
        
        self.rise = getRise(self, originY, destY)
        self.run = getRun(self, originX, destX)
        self.c = getC(self, self.run, self.rise)
        self.unit = getUnit(self, self.run, self.rise, self.c)
        self.arrived = False
        self.meterDiff = metersPerFrame(self.maxVelocity)

    def riseByRun(self):
        return self.rise*self.meterDiff/self.run

    def newPointFormula(self, betaRise):
        addToX = self.meterDiff*self.unit
        addToY = betaRise*self.unit
        if abs(self.destX - self.currentX) < abs(addToX):
            self.currentX = self.destX
        else:
            self.currentX = self.currentX + addToX
        if abs(self.destY - self.currentY) < abs(addToY):
            self.currentY = self.destY
        else:
            self.currentY = self.currentY + addToY

    def getDirection(self):
        yDiff = self.destY - self.originY
        xDiff = self.destX - self.originX
        thetaToX = math.atan2(yDiff, xDiff)
        thetaToY = 90 - math.degrees(thetaToX) 
        return thetaToY

    def newThetaFormula(self):
        if (self.currentX == self.destX and self.currentY == self.destY):
            self.currentTheta = self.destTheta
        else:
            self.currentTheta = self.getDirection()

    def hasArrived(self):
        if (self.currentX != self.destX):
            return False
        elif (self.currentY != self.destY):
            return False
        elif (self.currentTheta != destTheta):
            return False
        else:
            return True

    def updateFlight(self):
        self.xPoints.append(self.currentX)
        self.yPoints.append(self.currentY)
        print('HoloRobo - X: {0} Y: {1} Theta: {2}'.format(self.currentX, self.currentY, self.currentTheta))
        betaRise = self.riseByRun()
        self.newPointFormula(betaRise)
        self.newThetaFormula()
        self.arrived = self.hasArrived()
        if (self.arrived == True):
                self.xPoints.append(self.destX)
                self.yPoints.append(self.destY)
                print('HoloRobo - X: {0} Y: {1} Theta: {2}'.format(self.currentX, self.currentY, self.currentTheta))

class NonHolonomicSim:
    def __init__(self, originX, originY, theta, destX, destY, destTheta, maxVelocity, maxSteeringAngle):
        self.originX = originX
        self.originY = originY
        self.theta = theta
        self.destX = destX
        self.destY = destY
        self.destTheta = destTheta
        self.currentX = originX
        self.currentY = originY
        self.currentTheta = theta
        self.maxVelocity = maxVelocity
        self.maxSteeringAngle = maxSteeringAngle
        self.xPoints = []
        self.yPoints = []
        self.arrived = False
        self.goalTheta = 0
        self.front = (0,0)
        self.back = (0,0)
        self.now = time.time()
        self.arrived = False
        self.currentTurnRadius = self.getTurnRadius()
        self.addToX = 0
        self.addToY = 0

        def getRise(self, originY, destY):
            return destY - originY
        
        def getRun(self, originX, destX):
            return destX - originX
        
        def getC(self, run, rise):
            return math.sqrt((run*run) + (rise*rise))
        
        def getUnit(self, run, rise, c):
            newRun = (run/2)*(run/2)
            newRise = (rise/2)*(rise/2)
            newValue = newRun + newRise
            newC = (c/2)*(c/2)
            return newC / newValue
        
        def metersPerFrame(maxVelocity):
            return maxVelocity / FPS
        
        self.rise = getRise(self, originY, destY)
        self.run = getRun(self, originX, destX)
        self.c = getC(self, self.run, self.rise)
        self.unit = getUnit(self, self.run, self.rise, self.c)
        self.arrived = False
        self.meterDiff = metersPerFrame(self.maxVelocity)

    def riseByRun(self):
        return self.rise*self.meterDiff/self.run

    def flyToNewPoint(self, betaRise):
        self.addToX = self.meterDiff*self.unit
        self.addToY = betaRise*self.unit
        if abs(self.destX - self.currentX) < abs(self.addToX):
            self.currentX = self.destX
        else:
            self.currentX = self.currentX + self.addToX
        if abs(self.destY - self.currentY) < abs(self.addToY):
            self.currentY = self.destY
        else:
            self.currentY = self.currentY + self.addToY

    def updateFrontBack(self):
        m = math.tan(self.currentTheta)
        b = self.currentY - (m*self.currentX)
        frontX = self.currentX + float(ROBO_LENGTH/2)
        backX = self.currentX - float(ROBO_LENGTH/2)
        frontY = m*frontX + b
        backY = m*backX + b
        print()
        self.front = (frontX, frontY)
        self.back = (backX, backY)

    def crossProduct(self):
        self.updateFrontBack()
        pointA = ((self.currentX - self.front[0]), (self.currentY - self.front[1]))
        pointB = ((self.destX - self.currentX), (self.destY - self.currentY))
        return pointA[0]*pointB[1] - pointB[0]* pointA[1]

    def hasArrived(self):
        if (int(self.currentX) != int(self.destX)):
            return False
        elif (int(self.currentY) != int(self.destY)):
            return False
        elif (int(self.currentTheta) != int(destTheta)):
            return False
        else:
            return True

    def getTurnRadius(self):
        return ROBO_LENGTH / 2 * math.sin(self.maxSteeringAngle)

    def getDirection(self):
        yDiff = self.destY - self.currentY
        xDiff = self.destX - self.currentX
        thetaToX = math.atan2(yDiff, xDiff)
        thetaToY = 90 - math.degrees(thetaToX)
        if thetaToY < 0:
            thetaToY = thetaToY + 360
        return thetaToY

    def pickSteering(self):
        cross = self.crossProduct()
        if cross < 0:
            return -1*self.maxSteeringAngle
        else:
            return self.maxSteeringAngle

    def goalIsStraight(self):
        self.goalTheta = self.getDirection()
        if int(self.goalTheta) == int(self.currentTheta):
            return True
        else:
            return False

    def turnRobo(self):
        alpha = self.pickSteering()
        now = time.time()
        deltaTime = (now - self.now)
        newTheta = self.currentTheta + ((self.maxVelocity/ROBO_LENGTH)*math.tan(alpha)*deltaTime)
        newX = self.currentX - (math.radians(math.sin(self.currentTheta))*deltaTime*self.maxVelocity)
        newY = self.currentY + (math.radians(math.cos(self.currentTheta))*deltaTime*self.maxVelocity)
        if abs(self.destX - self.currentX) < abs(newX - self.currentX):
            newX = self.destX
        if abs(self.destY - self.currentY) < abs(newY - self.currentY):
            newY = self.destY
        if abs(self.goalTheta - self.currentTheta) < abs(newTheta - self.currentTheta):
            newTheta = self.goalTheta
        self.currenX = newX
        self.currentY = newY
        self.currentTheta = newTheta

    def updateTheta(self, addToX):
        thetaDiff = abs(self.destTheta - self.currentTheta)
        timesToUpdate = thetaDiff / self.maxSteeringAngle
        xDiff = abs(self.destX - self.currentX)
        unitsLeft = xDiff / addToX
        if timesToUpdate != 0:
            if unitsLeft <= timesToUpdate:
                thetaUnit = thetaDiff / timesToUpdate
                self.currentTheta = self.currentTheta + thetaUnit
        else:
            self.currentTheta = self.destTheta

    def flyToGoal(self):
        betaRise = self.riseByRun()
        self.flyToNewPoint(betaRise)
        self.updateTheta(self.addToX)
        self.arrived = self.hasArrived()
        if (self.arrived == True):
                self.xPoints.append(self.destX)
                self.yPoints.append(self.destY)

    def updateFlight(self):
        self.xPoints.append(self.currentX)
        self.yPoints.append(self.currentY)
        print('NonHoloRobo - X: {0} Y: {1} Theta: {2}'.format(self.currentX, self.currentY, self.currentTheta))   
        if self.goalIsStraight():
            self.goalTheta = destTheta
            self.flyToGoal()
        elif int(self.currentX) == int(self.destX) and int(self.currentY) == int(self.destY):
            self.arrived = True
        else:
            self.turnRobo()

def nonHoloRoboImg(x, y, theta):
    roboImg = pygame.image.load('nonHoloRobo2.jpg')
    pygame.transform.scale(roboImg, (1*SCREEN_SCALE, 1*SCREEN_SCALE))
    newRobo = pygame.transform.rotate(roboImg, theta)
    SCREEN_RENDER.blit(newRobo, (x, y))  

def holoRoboImg(x, y, theta):
    roboImg = pygame.image.load('holoRobo2.jpg')
    pygame.transform.scale(roboImg, (ROBO_LENGTH*SCREEN_SCALE, ROBO_LENGTH*SCREEN_SCALE))
    newRobo = pygame.transform.rotate(roboImg, theta)
    SCREEN_RENDER.blit(newRobo, (x, y))

#############################
#####  Get Config Info  #####
#############################

configFile = sys.argv[1]

f = open(configFile)

data = json.load(f) 
  
startList = data['start']
distList = data['goal']
maxVelocity = data['maxVelocity']
maxSteeringAngle = data['maxSteeringAngle']

f.close() 

originX = startList[0]
originY = startList[1]
originTheta = startList[2]

destX = distList[0]
destY = distList[1]
destTheta = distList[2]

#############################
########  Game Loop  ########
#############################

pygame.init()

holoRobo = HolonomicSim(originX, originY, originTheta, destX, destY, destTheta, maxVelocity)
nonHoloRobo = NonHolonomicSim(originX, originY, originTheta, destX, destY, destTheta, maxVelocity, maxSteeringAngle)

# First display with a wait so we can see original positions
SCREEN_RENDER.fill(WHITE)
holoRoboImg(int(holoRobo.currentX*SCREEN_SCALE), int(holoRobo.currentY*SCREEN_SCALE), holoRobo.currentTheta)
nonHoloRoboImg(int(nonHoloRobo.currentX*SCREEN_SCALE), int(nonHoloRobo.currentY*SCREEN_SCALE), nonHoloRobo.currentTheta)
pygame.display.set_caption('Flight Simulation')
pygame.display.update()
time.sleep(1)

while (holoRobo.arrived == False or nonHoloRobo.arrived == False):
    holoRobo.updateFlight()
    nonHoloRobo.updateFlight()
    #time.sleep(1)
    SCREEN_RENDER.fill(WHITE)
    holoRoboImg(int(holoRobo.currentX*SCREEN_SCALE), int(holoRobo.currentY*SCREEN_SCALE), holoRobo.currentTheta)
    nonHoloRoboImg(int(nonHoloRobo.currentX*SCREEN_SCALE), int(nonHoloRobo.currentY*SCREEN_SCALE), nonHoloRobo.currentTheta)
    pygame.display.update()

time.sleep(1)

pygame.quit()

plt.xlim(0, 100)
plt.ylim(0, 100)
plt.xlabel('X Values', fontsize=18)
plt.ylabel('Y Values', fontsize=16)
plt.plot(holoRobo.xPoints, holoRobo.yPoints, "blue")
plt.plot(nonHoloRobo.xPoints, nonHoloRobo.yPoints, "r-")
plt.show()
