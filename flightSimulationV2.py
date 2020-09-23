## Stephanie Gonzales ##
###### A00979467 ######

import json
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
        xDiff = self.destX - self.currentX
        yDiff = self.destY - self.currentY
        if (xDiff < addToX):
            self.currentX = self.destX
        else:
            self.currentX = self.currentX + addToX
        if (yDiff < addToY):
            self.currentY = self.destY
        else:
            self.currentY = self.currentY + addToY

    def getDirection(self):
        yDiff = self.destY - self.originY
        xDiff = self.destX - self.originX
        theta = math.atan2(yDiff, xDiff)
        return math.degrees(theta)

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
        print('X: {0} Y: {1}'.format(self.currentX, self.currentY))
        betaRise = self.riseByRun()
        self.newPointFormula(betaRise)
        self.newThetaFormula()
        self.arrived = self.hasArrived()
        if (self.arrived == True):
                self.xPoints.append(self.destX)
                self.yPoints.append(self.destY)
                print('X: {0} Y: {1}'.format(self.currentX, self.currentY))

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
            return "bla"

        self.arrived = hasArrived(self)

class BadHolonomicSim:
    def __init__(self, originX, originY, theta, destX, destY, destTheta):
        self.originX = originX
        self.originY = originY
        self.theta = theta
        self.destX = destX
        self.destY = destY
        self.destTheta = destTheta
        self.currentX = originX
        self.currentY = originY
        self.currentTheta = theta

        def hasArrived(self):
            if (self.currentX != self.destX):
                return False
            elif (self.currentY != self.destY):
                return False
            elif (self.currentTheta != destTheta):
                return False
            else:
                return True
        self.arrived = hasArrived(self)


def metersPerFrame(maxVelocity):
    return maxVelocity / FPS

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

def badHoloRoboImg(x, y, theta):
    roboImg = pygame.image.load('badRobos.jpg')
    pygame.transform.scale(roboImg, (ROBO_LENGTH*SCREEN_SCALE, ROBO_LENGTH*SCREEN_SCALE))
    newRobo = pygame.transform.rotate(roboImg, theta)
    SCREEN_RENDER.blit(newRobo, (x, y))

def getThetaG(maxVelocity):
    return maxVelocity / ROBO_LENGTH * math.tan(ALPHA)

def getXG(maxVelocity, theta):
    return -1 * (maxVelocity / ROBO_LENGTH) * math.sin(theta)

def getYG(maxVelocity, theta):
    return maxVelocity * math.cos(theta)

def newNonHoloPointFormula(nonHoloObj, maxVelocity, meterDiff):
    xDiff = nonHoloObj.destX - nonHoloObj.currentX
    yDiff = nonHoloObj.destY - nonHoloObj.currentY
    if (xDiff < meterDiff):
        nonHoloObj.currentX = destX
    else:
        nonHoloObj.currentX = nonHoloObj.currentX + getXG(maxVelocity, nonHoloObj.currentTheta) * meterDiff

def getHoloOrientation(xDirection, yDirection, diff):
    if (xDirection and diff < 0):
        return 270
    elif (xDirection and diff > 0):
        return 90
    elif (yDirection and diff < 0):
        return 180
    else:
        return 0

def newHoloPointFormula(holoObj, meterDiff):
    xDiff = holoObj.destX - holoObj.currentX
    yDiff = holoObj.destY - holoObj.currentY
    moveX = False
    moveY = False
    diff = 0
    if (xDiff < meterDiff):
        holoObj.currentX = destX
    else:
        holoObj.currentX += meterDiff
        moveX = True
        diff = xDiff
    if (holoObj.currentX == destX):
        if(yDiff < meterDiff):
            holoObj.currentY = destY
        else:
            holoObj.currentY += meterDiff
            moveY = True
            diff = yDiff
    if (moveX == False and moveY == False):
        holoObj.currentTheta = destTheta
        holoObj.arrived = True
    else:
        holoObj.currentTheta = getHoloOrientation(moveX, moveY, diff)

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
#nonHoloRobo = 

# First display with a wait so we can see original positions
SCREEN_RENDER.fill(WHITE)
holoRoboImg(int(holoRobo.currentX*SCREEN_SCALE), int(holoRobo.currentY*SCREEN_SCALE), holoRobo.currentTheta)
#nonHoloRoboImg(int())
pygame.display.set_caption('Flight Simulation')
pygame.display.update()
time.sleep(1)

while (holoRobo.arrived == False):
    holoRobo.updateFlight()
    SCREEN_RENDER.fill(WHITE)
    holoRoboImg(int(holoRobo.currentX*SCREEN_SCALE), int(holoRobo.currentY*SCREEN_SCALE), holoRobo.currentTheta)
    pygame.display.update()

time.sleep(1)

pygame.quit()

plt.plot(holoRobo.xPoints, holoRobo.yPoints)
plt.xlabel('X Values', fontsize=18)
plt.ylabel('Y Values', fontsize=16)
#plt.plot(nonHoloExes, nonHoloWhys)
plt.show()
