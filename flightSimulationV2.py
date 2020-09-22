## Stephanie Gonzales ##
###### A00979467 ######

import json
import sys, pygame, math
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

class FlightSim:
    def __init__(self, originX, originY, destX, destY):
        self.originX = originX
        self.originY = originY
        self.destX = destX
        self.destY = destY

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

        self.rise = getRise(self, originY, destY)
        self.run = getRun(self, originX, destX)
        self.c = getC(self, self.run, self.rise)
        self.unit = getUnit(self, self.run, self.rise, self.c)

class NonHolonomic:
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

class Holonomic:
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

def riseByRun(run, alphaRun, alphaRise):
    return alphaRise*run/alphaRun

def newPointFormula(originX, originY, rise, run, unit):
    addToX = run*unit
    addToY = rise*unit
    pointX = originX + addToX
    pointY = originY + addToY
    return (pointX, pointY)

def metersPerFrame(maxVelocity):
    return maxVelocity / FPS

def robo(x, y, theta):
    roboImg = pygame.image.load('holoRobo2.jpg')
    pygame.transform.scale(roboImg, (1*SCREEN_SCALE, 1*SCREEN_SCALE))
    newRobo = pygame.transform.rotate(roboImg, 0)
    SCREEN_RENDER.blit(newRobo, (x, y))

def nonHoloRobo(x, y, theta):
    roboImg = pygame.image.load('nonHoloRobo2.jpg')
    pygame.transform.scale(roboImg, (1*SCREEN_SCALE, 1*SCREEN_SCALE))
    newRobo = pygame.transform.rotate(roboImg, theta)
    SCREEN_RENDER.blit(newRobo, (x, y))  

def holoRobo(x, y, theta):
    roboImg = pygame.image.load('holoRobo2.jpg')
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

holo = Holonomic(originX, originY, originTheta, destX, destY, destTheta)
holoExes = []
holoWhys = []

nonHolo = NonHolonomic(originX, originY, originTheta, destX, destY, destTheta)
nonHoloExes = []
nonHoloWhys = []

meterDiff = metersPerFrame(maxVelocity)

while (holo.arrived == False):
    holoExes.append(holo.currentX)
    holoWhys.append(holo.currentY)
    print('X: {0} Y: {1}'.format(holo.currentX, holo.currentY))
    SCREEN_RENDER.fill(WHITE)
    holoRobo(int(holo.currentX*SCREEN_SCALE), int(holo.currentY*SCREEN_SCALE), holo.currentTheta)
    pygame.display.set_caption('Flight Simulation')
    pygame.display.update()
    newHoloPointFormula(holo, meterDiff)

pygame.display.set_caption('Flight Simulation')
pygame.display.update()

pygame.quit()

plt.plot(holoExes, holoWhys)
plt.plot(nonHoloExes, nonHoloWhys)
plt.show()
