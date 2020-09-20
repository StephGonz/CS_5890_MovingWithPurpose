## Stephanie Gonzales ##
###### A00979467 ######

import json
import sys, pygame, math

def newPointFormula(originX, originY, rise, run, unit):
    addToX = run*unit
    addToY = rise*unit
    pointX = originX + addToX
    pointY = originY + addToY
    return (pointX, pointY)

def getRise(originY, distY):
    return distY - originY

def getRun(originX, distX):
    return distX - originX

def riseByRun(run, alphaRun, alphaRise):
    return alphaRise*run/alphaRun

def getC(run, rise):
    return math.sqrt((run*run) + (rise*rise))

def getUnit(run, rise, c):
    newRun = (run/2)*(run/2)
    newRise = (rise/2)*(rise/2)
    newValue = newRun + newRise
    newC = (c/2)*(c/2)
    return newC / newValue

def metersPerFrame(maxVelocity, framesPerSec):
    return maxVelocity / framesPerSec

pygame.init()
size = width, height = 500, 500
screen = pygame.display.set_mode(size)
roboImg = pygame.image.load('robot.png')
pygame.transform.scale(roboImg, (20, 20))
white=[255,255,255]
FPS = 60 

def robo(x, y, theta):
    newRobo = pygame.transform.rotate(roboImg, theta)
    screen.blit(newRobo, (x, y))

configFile = sys.argv[1]

f = open(configFile) 

data = json.load(f) 
  
startList = data['start']
distList = data['goal']
maxVelocity = data['maxVelocity']

f.close() 

originX = startList[0]
originY = startList[1]
originTheta = startList[2]

distX = distList[0]
distY = distList[1]
distTheta = distList[2]

rise = getRise(originY, distY)
run = getRun(originX, distX)
c = getC(run, rise)
unit = getUnit(run, rise, c)
meterDiff = metersPerFrame(maxVelocity, FPS)

currentX = originX
currentY = originY
theta = 0
while currentX < distX and currentY < distY:
    theta += 0.5
    print('X: {0} Y: {1}'.format(currentX, currentY))
    betaRise = riseByRun(meterDiff, run, rise)
    currentX, currentY = newPointFormula(currentX, currentY, betaRise, meterDiff, unit)
    screen.fill(white)
    robo(int(currentX), int(currentY), theta)
    pygame.display.set_caption('Flight Simulation')
    pygame.display.update()
currentX = distX
currentY = distY
print('X: {0} Y: {1}'.format(currentX, currentY))
screen.fill(white)
robo(int(currentX), int(currentY), theta)

pygame.display.set_caption('Flight Simulation')
pygame.display.update()

pygame.quit()
