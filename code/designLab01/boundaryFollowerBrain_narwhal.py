from lib601.plotWindow import PlotWindow
import lib601.soarWorld as soarWorld
from soar.io import io

# this function is called when the brain is loaded
def setup():
    robot.slimeX = []
    robot.slimeY = []

# this function is called when the start button is pushed
def brainStart():
    pass

# this function is called 10 times per second
def step():
    #io.sonarMonitor(True)
    s = io.getSonars()
    x,y,theta = io.getPosition()
    robot.slimeX.append(x)
    robot.slimeY.append(y)

    if s[2] < 0.3: #s[2] is the right sonar
        io.setForward(0.1)
        io.setRotational(0.4)
    elif s[1] == 0.3 and s[2] == 0.3: #s[2] is the center sonar
        io.setForward(0.05)
        io.setRotational(0)
    else:
        io.setForward(0.1)
        if s[1] > 0.4:
            io.setRotational(-0.4)
        elif s[1] < 0.3:
            io.setRotational(0.5)
        else:
            io.setRotational(0)
    print str(s[1]) + ',' + str(s[2])

#change this line if the world file is in a different location on your system
PATH_TO_WORLD = '/usr/local/lib/python2.7/dist-packages/soar/worlds/boundaryFollowerWorld.py'

# called when the stop button is pushed
def brainStop():
    p = PlotWindow('Slime Trail')
    soarWorld.plotSoarWorld(PATH_TO_WORLD,p) #show the soar world
    p.plot(robot.slimeX,robot.slimeY) #plot the recorded slime trail data

# called when brain or world is reloaded (before setup)
def shutdown():
    pass
