from soar.io import io
import graph
import lib601.idealReadings as idealReadings
import lib601.markov as markov
import lib601.dist as dist
#import lib601.sonarDist as sonarDist
import lib601.util as util
import os,os.path
import math

####################################################################
###
### Preliminaries -- do not change the following code
###
####################################################################

labPath = os.getcwd()
WORLD_FILE = os.path.join(labPath,'oneDWorld.py')
FORWARD_VELOCITY = 0.1

# Robot's Ideal Readings
ideal = None

# Where the robot will be in the world
(xMin, xMax) = (0.5, 7.5)
robotY = y = 1.0

# Number of discrete locations 
numStates = 200 

# Number of discrete observations
sonarMax = 0.8 #maximum 'good' sonar reading
numObservations = 30

#method to discretize values into boxes of size gridSize
def discretize(value, gridSize, maxBin=float('inf')):
    return min(int(value/gridSize), maxBin)

#method to clip x to be within lo and hi limits, inclusive
def clip(x, lo, hi):
    return max(lo, min(x, hi))

#desired distance from right wall
desiredRight = 0.5
Kp,Ka = (10.0,2.0)

####################################################################
###
###          Probabilistic Models -- you may change this code
###
####################################################################

import lib601.dist as dist
# lib601.dist provides methods for manipulating discrete probability distributions
# see the documentation for details

#observation model
def obsModel(s):
    width = 0.2 #half width of triangle 
    r = numObservations*width/sonarMax
    return dist.mixture(dist.squareDist(0, numObservations), dist.triangleDist(ideal[s], int(r), 0, numObservations-1), 1e-3)

#transition model
def transModel(s):
    r = (FORWARD_VELOCITY / 10.0) * numStates / (xMax - xMin)
    return dist.mixture(dist.deltaDist(clip(s+int(r), 0, numStates-1)), dist.deltaDist(clip(s+int(r)+1, 0, numStates-1)), int(r)+1-r)

def confidentLocation(belief):

    center = belief.maxProbElt()
    devs = belief.expectation(lambda x: abs(x - center))
    #print center, devs
    return (center, devs < 6)

"""
    stDevCutoff = 10
    #will be sure when standard deviation is stDevCutoff states, so ~95% chance of being in radius of stDevCutoff states
    #this is because every state is 7/200 = 3.5 cm, so this radius is 21 cm; handout says 15cm is good
    
    Ex = belief.expectation(lambda x: x)
    Exx = belief.expectation(lambda x: x*x)
    var = Exx - Ex*Ex


    print Ex, Exx, var
    return (belief.maxProbElt(), var < stDevCutoff ** 2)
"""

# flag for whether to update the belief based on observations
# turn off to take measurements
DO_ESTIMATION = True

######################################################################
###
###          Brain Methods -- do not change the following code
###
######################################################################

def getParkingSpot(ideal):
    avg = sum(ideal)/float(len(ideal))
    i = len(ideal)-1
    while i>0 and ideal[i]>avg:
        i -= 1
    j = i
    while j>0 and ideal[j]<avg:
        j -= 1
    i = j
    while i>0 and ideal[i]>avg:
        i -= 1
    return (i+1+j)/2

def setup():
    global ideal,confident,parkingSpot,targetTheta,targetX
    ideal = idealReadings.computeIdealReadings(WORLD_FILE, xMin, xMax, robotY, numStates, numObservations)
    parkingSpot = getParkingSpot(ideal)
    targetX = None
    targetTheta = math.pi/2
    confident = False
    if not (hasattr(robot,'g') and robot.g.winfo_exists()):
        robot.g = graph.Grapher(ideal)
        robot.nS = numStates
    if robot.nS != numStates:
        robot.g.destroy()
        robot.g = graph.Grapher(ideal)
        robot.nS = numStates
    robot.estimator = markov.StateEstimator(dist.uniformDist(range(numStates)), transModel, obsModel)
    robot.g.updateObsGraph([0 for s in xrange(numStates)])
    robot.g.updateBeliefGraph([robot.estimator.belief.prob(s) for s in xrange(numStates)])
    robot.distances = []

def brainStart():
    pass

def step():
    global confident, targetX, targetTheta
    inp = io.SensorInput()
    sonars = inp.sonars
    #print "x: ", inp.odometry.x, "theta: ", inp.odometry.theta

    # current discretized sonar reading
    left = discretize(sonars[0], sonarMax/numObservations, numObservations-1)
   
    if not confident:
        print "not confident ", inp.odometry.x
        # GRAPHICS
        if robot.g is not None:
            # update observation model graph
            robot.g.updateObsLabel(left)
            robot.g.updateObsGraph([obsModel(s).prob(left) for s in xrange(numStates)])

        if DO_ESTIMATION:
            # update belief state
            robot.estimator.update(left)

        (location, confident) = confidentLocation(robot.estimator.belief)

        # GRAPHICS
        if robot.g is not None:
            # update belief graph
            robot.g.updateBeliefGraph([robot.estimator.belief.prob(s) for s in xrange(numStates)])
        if confident:
            targetX = (parkingSpot-location)*(xMax-xMin)/float(numStates)+inp.odometry.x
            print 'I am at x =',location,': proceeding to parking space'
        
        # DL6 Angle Controller
        (distanceRight, theta) = getDistanceRightAndAngle(sonars)
        if not theta:
           theta = 0
           #print 'Angle too large!'
        e = desiredRight-distanceRight
        #print "distanceRight: ", distanceRight, "theta", theta
        robot.distances.append(distanceRight)
        if len(robot.distances) > 10:
            ROTATIONAL_VELOCITY = Kp*e - Ka*theta
            #print ROTATIONAL_VELOCITY
            io.setForward(FORWARD_VELOCITY)
            io.setRotational(ROTATIONAL_VELOCITY)
        else:
            io.setForward(0)
            io.setRotational(0)
    else:
        inp.odometry.theta = util.fixAnglePlusMinusPi(inp.odometry.theta)
        print inp.odometry.theta, inp.odometry.x
        if inp.odometry.x>targetX+.05 and abs(inp.odometry.theta)<math.pi/4:
            io.Action(fvel=-0.1,rvel=0).execute() #drive backwards if necessary
            print "backwards!"
        elif inp.odometry.x<targetX and abs(inp.odometry.theta)<math.pi/4:
            io.Action(fvel=0.1,rvel=0).execute()  #drive to desired x location
            print "driving to ", targetX
        elif inp.odometry.theta<targetTheta-.05:
            io.Action(fvel=0,rvel=0.3).execute()  #rotate
            print "rotating"
        elif inp.sonars[2]>.3:
            io.Action(fvel=0.1,rvel=0).execute()  #drive into spot
            print "driving into spot"
        else:
            io.Action(fvel=0,rvel=0).execute()  #congratulate yourself (or call insurance company)
            print "parked!"

def brainStop():
    pass

def shutdown():
    pass

def getDistanceRightAndAngle(s):
    angleSpaced = 65*math.pi/180#(40+20*2)*math.pi/180
    right = s[3]
    center = s[2]
    sonarMax = 0.8
    #print "right: ", right, " center: ", center
    # if right > center:
    #     return(center, 0)
    if right < sonarMax and center < sonarMax:
        radius = 0.152/2 #little robot radius
        #radius = 0.38/2 #big robot radius
        a = center+radius
        b = right+radius
        c = math.sqrt(a*a+b*b-(2*a*b*math.cos(angleSpaced)))
        phi = math.asin((a/c)*math.sin(angleSpaced))
        if phi > math.pi/2:
            d = b*math.sin(phi-math.pi/2)
            #d -= radius
            alpha = math.acos(d/b)
            theta = -alpha
        else:
            d = b*math.sin(phi)
            if d/a > 1:
                return(right,None)
            alpha = math.acos(d/a)
            #d -= radius
            theta = alpha-angleSpaced
        return(d,theta)
    elif right < sonarMax:
        return(right, None)
    elif center < sonarMax:
        return(center, None)
    else:
        return(sonarMax, None)
