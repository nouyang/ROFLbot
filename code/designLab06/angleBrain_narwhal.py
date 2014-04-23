import lib601.sm as sm
from soar.io import io
#import lib601.sonarDist as sonarDist
#from lib601.sonarDist import *
import lib601.plotWindow as plotWindow
import math

desiredRight = 0.3
forwardVelocity = 0.1
time = 0

def setup():
    robot.distances = []
    robot.rvels = []

def brainStart():
    io.setForward(forwardVelocity)

def step():
    sonars = io.getSonars()
    (distanceRight, theta) = getDistanceRightAndAngle(sonars)
    print 'd_o =',distanceRight,' theta =',theta
    robot.distances.append(distanceRight)

    lastAngle = 0
    if not theta == None:
        lastAngle = theta

    Kps = [1, 3, 10, 30, 100, 300]
    Kas = [0.632355, 1.095344, 1.99899, 3.46400, 6.324455, 10.95435]

    i = 2
    Kp = Kps[i]
    Ka = Kas[i]

    #delay before starting to adjust values
    if len(robot.distances) > 10:
        io.setForward(forwardVelocity)
        rotationalVelocity = Kp * (desiredRight - distanceRight) - Ka * lastAngle
        #print rotationalVelocity
    else:
        io.setForward(0)
        rotationalVelocity = 0
        
    robot.rvels.append(rotationalVelocity)
    io.setRotational(rotationalVelocity)
    
    inp = io.SensorInput()
    print inp.odometry.x

def brainStop():
    withClips = [(x,max(-0.5,min(0.5,x))) for x in robot.rvels]
    #plotWindow.PlotWindow(title="distanceRight vs time").plot(robot.distances)
    #plotWindow.PlotWindow(title="rotationalVelocity vs time").plot(withClips)

def shutdown():
    pass

# for three sensors spaced pi/3 away from each other one in the center
# def getDistanceRightAndAngle(s):
#     sonarMax = 1.5
#     if s[2] < sonarMax and s[1] < sonarMax:
#         radius = 0.089/2
#         a = s[1]+radius
#         print "a=", a
#         b = s[2]+radius
#         print "b=", b
#         c = sqrt(a*a+b*b-2*a*b*cos(pi/3))
#         print "c=", c
#         if b/c > 1:
#             return(s[2], None)
#         phi = asin((b/c)*sin(pi/3))
#         print "phi=", phi
#         d = b*sin(phi)
#         print "d=", d
#         if d/a > 1:
#             return(s[2],None)
#         alpha = acos(d/a)
#         print "alpha=", alpha
#         d -= radius
#         alpha -= pi/3
#         return(d,alpha)
#     elif s[2] < sonarMax:
#         return(s[2], None)
#     elif s[1] < sonarMax:
#         return(s[1], None)
#     else:
#         return(1.5, None)

# for three sensors spaced pi/2 away from each other one in the center
# def getDistanceRightAndAngle(s):
#     sonarMax = 0.8
#     #print s[7], s[3]
#     right = s[2]
#     center = s[1]
#     #right = s[7]
#     #center = s[3]
#     if right < sonarMax and center < sonarMax:
#         radius = 0.089/2
#         a = center+radius
#         b = right+radius
#         c = math.sqrt(a*a+b*b)
#         phi = math.asin(a/c)
#         d = b*math.sin(phi)
#         alpha = math.acos(d/a)
#         d -= radius
#         alpha -= math.pi/2
#         return(d, alpha)
#     elif right < sonarMax:
#         return(right, None)
#     elif center < sonarMax:
#         return(center, None)
#     else:
#         return(0.7, None)

def getDistanceRightAndAngle(s):
    angleSpaced = 65*math.pi/180#(40+20*2)*math.pi/180
    right = s[2]
    center = s[1]
    sonarMax = 0.8
    #print "right: ", right, " center: ", center
    # if right > center:
    #     return(center, 0)
    if right < sonarMax and center < sonarMax:
        radius = 0.089/2 #little robot radius
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
