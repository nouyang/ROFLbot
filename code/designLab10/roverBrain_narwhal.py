import lib601.sm as sm
#import time
from soar.io import io
######################################################################
###
###          Brain methods
###
######################################################################


def setup():
    pass

def brainStart():
    pass

def step():
    #io.setRotational(0.5)
    vNeck,vLeft,vRight,_ = io.getAnalogInputs()
        
    #print vNeck,'Left:',vLeft,'Right:',vRight, 'Difference: ', vRight-vLeft
    #io.setForward(0)
    #io.setRotational(0)

    #move robot backwards or forwards at a rate proportional to the amount of light the photodiodes are reading
    average = (vLeft + vRight) / 16.0 #
    print average

    if vLeft > 0.8 or vRight > 0.8:
        io.setForward(-average+0.05)
        seekLight(vNeck)
        print "light is too close"
    elif vLeft > 0.6 or vRight > 0.6:
        seekLight(vNeck)
        io.setForward(0)
        print "TARGET IN SIGHT"
    elif vLeft > 0.2 or vRight > 0.2:
        seekLight(vNeck)
        io.setForward(average)
        print "seeking light"
    else:
        io.setRotational(0)
        io.setForward(0)
        print "light is off"
    
def brainStop():
    pass

def shutdown():
    pass

def seekLight(vNeck):
    if vNeck < 2.2:
        io.setRotational(-0.3)
    elif vNeck > 2.9:
        io.setRotational(0.3)
    else:
        io.setRotational(0)
