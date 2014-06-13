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
   
    average = (vLeft + vRight)/2
    print vLeft, vRight, average

    if vLeft > 0.18 or vRight > 0.18:
        io.setForward(-average/10)
        seekLight(vNeck)
        print "light is too close"
    elif vLeft > 0.16 or vRight > 0.16:
        seekLight(vNeck)
        io.setForward(0)
        print "TARGET IN SIGHT"
    elif vLeft > 0.155 or vRight > 0.155:
        seekLight(vNeck)
        io.setForward(average/10)
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
