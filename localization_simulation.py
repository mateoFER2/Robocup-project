import random
import os
import sys
import math
import sympy
from collections import namedtuple
from sympy import Intersection, Interval
import localization

TOLLERANCE=10#accepteble tollerance to determine estimate is "close enough" to be considered the real position
NOISE_LIMIT=7
ERROR=3 #possible error of x and y values when calculating position
NUM_OF_SONARS=4
RADIUS=10  #radius of robot
FIELDX=182 #width of field
FIELDY=243 #height of field
SONAR_ANGLES=(0, math.pi*0.5, math.pi, math.pi*1.5)#angles of sonars 0,1,2,3 on the robot

def generate_XYA():
    x = random.randint(RADIUS, FIELDX - RADIUS)
    y = random.randint(RADIUS, FIELDY - RADIUS)
    a=random.uniform(0,2*math.pi)
    return (x,y,a)
def points_distance(a,b):
    return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
errors=0
total=0
noneErrors=0
distanceWhenCorrect=0
for i in range (0 ,10000):

    total+=1
    xya=generate_XYA();
    print ("Real: ","X={} ".format(xya[0])+ "Y={} ".format(xya[1]) +"A={} ".format(xya[2]))

    distances=localization.distances_from_XYA(xya[0]*1.0,xya[1]*1.0,xya[2])
    for j in range(0,len(distances)):
        distances[j]=abs(distances[j]+random.uniform(-NOISE_LIMIT,NOISE_LIMIT))

    estimate=localization.localize(distances,xya[2])

    print("Estimated:  X=%f Y=%f" % (estimate[0],estimate[1]))
    if estimate!=None:
        d=points_distance(estimate,(xya[0],xya[1]))
        if d>TOLLERANCE:
                errors+=1
                print("ERROR: wrong point")
        else :
            distanceWhenCorrect+=d
    else:
        print("ERROR: None")
        noneErrors += 1;




print("\nNumber of tests:",total)
print("Error precentage:",errors/total*100,"%")
print("Average distance between estimate and real position:",distanceWhenCorrect/(total-errors))
print("None errors:",noneErrors)



