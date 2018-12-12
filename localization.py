import random
import os
import sys
import math
import sympy
from collections import namedtuple
from sympy import Intersection, Interval

NUM_OF_SONARS=4
ERROR=1 #possible error of x and y values when calculating positing
RADIUS=10  #radius of robot
FIELDX=182 #width of field
FIELDY=243 #height of field
SONAR_ANGLES=(0, math.pi*0.5, math.pi, math.pi*1.5)#angles of sonars 0,1,2,3 on the robot

class HorzSegment:
    def __init__(self, y, xL, xR):
        if xL < xR:
            self.y= y
            self.xL=xL
            self.xR=xR

    def toString(self):
        return "y={0:.3f} ".format(self.y)+ "xL={0:.3f} ".format(self.xL) + "xR={0:.3f} ".format(self.xR)

    def toStringInt(self):
        return "y={} ".format(self.y)+ "xL={} ".format(self.xL) + "xR={} ".format(self.xR)

    def intersection_2_HorzSegments_special_error(self, h, error):
        if abs(self.y-h.y) < error:
            interval=interval_intersection((self.xL, self.xR),(h.xL, h.xR))
            if interval!=None:
                return HorzSegment((self.y+h.y)/2, interval[0], interval[1])

        else :
            return None

    def intersection_2_HorzSegments(self, h):
        return self.intersection_2_HorzSegments_special_error(h, ERROR)

class VertSegment:

    def __init__(self,x,yD,yU):
        if yD<yU :
            self.x= x
            self.yD=yD
            self.yU=yU

    def toString(self):
        return "x={0:.3f} ".format(self.x)+ "yD={0:.3f} ".format(self.yD) + "yG={0:.3f} ".format(self.yU)

    def toStringInt(self):
        return "x={} ".format(self.x)+ "yD={} ".format(self.yD) + "yG={} ".format(self.yU)


    def intersection_2_VertSegments_special_error(self, v, error):
        if abs(self.x-v.x) < error:
            interval = interval_intersection((self.yD, self.yU), (v.yD, v.yU))
            if interval != None:
                return VertSegment((self.x + v.x) / 2, interval[0], interval[1])

        else:
            return None

    def intersection_2_VertSegments(self, h):
        return self.intersection_2_VertSegments_special_error(h, ERROR)

    def similar_VertSegment(self,v):
        if self.intersection_2_VertSegments(v)==None:
            return False
        return True


class LGraph:

    def __init__(self, x, yD, yU,y,xL,xR):
        self.vertSegment = VertSegment(x, yD, yU)
        self.horzSegment = HorzSegment(y, xL, xR)

    def toString(self):
        return "Vertical:"+ self.__vertSegmnet.toString()+"Horizontal:" + self.__vertSegment.toString()



def distance_from_XYA(x,y,a):
    a=a%(math.pi*2)
    if a > 0 and a < math.pi * 0.5 :
       d1 = math.sqrt((FIELDX -x)**2+(math.tan(a)*(FIELDX-x))**2)
       d2 = math.sqrt(((FIELDY -y)/math.tan(a))**2 + (FIELDY -y)**2)

    elif a > math.pi * 0.5 and a < math.pi :
       d1 = math.sqrt((x) ** 2 + (math.tan(a) * (x)) ** 2)
       d2 = math.sqrt(((FIELDY - y) / math.tan(a)) ** 2 + (FIELDY - y) ** 2)

    elif a > math.pi and a < math.pi*1.5:
       d1 = math.sqrt((x) ** 2 + (math.tan(a) * (x)) ** 2)
       d2 = math.sqrt(((y) / math.tan(a)) ** 2 + (y) ** 2)

    elif a > math.pi*1.5 and a < math.pi * 2:
       d1 = math.sqrt((FIELDX - x) ** 2 + (math.tan(a) * (FIELDX - x)) ** 2)
       d2 = math.sqrt(((y) / math.tan(a)) ** 2 + (y) ** 2)

    return min(d1,d2)-RADIUS;
def distances_from_XYA(x,y,a):
    distances=[]
    for i in range(0,NUM_OF_SONARS):
        distances.append(distance_from_XYA(x,y,a+SONAR_ANGLES[i]))

    return distances
def lGraphFromDA(d,a):
    a=a%(2*math.pi)
    xDistance=math.sqrt((d+RADIUS)**2/(1+(math.tan(a))**2))
    yDistance=math.sqrt((d+RADIUS)**2/(1+(1/math.tan(a))**2))
    if a>0 and a<math.pi/2:
        x=FIELDX-xDistance
        yD=0
        yU=FIELDY-yDistance

        y=FIELDY-yDistance
        xL=0
        xR=FIELDX-xDistance

    elif (a>math.pi*0.5 and a<math.pi):
        x=xDistance
        yD=0
        yU=FIELDY-yDistance

        y=FIELDY-yDistance
        xL=xDistance
        xR=FIELDX
    elif (a > math.pi and a < math.pi*1.5):
        x = xDistance
        yD = yDistance
        yU = FIELDY

        y=yDistance
        xL=xDistance
        xR=FIELDX
    elif (a > math.pi*1.5 and a < math.pi*2 ):
        x =FIELDX - xDistance
        yD = yDistance
        yU = FIELDY

        y=yDistance
        xL=0
        xR=FIELDX-xDistance

    return LGraph(x,yD,yU,y,xL,xR)
def l_graphs_distances_angle(distances,a):
    lGraphs=[]
    for i in range (0,NUM_OF_SONARS):
        if distances[i]!=None :
            lGraphs.append(lGraphFromDA(distances[i],a+SONAR_ANGLES[i]))

    return lGraphs
def intersection_VertSegment_HorzSegment(v,h):
    if h.xL<v.x and v.x<h.xR and v.yD<h.y and h.y<v.yU:
        return (v.x,h.y)
    return None
def all_intersection_points_Vert_Horz(v,h):
    points=[]
    for i in range(0, len(v)):
        for j in range(0,len(h)):
            if v[i]!=None and h[j]!=None and i!=j:
               intersectionPoint=intersection_VertSegment_HorzSegment(v[i],h[j])
               if intersectionPoint!=None:
                   points.append(intersectionPoint)
    return points
def all_vertSegment_intersections(v):
    intersections=[]
    for i in range(0, len(v)-1):
        for j in range(i+1,len(v)):
            if v[i]!=None and v[j]!=None :
               intersection=v[i].intersection_2_VertSegments(v[j])
               if intersection!=None:
                   intersections.append(intersection)
    return intersections
def all_horzSegment_intersections(h):
    intersections=[]
    for i in range(0, len(h)-1):
        for j in range(i+1,len(h)):
            if h[i]!=None and h[j]!=None:
               intersection=h[i].intersection_2_HorzSegments(h[j])
               if intersection!=None:
                   intersections.append(intersection)
    return intersections
def points_similar(p1,p2):
    if abs(p1[0]-p2[0])<ERROR and abs(p1[1]-p2[1])<ERROR:
        return True
    return False
def interval_intersection(a, b):
    start = max(a[0], b[0])
    end = min(a[1], b[1])
    if start < end:
        return (start, end)
    else:
        return None
def lGraph_contains_point(lGraph: LGraph,point):
    if lGraph.vertSegment!=None:
        if abs(lGraph.vertSegment.x-point[0])<ERROR and lGraph.vertSegment.yD<point[1]and point[1]<lGraph.vertSegment.yU:
            return True
    if lGraph.horzSegment!=None:
        if abs(lGraph.horzSegment.y-point[1])<ERROR and lGraph.horzSegment.xL<point[0] and point[0]<lGraph.horzSegment.xR:
            return True
    return False
def print_cross_LGraphs_Results(intersections):
    points = intersections[0]
    print("Points: ",len(points))
    for i in range(0, len(points)):
        s = "x= %f y=%f weight=%d" % (points[i][0][0], points[i][0][1], points[i][1])
        print(s)

    xs = intersections[1]
    print("Vertical lines(Xs):")
    for i in range(0, len(xs)):
        s = xs[i][0].toStringInt() + "weight=%d" % (xs[i][1])
        print(s)

    ys = intersections[2]
    print("Horizontal lines(Ys):")
    for i in range(0, len(ys)):
        s = ys[i][0].toStringInt() + "weight=%d" % (ys[i][1])
        print(s)
def cross_all_LGraphs(lGraphs):
    vertSegments=[]
    horzSegments=[]

    for i in range (0,len(lGraphs)):
        if lGraphs[i].vertSegment!=None:
            vertSegments.append(lGraphs[i].vertSegment)
        if lGraphs[i].horzSegment != None:
            horzSegments.append(lGraphs[i].horzSegment)

    allPoints=all_intersection_points_Vert_Horz(vertSegments,horzSegments)
    pointsWeigted=[]
    if allPoints:
        pointsWeigted.append([allPoints[0],0])
        for i in range(1,len(allPoints)):
            similarFound=False
            for j in range(0, len(pointsWeigted)):
                if points_similar(pointsWeigted[j][0],allPoints[i]) :
                    similarFound=True
                    break

            if similarFound==False :
                 pointsWeigted.append([allPoints[i],0])
    for i in range(0, len(pointsWeigted)):
        for j in range(0,len(lGraphs)):
            if lGraph_contains_point(lGraphs[j],pointsWeigted[i][0]):
                pointsWeigted[i][1]+=1


    vertIntersectionsWeighted=[]
    if vertSegments:
        vertIntersectionsWeighted.append([vertSegments[0],1])
        for i in range(1,len(vertSegments)):
            similarFound=False
            for j in range(0, len(vertIntersectionsWeighted)):
                intersection=vertSegments[i].intersection_2_VertSegments(vertIntersectionsWeighted[j][0])
                if intersection!=None:
                    vertIntersectionsWeighted[j][0]=intersection
                    vertIntersectionsWeighted[j][1]+=1
                    similarFound=True
                    break

            if similarFound==False :
                 vertIntersectionsWeighted.append([vertSegments[i],1])

    horzIntersectionsWeighted=[]
    if horzSegments:
        horzIntersectionsWeighted.append([horzSegments[0],1])
        for i in range(1,len(horzSegments)):
            similarFound=False
            for j in range(0, len(horzIntersectionsWeighted)):
                intersection=horzSegments[i].intersection_2_HorzSegments(horzIntersectionsWeighted[j][0])
                if intersection!=None:
                    horzIntersectionsWeighted[j][0]=intersection
                    horzIntersectionsWeighted[j][1]+=1
                    similarFound=True
                    break

            if similarFound==False :
                 horzIntersectionsWeighted.append([horzSegments[i],1])




    return (pointsWeigted,vertIntersectionsWeighted,horzIntersectionsWeighted)



def localize(distances,a):
    lGraphs=l_graphs_distances_angle(distances,a)
    intersections=cross_all_LGraphs(lGraphs)
    assumedPoints=intersections[0]
    assumedVerts=intersections[1]
    assumedHorzs=intersections[2]


    if assumedPoints:
        """
        bestGuess=assumedPoints[0]
        for i in range(1,len(assumedPoints)):
            if assumedPoints[i][1]>bestGuess[1]:
                bestGuess=assumedPoints[i]
        #return bestGuess[0]
        """
    else:
        return None

    assumedDistances=[]
    distanceErrorSquares = []
    for i in range(0, len(assumedPoints)):
        assumedDistances.append(distances_from_XYA(assumedPoints[i][0][0],assumedPoints[i][0][1],a))
        distanceErrorSquares.append(0)
        for j in range(0,NUM_OF_SONARS):
            if distances[j]!=None:
                distanceErrorSquares[i]+=(distances[j]-assumedDistances[i][j])**2

        #print(assumedPoints[i])
        #print(distances)
        #print(assumedDistances[i],distanceErrorSquares[i])


    if distanceErrorSquares:
        minimumErrorIndex=0
        for i in range(1, len(assumedPoints)):
            if distanceErrorSquares[minimumErrorIndex] > distanceErrorSquares[i]:
                minimumErrorIndex = i

    return assumedPoints[minimumErrorIndex][0]








"""
distances=distances_from_XYA(36.0,86.0,0.362)

intersections=cross_all_LGraphs(l_graphs_distances_angle(distances,0.362))

print_cross_LGraphs_Results(intersections)

print(localize(distances,0.362))
"""