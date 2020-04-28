#!/usr/bin/env python
# coding: utf-8

# In[42]:


##Import Cell
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from random import seed
from random import random
import sys
from time import time


# -------------Alex's Section--------------

# In[43]:


#Translated from Real Time Collision Detection Chapter 8 C++ section
#Christer Ericson. 2004. Real-Time Collision Detection. CRC Press, Inc., USA.

#define Global constants
CANVAS = [10, 10]
MAX_DEPTH = 100
FLT_MAX = 100 #Find a reliable FLT value
MIN_LEAF_SIZE = 1
Max_Points = 2


#random returns a random 32-bit floating point from 0 to 1, chances of floating points colliding are very close to 0 
POINT_IN_FRONT_OF_PLANE = random()
POINT_BEHIND_PLANE = random()
POINT_ON_PLANE = random()
POINT_ON_BOUNDARY = random()
COPLANAR_WITH_PLANE = random()
IN_FRONT_OF_PLANE = random()
BEHIND_PLANE = random()
STRADDLING_PLANE = random()
POINT_INSIDE = random()
POINT_OUTSIDE = random


POLYGON_STRADDLING_PLANE = random()
POLYGON_COPLANAR_WITH_PLANE = random()
POLYGON_IN_FRONT_OF_PLANE = random()
POLYGON_BEHIND_PLANE = random()
print('epsilon=%g' % sys.float_info.epsilon)
EPSILON = sys.float_info.epsilon
PLANE_THICKNESS_EPSILON = sys.float_info.epsilon


# In[44]:


def ClassifyPointToPlane(p, plane):
  dist = np.dot([plane.n[1][0]- plane.n[0][0], plane.n[1][1] -plane.n[0][1]],
                 [p[0] -plane.n[0][0],p[1] - plane.n[0][1]] ) - plane.d #use np dot
  if dist > PLANE_THICKNESS_EPSILON:
      return POINT_IN_FRONT_OF_PLANE
  elif dist < -PLANE_THICKNESS_EPSILON:
      return POINT_BEHIND_PLANE
  return POINT_ON_PLANE
    


# In[45]:


#returns if polygon is in front or behind plane/line 
def ClassifyPolygonToPlane(poly, plane):
    numInFront = 0
    numBehind = 0
    numVerts = len(poly.points)
    
    #loop through vertices of poly, and see what is in front and behind plane
    for i in range(numVerts):
        p = poly.points[i]
        temp = ClassifyPointToPlane(p,plane)
        if temp == POINT_IN_FRONT_OF_PLANE:
            numInFront += 1
        elif temp == POINT_BEHIND_PLANE:
            numBehind += 1
            
    if numBehind != 0 and numInFront != 0:
        return POLYGON_STRADDLING_PLANE
        #this polygon is straddling, it must be split
    if numInFront != 0:
        return POLYGON_IN_FRONT_OF_PLANE
    if numBehind != 0:
        return POLYGON_BEHIND_PLANE
    #MIGHT BE EXCLUDED IF WE'RE WORKING WITH LINES INSTEAD OF PLANES
    return POLYGON_COPLANAR_WITH_PLANE 


# In[46]:


#Ref: https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def IntersectEdgeAgainstPlane(a, b, plane):
  line1 = [a,b]
  line2 = plane.points
  xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
  ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

  def det(a, b):
      return a[0] * b[1] - a[1] * b[0]

  div = det(xdiff, ydiff)
  if div == 0:
      raise Exception('lines do not intersect')

  d = (det(*line1), det(*line2))
  x = det(d, xdiff) / div
  y = det(d, ydiff) / div
  return x, y


# In[47]:


def SplitPolygon(poly, plane, frontPoly, backPoly):
  #numFront = 0
  #numBack = 0
  frontVerts = []
  backVerts = [] 

  #test every edge (a, b) from last to first vertex
  numVerts = len(poly.points)
  a = poly.points[numVerts - 1]
  aSide = ClassifyPointToPlane(a, plane)
  #print(numVerts)

  for n in range(numVerts):
    b = poly.points[n]
    bSide = ClassifyPointToPlane(b, plane)
    if bSide == POINT_IN_FRONT_OF_PLANE:
      #print("case1")
      if aSide == POINT_BEHIND_PLANE:
        #print(a)
        #print(b)
        #print(plane.points)
        i = IntersectEdgeAgainstPlane(b, a, plane)
        #print(i)
        assert ClassifyPointToPlane(i, plane) == POINT_ON_PLANE
        frontVerts.append(i)
        backVerts.append(i)
      #print("a: " + str(a))
      #print("b: " + str(a))
      frontVerts.append(b)
    elif bSide == POINT_BEHIND_PLANE:
      #print("case2")
      #print(plane.points)
      #print(plane.n)
      #print(plane.d)
      #print(a)
      #print(b)
      if aSide == POINT_IN_FRONT_OF_PLANE:
        i = IntersectEdgeAgainstPlane(a, b, plane)
        assert ClassifyPointToPlane(i, plane) == POINT_ON_PLANE
        frontVerts.append(i) 
        backVerts.append(i)
      elif aSide == POINT_ON_PLANE:
        backVerts.append(a)
      backVerts.append(b)
    else:
      #print("case3")
      frontVerts.append(b)
      if aSide == POINT_BEHIND_PLANE:
        backVerts.append(b)
      
    a = b
    aSide = bSide

  #print(frontVerts)
  #print(backVerts)

  frontPoly = line(frontVerts)
  backPoly = line(backVerts)

  return frontPoly, backPoly


# In[48]:


def PointInSolidSpace(node, p):
  while(not node.IsLeaf):
    #distance of point to diciding plane
    dist = np.dot([node.polygons[0].n[1][0] - node.polygons[0].n[0][0], node.polygons[0].n[1][1] - node.polygons[0].n[0][1] ],[ p[0] - node.polygons[0].n[0][0], p[1] - node.polygons[0].n[0][1]]) - node.polygons[0].d
    if dist > EPSILON: #Pick arbitrary epsilon or refer to pg 442 of book
      #epsilon ust be larger than thickness term
      #print("front")
      
      if node.front == None:
        #print("PIN")
        return POINT_INSIDE
      node = node.front
    elif dist < -EPSILON:
      #Pt behind of plane, so traverse bbackwards
      #print("back")
      
      if node.back == None:
        #print("PO")
        return POINT_OUTSIDE
      node = node.back
    else: 
      #print("Both")
      #print(node.front)
      #print(node.back)
      if node.front == None or node.back == None:
        return POINT_ON_BOUNDARY
      front = PointInSolidSpace(node.front, p)
      back = PointInSolidSpace(node.back, p)
      if front == back:
        return front
      else:
        return POINT_ON_BOUNDARY 
  if node.IsSolid:
    return POINT_INSIDE
  else:
    return POINT_OUTSIDE


# In[49]:


def RayIntersect(node, p, d, tmin, tmax, thit):
  nodeStack = []
  timeStack = None

  assert node!= NULL
  while True:
    if not node.IsLeaf:
      denom = np.dot(node.plane.n, d)
      dist = node.plane.d - np.dot(node.plane.n, p)
      if dist > 0:
        nearIndex = 1
      else:
        nearIndex = 0
      if denom != 0:
        t = dist/denom
        if 0 <= t and t <= tmax:
          if t >= tmin:
            nodeStack.append(node.child[1 ^ nearIndex])
            timeStack.append(tmax)
            tmax = t
          else:
            nearIndex = 1 ^ nearIndex
      node = node.Child[nearIndex]
    else:#Leaf node has been reached, if solid, then record hit at Tmin
      if node.IsSolid:
        thit = tmin
        return 1
      if not nodeStack:
        break
      tmin = tmax
      node = nodeStack[len(nodeStack)]
      nodeStack.pop()
      tmax = timeStack[len(nodeStack)]
  return 0


# In[50]:


def PolygonInSolidSpace(p, node):
  frontPart = None
  backPart = None
  while(not node.IsLeaf):
    c = ClassifyPolygonToPlane(p, node.plane)
    if c == POLYGON_IN_FRONT_OF_PLANE:
      node = node.child[0]
    elif c == POLYGON_BEHIND_PLANE:
      node = node.child[1]
    elif c == POLYGON_STRADDLING_PLANE:
      SplitPolygon(p, node.plane, frontPart, backPart)
      if PolygonInSolidSpace(frontPart, node.child[0]):
        return 1
      if PolygonInSolidSpace(backpart, node.child[1]):
        return 1
      return 0
  return node.IsSolid()


# In[51]:


def PickSplittingPlane(polygons):
    K = 0.8
    
    #bestPlane = Plane(0,0)
    bestScore = FLT_MAX
    #Try the plane of each polygon as a dividing plane
    for i in range(len(polygons)):
        numInFront = 0
        numBehind = 0
        numStraddling = 0
        #planes refer to line segments 
        plane = polygons[i]
        #Test vs other polys
        for j in range(len(polygons)):
            if i == j:
                continue
            
            temp = ClassifyPolygonToPlane(polygons[j], plane)
            if temp == POLYGON_COPLANAR_WITH_PLANE or temp == POLYGON_IN_FRONT_OF_PLANE:
                #Coplanars are treated as being in front of plane
                numInFront += 1
            elif temp == POLYGON_BEHIND_PLANE:
                numBehind += 1
            elif temp == POLYGON_STRADDLING_PLANE:
                numStraddling += 1
        score = K * numStraddling + (1 - K) * abs(numInFront - numBehind)
        #print("Score: " + str(score) + "\nBest Score: " + str(bestScore))
        if score < bestScore:
            bestScore = score
            bestPlane = plane
    return bestPlane


# In[52]:


class BSPNode:
  def __init__(self, Polygons, front, back):
    self.front=front
    self.back=back
    self.polygons = Polygons #Set of convex shapes
    self.leftSize=0
    if front == None and back == None and len(Polygons) == 0:
      self.IsLeaf = False
      self.front = BSPNode(None, None, None)
      self.front.IsSolid = False
      self.front.IsLeaf = True
      self.back = BSPNode(None, None, None)
      self.back.IsSolid = True
      self.back.IsLeaf = True
    else:
      self.IsLeaf = False
    self.plane = None
    self.IsSolid = False

class BSP_Tree:
  def __init__(self):
    self.BSPNodes = []
    self.polylen = 0
    self.BSP_ITERATIONS = 0

  def BuildBSPTree(self, polygons, depth):
      self.BSP_ITERATIONS += 1
      if len(polygons) == 0:
          return  
      #Return empty tree if there are no polygons
      self.polylen += len(polygons)
      self.Nodes = []
      self.root = None
      
      #Get number of polygons in the input vector
      #If criterion for a leaf is matched, create a leaf node from remaining polygons
      if depth >= MAX_DEPTH or self.polylen <= MIN_LEAF_SIZE:
        #print("MAXDEPTH MINLEAF")
        return BSPNode(polygons, None, None);
      
      #Select best possible partitioning plane based on the input geometry
      splitPlane = PickSplittingPlane(polygons)
      
      frontList = []
      backList = []
      
      frontPart = None
      backPart = None

      #print(len(polygons))

      for poly in polygons:
          check = ClassifyPolygonToPlane(poly, splitPlane) 
          if check == COPLANAR_WITH_PLANE or check == IN_FRONT_OF_PLANE:
              frontList.append(poly)
          elif check == BEHIND_PLANE:
              backList.append(poly)
          else:
              frontPart, backPart = SplitPolygon(poly, splitPlane, frontPart, backPart)
              if frontPart != None:
                frontList.append(frontPart)
              if backPart != None:
                backList.append(backPart)
      #print("First Rec: " + str(len(frontList)))
      frontTree = self.BuildBSPTree(frontList, depth + 1)
      #print("Do we get here?"+ str(len(backList)))
      backTree = self.BuildBSPTree(backList, depth + 1)
      return BSPNode(polygons, frontTree, backTree)
    
  def add_polygon(self, poly):
      self.P.append(poly)
        


# In[53]:


#since the dataset is simple squares, it's easy to tell which side is considered the 
#front and back side
class Plane:
  def __init__(self,points, normal,d):
    self.points = points
    p = np.array(points)
    n = np.array(normal)
    self.n = normal
    self.d = d
    self.label = ''
   
    
def line(vertices):
  if len(vertices) <= 1:
    return None

  #print("Vertex Line: " + str(vertices))

  #[[4.9, 4.8], [5.9, 4.8]]

  vertex = vertices
  if (vertices[0][1]-vertices[1][1] == 0):
    #Bottom facing Edge
    d = abs((vertices[1][0]-vertices[0][0])/2)
    n = [[d + vertices[0][0], vertices[1][1]] , [d + vertices[0][0], vertices[1][1] + 1]]
    p = Plane([vertices[0], vertices[1]], n, d)
    edge = p
  else:
    #Right facing Edge
    d = (vertices[1][1]-vertices[0][1])/2
    n = [[vertices[1][0], vertices[1][1] + d] , [vertices[1][0] + 1, vertices[1][1] + d]]
    p = Plane([vertices[0], vertices[1]], n, d)
    edge = p
  edge.label = "line"
  return edge

def Polygon( vertices, label):
  vertex = vertices
  edge = []

  #Bottom Edge
  d = (vertices[1][0]-vertices[0][0])/2
  n = [[d + vertices[0][0], vertices[1][1]] , [d + vertices[0][0], vertices[1][1] - 1]]
  p = Plane([vertices[0], vertices[1]], n, d)
  p.label = "polygon"
  edge.append(p)

  #Right Edge
  d = (vertices[2][1]-vertices[1][1])/2
  n = [[vertices[1][0], vertices[1][1] + d] , [vertices[1][0] + 1, vertices[1][1] + d]]
  p = Plane([vertices[0], vertices[1]], n, d)
  p.label = "polygon"
  edge.append(p)

  #Top Edge
  d = (vertices[2][0]-vertices[3][0])/2
  n = [[d + vertices[3][0], vertices[3][1]] , [d + vertices[3][0], vertices[2][1] + 1]]
  p = Plane([vertices[0], vertices[1]], n, d)
  p.label = "polygon"
  edge.append(p)

  #Left Edge
  d = (vertices[3][1]-vertices[0][1])/2
  n = [[vertices[0][0], vertices[0][1] + d] , [vertices[0][0] - 1, vertices[0][1] + d]]
  p = Plane([vertices[0], vertices[1]], n, d)
  p.label = "polygon"
  edge.append(p)
  return edge

#[label, [x,y], w, h]
def Square_to_Polygon(point):
  w = point[2]
  h = point[3]
  #Starting from bottom left, counter clockwise
  #Bottom Left, Bottom Right, Top Right, Top Left
  vertices = [point[1], [point[1][0] + w, point[1][1]], [point[1][0] + w, point[1][1] + h], [point[1][0], point[1][1] + h]]
  return Polygon(vertices, point[0])
  


# -----------Bi's Section-----------------

# In[54]:


def draw_space(space_data):
    fig1 = plt.figure(figsize=[10, 10])
    ax1 = fig1.gca(aspect='equal')
    ax1.set_ylim(0, 6)
    ax1.set_xlim(0, 6)
    for i in space_data:
        if i[0] == 'rect':
            ax1.add_patch(
                patches.Rectangle(
                    i[1], i[2], i[3]
                )
            )
    plt.show()


# In[55]:


#[label, [x,y], w, h]
testing_data = [
    ['rect', [0.3, 0.1], 1, 0.9],
    ['rect', [0.9, 2.1], 1, 0.9],
    ['rect', [1.99, 0.59], 1.5, 2],
    ['rect', [2.06, 3.5], 1, 0.9],
    ['rect', [0.68, 3.1], 1, 1.1],
    ['rect', [2.68, 4.6], 1, 0.9],
    ['rect', [0.3, 0.1], 1, 0.9],
    ['rect', [4.2, 1.1], 1, 0.9],
    ['rect', [4.0, 2.8], 1, 0.9],
    ['rect', [4.9, 4.8], 1, 0.9]
]
draw_space(testing_data)


# End of Bi's section

# In[56]:


polys = []
for t in testing_data:
  temp = Square_to_Polygon(t)
  for e in temp:
    polys.append(e)
#works
T = BSP_Tree()
N = T.BuildBSPTree(polys, 0)
print(T.BSP_ITERATIONS)

#accuracy test
x = [3, 2]
P = PointInSolidSpace(N, x)
if P == POINT_INSIDE:
  print("Inside")
elif P == POINT_OUTSIDE:
  print("Outside")
else:
  print("ON")


# In[57]:


#Timing test
#https://stackoverflow.com/questions/1557571/how-do-i-get-time-of-a-python-programs-execution

start = time()
print("512: ")
for i in range(512):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))

start = time()
print("1024: ")
for i in range(1024):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))

start = time()
print("2048: ")
for i in range(2048):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))

start = time()
print("4096: ")
for i in range(4096):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))

start = time()
print("8192: ")
for i in range(8192):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))

start = time()
print("16384: ")
for i in range(16384):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))

start = time()
print("65536: ")
for i in range(65536):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))

start = time()
print("131072: ")
for i in range(131072):
  x = [random()*6, random()*6]
  P = PointInSolidSpace(N, x)
print("--- %s seconds ---" % (time() - start))


# In[59]:


#Credits to Ben
x=[512,1024,2048,4096,8192,16384, 65536,131072]
y_BST=[0.3480031490325928 , 0.6360011100769043, 1.5430042743682861, 
          2.5446102619171143, 5.221662998199463, 10.372657775878906,42.85998725891113 ,88.04136610031128]
import matplotlib.pyplot as plt
plt.plot(x,y_BST)
plt.xlabel('data set size')
plt.ylabel('time (seconds)')
plt.title('BST Search for Collision Method Run time')


# In[ ]:




