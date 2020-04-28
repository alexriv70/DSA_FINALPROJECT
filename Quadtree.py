#!/usr/bin/env python
# coding: utf-8

# In[294]:


import random
import time
#Node Class
class Node:
    #inputs of xrange, yrange are both lists lenght of 2
    #not applying to negative coordinations
    def __init__(self,xrange,yrange):
        #ranges are checking if a point is in the range
        self.xr=xrange
        self.yr=yrange
        self.ne=None
        self.se=None
        self.nw=None
        self.sw=None
        #how many points are allowed in the same space
        self.capacity=[]
        
class Point:
    def __init__(self,xval,yval):
        self.x=xval
        self.y=yval

        
def QT(test_size):
    
    def Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid):
        if xr1<x<xr_mid:
            if yr1<y<=yr_mid:
                return 1
            else:
                return 2
        else:
            if yr1<y<=yr_mid:
                return 3
            else:
                return 4
        

    def insert(ref_root,root,node,point,holder):
        
        if root==None:
            node.capacity=[]
            node.capacity.append([point.x,point.y])
            root=node
            return
        
        if point.x>1000 or point.x<0 or point.y>1000 or point.y<0:
            #print('se,this input point is out of range')
            return
        Nte='Not exist'
        x=point.x
        y=point.y
        xr1=root.xr[0]
        xr2=root.xr[1]
        yr1=root.yr[0]
        yr2=root.yr[1]
        xr_mid=(xr2-xr1)/2
        yr_mid=(yr2-yr1)/2
        
        if QT_find(root,point)==True:
           # print('Collision Detected')
            return
            
        
        if yr_mid<0.5 or xr_mid < 0.5:
            #print('collision detected from range')
            return
        
        
        if len(root.capacity)<=6:
            root.capacity.append([x,y])
            return
        
        #after determining each root's range, check which root fits
        #case 0, point is out of range of the whole setted space
        #holder=root.capacity
        if len(root.capacity)>6 and root.capacity != Nte: 
            holder=root.capacity
            root.capacity=Nte
            insert(ref_root,root,node,point,holder)
        if len(holder)>6 and holder != Nte:
            new_node1=Node([xr1,xr_mid],[yr1,yr_mid])
            new_node2=Node([xr1,xr_mid],[yr_mid,yr2])
            new_node3=Node([xr_mid,xr2],[yr1,yr_mid])
            new_node4=Node([xr_mid,xr2],[yr_mid,yr2])

            for item in holder:
                x=item[0]
                y=item[1]
                pt=Point(x,y)
                if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 1:
                    insert(ref_root,root.sw,node,pt,holder)
                if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 2:

                    insert(ref_root,root.nw,node,pt,holder)

                if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 3:

                    insert(ref_root,root.se,node,pt,holder)

                if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 4:

                    insert(ref_root,root.ne,node,pt,holder)
            
               
        #case 1 the point is at range x1
        #Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid)
        if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 1:            
            if root.sw == None:
                new_node=Node([xr1,xr_mid],[yr1,yr_mid])
                new_node.capacity.append([point.x,point.y])
                root.sw=new_node    
                return

            else:
                if len(root.sw.capacity) <=6:
                    root.sw.capacity.append([x,y])
                    return
                else:
                    holder=root.sw.capacity
                    root.sw.capacity=Nte
                    insert(ref_root,root.sw,node,point,holder)

        #case 2 the point is falling into nw section
        if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 2:

            if root.nw == None:
                new_node=Node([xr1,xr_mid],[yr_mid,yr2])
                new_node.capacity.append([point.x,point.y])
                root.nw=new_node    
                return

            else:
                if len(root.nw.capacity) <=6:
                    root.nw.capacity.append([x,y])
                    return
                else:
                    holder=root.nw.capacity
                    root.nw.capacity=Nte
                    insert(ref_root,root.nw,node,point,holder)

        #case 3 the point is falling into se section
        if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 3:

            if root.se == None:
                new_node=Node([xr_mid,xr2],[yr1,yr_mid])
                new_node.capacity.append([x,y])
                root.se=new_node 
                return

            #if ss is not full
            else:
                if len(root.se.capacity) <=6:
                    root.se.capacity.append([point.x,point.y])
                    return
                else:
                    holder=root.se.capacity
                    root.se.capacity=Nte
                    insert(ref_root,root.se,node,point,holder)

        #case 4 the point is falling into ne section
        if Range_helper(x,y,xr1,xr2,xr_mid,yr1,yr2,yr_mid) == 4:
            if root.ne == None:
                new_node=Node([xr_mid,xr2],[yr_mid,yr2])
                new_node.capacity.append([x,y])
                root.ne=new_node    
                return
            #if the space is divided
            else:
                if len(root.ne.capacity) <=6:
                    root.ne.capacity.append([point.x,point.y])
                    return
                else:
                    holder=root.ne.capacity
                    root.ne.capacity=Nte
                    insert(ref_root,root.ne,node,point,holder)
                
    def QT_find(node,target_pt):
        x=target_pt.x
        y=target_pt.y
        xr1=root.xr[0]
        xr2=root.xr[1]
        yr1=root.yr[0]
        yr2=root.yr[1]
        xr_mid=(xr2-xr1)/2
        yr_mid=(yr2-yr1)/2
        if node==None:
            return False
        if node.capacity!='Not exist':
            if [x,y] in node.capacity:
                #print('collision detected')
                return True
        if x>xr_mid and y>yr_mid:
            if node.ne==None:
                return False
            QT_find(node.ne,target_pt)
            
        if x<xr_mid and y<yr_mid:
            if node.sw==None:
                return False
            QT_find(node.se,target_pt)
        if x>xr_mid and y<yr_mid:
            if node.se==None:
                return False
            QT_find(node.se,target_pt)
            
        else:
            if node.nw==None:
                return False
            QT_find(node.nw,target_pt)
            
    
    def QT_inorderT(node):
        if node == None:
            #print('empty tree')
            return
        QT_inorderT(node.nw)
        QT_inorderT(node.ne)
        if node.capacity != 'Not exist':
            print(node.capacity)
        QT_inorderT(node.sw)
        QT_inorderT(node.se)

        
    def random_point_generator(test_size):
        res=[]
        for i in range (test_size):
            res.append([random.randint(1,100),random.randint(1,100)])
        return res
    
    ref_root=Node([0,100],[0,100])
    test_point_set=random_point_generator(test_size)
    #print(test_point_set[0][0])
    root=Node([0,100],[0,100])
    
    time_start_insert=time.time()
    
    for item in test_point_set:
        pt=Point(item[0],item[1])
        insert(ref_root,root,Node([0,100],[0,100]),pt,[])
    
    time_end=time.time()-time_start_insert
    time_find_end=time.time()
    for item in test_point_set:
        pt=Point(item[0],item[1])
        QT_find(root,pt)
    time_find=time.time()-time_find_end
    
    return time_end,time_find
    #print(test_point_set)


t_insert_L=[]
t_find_L=[]
test_size=[512]
for i in range (len(test_size)):
    t_insert, t_find=QT(test_size[i])
    t_insert_L.append(t_insert)
    t_find_L.append(t_find)
print(t_insert,t_find)


# In[293]:


x=[512,1024,2048,4096,8192,16384]
y_insert=[0.01795172691345215, 0.09674072265625, 0.4803588390350342, 
          3.948439121246338, 26.332525730133057, 206.5012173652649]
y_find=[0.0059642791748046875, 0.01795172691345215, 0.0629270076751709, 
        0.2702786922454834, 1.0242640972137451, 3.9534244537353516]
y_bf=[0.02191472053527832, 0.09574270248413086, 0.35704922676086426, 
      1.5039730072021484, 5.879926681518555, 23.56330108642578]
import matplotlib.pyplot as plt
plt.plot(x,y_bf)
plt.xlabel('data set size')
plt.ylabel('time (second)')
plt.title('Brute Force Method Run time')
plt.imshow()


# In[292]:


import random
import time
class Brute_Force:
    def random_point_generator(self,test_size):
        td1=[]
        td2=[]
        for i in range (test_size):
            td1.append([random.randint(1,100),random.randint(1,100)])
            td2.append([random.randint(1,100),random.randint(1,100)])
        return td1,td2
    
    def BF(self,size):
        td1,td2=self.random_point_generator(size)
        
        for i in range (len(td1)):
            for j in range(len(td2)):
                if td1[i]==td2[j]:
                    print('collision detected, collided point is ',td1[i])
        return
    
B=Brute_Force()
t_bf=[]
test_size=[512,1024,2048,4096,8192,16384]
for i in range (len(test_size)):
    time_start=time.time()
    B.BF(test_size[i])
    t_bf.append(time.time()-time_start)
print(t_bf)

