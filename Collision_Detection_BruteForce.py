#!/usr/bin/env python
# coding: utf-8

# In[ ]:


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

