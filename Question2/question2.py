#!/usr/bin/env python
# coding: utf-8

# In[1]:


from z3 import *
import random


# In[2]:


R = 4
N = 5
G = 6
r1 = 0
r2 = 1
r3 = 2
r4 = 3
obs = {(2,0),(3,0),(1,2),(3,2),(1,4),(2,4)}
x0_init = 0
y0_init = 0
x1_init = 0
y1_init = 1
x2_init = 1
y2_init = 0
x3_init = 1
y3_init = 1
final_coords = [(0,0),(0,4),(4,0),(4,4)]
traj_cost = 0
L=6
cost = 22.5


# In[3]:


def initConstraints(t,r,x,y):
    if(r==0 and x==x0_init and y==y0_init) or (r==1 and x==x1_init and y==y1_init) or (r==2 and x==x2_init and y==y2_init) or (r==3 and x==x3_init and y==y3_init):
        return True
    else:
        return False


# In[4]:


P = [[[[Bool("x_%s_%s_%s_%s" % (t, r, x, y)) for y in range(N) ]
      for x in range(N) ] for r in range(R)] for t in range(L+1)]


# In[5]:


C = [[Real("c_%s_%s" % (t, r)) for r in range(R) ]
      for t in range(L)]


# In[6]:


cells_cost  = [ Or(C[t][r] == 0.5, C[t][r] == 1.0, C[t][r] == 1.5)
             for t in range(L) for r in range(R) ]


# In[7]:


constraints = []


# In[8]:


grid_tuples = []
for x in range(N):
    for y in range(N):
        grid_tuples.append((x,y))


# In[9]:


def generate_init_constraints():
    t=0
    for x in range(N):
        for y in range(N):
            for r in range(R):             
                if initConstraints(t,r,x,y) == True:
                    constraints.append(P[t][r][x][y])
                else:
                    constraints.append(Not(P[t][r][x][y]))


# In[10]:


def generate_final_constraints():#takes care of random final positions
    final_coords = [(0,0),(0,4),(4,0),(4,4)]
    t=L
    for (x,y) in final_coords:
        constraints.append(Or(P[t][r1][x][y],P[t][r2][x][y],P[t][r3][x][y],P[t][r4][x][y]))
    


# In[11]:


def motionPrimitives(t,r,x,y):
    if x==0:
        if y==0:
            constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x+1][y],C[t][r]==1.0),And(P[t][r][x][y+1],C[t][r]==1.0),And(P[t][r][x+1][y+1],C[t][r]==1.5)))
        elif y<N-1:
            constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x+1][y],C[t][r]==1.0),And(P[t][r][x][y+1],C[t][r]==1.0),
                                  And(P[t][r][x][y-1],C[t][r]==1.0),And(P[t][r][x+1][y+1],C[t][r]==1.5),And(P[t][r][x+1][y-1],C[t][r]==1.5)))
        elif y==N-1:
            constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x+1][y],C[t][r]==1.0),And(P[t][r][x][y-1],C[t][r]==1.0),
                                  And(P[t][r][x+1][y-1],C[t][r]==1.5)))
    elif y==0:
        if x<N-1:
            constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x-1][y],C[t][r]==1.0),And(P[t][r][x+1][y],C[t][r]==1.0),
                                  And(P[t][r][x][y+1],C[t][r]==1.0),And(P[t][r][x-1][y+1],C[t][r]==1.5),And(P[t][r][x+1][y+1],C[t][r]==1.5)))
        elif x==N-1:
            constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x][y+1],C[t][r]==1.0),And(P[t][r][x-1][y],C[t][r]==1.0)
                                  ,And(P[t][r][x-1][y+1],C[t][r]==1.5)))
    elif x==N-1:
        if y<N-1:
            constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x-1][y],C[t][r]==1.0),
                                  And(P[t][r][x][y+1],C[t][r]==1.0),And(P[t][r][x][y-1],C[t][r]==1.0),
                                  And(P[t][r][x-1][y+1],C[t][r]==1.5),And(P[t][r][x-1][y-1],C[t][r]==1.5)))
        elif y==N-1:
            constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x][y-1],C[t][r]==1.0),
                                  And(P[t][r][x-1][y],C[t][r]==1.0),And(P[t][r][x-1][y-1],C[t][r]==1.5)))
    elif y==N-1:
        constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x][y-1],C[t][r]==1.0),
                              And(P[t][r][x+1][y],C[t][r]==1.0),And(P[t][r][x-1][y],C[t][r]==1.0),
                              And(P[t][r][x-1][y-1],C[t][r]==1.5),And(P[t][r][x+1][y-1],C[t][r]==1.5)))
    else:
        constraints.append(Or(Not(P[t+1][r][x][y]),And(P[t][r][x][y],C[t][r]==0.5),And(P[t][r][x+1][y],C[t][r]==1.0),And(P[t][r][x][y+1],C[t][r]==1.0),
                              And(P[t][r][x-1][y],C[t][r]==1.0),And(P[t][r][x][y-1],C[t][r]==1.0),
                             And(P[t][r][x-1][y-1],C[t][r]==1.5),And(P[t][r][x+1][y-1],C[t][r]==1.5),
                              And(P[t][r][x+1][y+1],C[t][r]==1.5),And(P[t][r][x-1][y+1],C[t][r]==1.5)))


# In[12]:


#motion primitives
def generate_motion_primitives():
    for x in range(N):
        for y in range(N):
            for r in range(R):
                for t in range(L):
                    motionPrimitives(t,r,x,y)


# In[13]:


def obstacleAvoidance(t,r,x,y):
    constraints.append(Not(P[t][r][x][y]))


# In[14]:


def generate_obstacle_constraints():
    for r in range(R):   
        for t in range(L+1):
            for (x,y) in obs:
                obstacleAvoidance(t,r,x,y)


# In[15]:


def sameSpaceCollisionAvoidance(t,x,y):
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r2][x][y]),Not(P[t][r3][x][y]),Not(P[t][r4][x][y])))


# In[16]:


#Generate same space collision avoidance constraints
def generate_same_space_collision_constraints():
    for x in range(N):
        for y in range(N):  
            for t in range(L+1):
                sameSpaceCollisionAvoidance(t,x,y)


# In[17]:


def hoizontal_headOnCollisionAvoidance(t,x,y):
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r2][x+1][y]),Not(P[t+1][r1][x+1][y]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r1][x+1][y]),Not(P[t+1][r2][x+1][y]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r3][x+1][y]),Not(P[t+1][r1][x+1][y]),Not(P[t+1][r3][x][y])))
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r1][x+1][y]),Not(P[t+1][r3][x+1][y]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r4][x+1][y]),Not(P[t+1][r1][x+1][y]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r1][x+1][y]),Not(P[t+1][r4][x+1][y]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r2][x+1][y]),Not(P[t+1][r3][x+1][y]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r3][x+1][y]),Not(P[t+1][r2][x+1][y]),Not(P[t+1][r3][x][y])))
    
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r2][x+1][y]),Not(P[t+1][r4][x+1][y]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r4][x+1][y]),Not(P[t+1][r2][x+1][y]),Not(P[t+1][r4][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r4][x+1][y]),Not(P[t+1][r3][x+1][y]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r3][x+1][y]),Not(P[t+1][r4][x+1][y]),Not(P[t+1][r3][x][y])))
    
    


# In[18]:


#Generate horizontal head on collision avoidance constraints
def generate_horizontal_headon_collision_constraints():
    for x in range(N-1):
        for y in range(N):  
            for t in range(L):
                hoizontal_headOnCollisionAvoidance(t,x,y)


# In[19]:


def vertical_headOnCollisionAvoidance(t,x,y):
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r2][x][y+1]),Not(P[t+1][r1][x][y+1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r1][x][y+1]),Not(P[t+1][r2][x][y+1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r3][x][y+1]),Not(P[t+1][r1][x][y+1]),Not(P[t+1][r3][x][y])))
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r1][x][y+1]),Not(P[t+1][r3][x][y+1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r4][x][y+1]),Not(P[t+1][r1][x][y+1]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r1][x][y+1]),Not(P[t+1][r4][x][y+1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r2][x][y+1]),Not(P[t+1][r3][x][y+1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r3][x][y+1]),Not(P[t+1][r2][x][y+1]),Not(P[t+1][r3][x][y])))
    
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r2][x][y+1]),Not(P[t+1][r4][x][y+1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r4][x][y+1]),Not(P[t+1][r2][x][y+1]),Not(P[t+1][r4][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r4][x][y+1]),Not(P[t+1][r3][x][y+1]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r3][x][y+1]),Not(P[t+1][r4][x][y+1]),Not(P[t+1][r3][x][y])))


# In[20]:


#Generate vertical head on collision avoidance constraints
def generate_vertical_headon_collision_constraints():
    for x in range(N):
        for y in range(N-1):  
            for t in range(L):
                vertical_headOnCollisionAvoidance(t,x,y)


# In[21]:


def diagonal_up_collisionAvoidance(t,x,y):
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r2][x+1][y+1]),Not(P[t+1][r1][x+1][y+1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r1][x+1][y+1]),Not(P[t+1][r2][x+1][y+1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r3][x+1][y+1]),Not(P[t+1][r1][x+1][y+1]),Not(P[t+1][r3][x][y])))
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r1][x+1][y+1]),Not(P[t+1][r3][x+1][y+1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r4][x+1][y+1]),Not(P[t+1][r1][x+1][y+1]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r1][x+1][y+1]),Not(P[t+1][r4][x+1][y+1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r2][x+1][y+1]),Not(P[t+1][r3][x+1][y+1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r3][x+1][y+1]),Not(P[t+1][r2][x+1][y+1]),Not(P[t+1][r3][x][y])))
    
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r2][x+1][y+1]),Not(P[t+1][r4][x+1][y+1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r4][x+1][y+1]),Not(P[t+1][r2][x+1][y+1]),Not(P[t+1][r4][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r4][x+1][y+1]),Not(P[t+1][r3][x+1][y+1]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r3][x+1][y+1]),Not(P[t+1][r4][x+1][y+1]),Not(P[t+1][r3][x][y])))


# In[22]:


def diagonal_down_collisionAvoidance(t,x,y):
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r2][x+1][y-1]),Not(P[t+1][r1][x+1][y-1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r1][x+1][y-1]),Not(P[t+1][r2][x+1][y-1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r3][x+1][y-1]),Not(P[t+1][r1][x+1][y-1]),Not(P[t+1][r3][x][y])))
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r1][x+1][y-1]),Not(P[t+1][r3][x+1][y-1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r1][x][y]),Not(P[t][r4][x+1][y-1]),Not(P[t+1][r1][x+1][y-1]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r1][x+1][y-1]),Not(P[t+1][r4][x+1][y-1]),Not(P[t+1][r1][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r2][x+1][y-1]),Not(P[t+1][r3][x+1][y-1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r3][x+1][y-1]),Not(P[t+1][r2][x+1][y-1]),Not(P[t+1][r3][x][y])))
    
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r2][x+1][y-1]),Not(P[t+1][r4][x+1][y-1]),Not(P[t+1][r2][x][y])))
    constraints.append(Or(Not(P[t][r2][x][y]),Not(P[t][r4][x+1][y-1]),Not(P[t+1][r2][x+1][y-1]),Not(P[t+1][r4][x][y])))
    
    constraints.append(Or(Not(P[t][r3][x][y]),Not(P[t][r4][x+1][y-1]),Not(P[t+1][r3][x+1][y-1]),Not(P[t+1][r4][x][y])))
    constraints.append(Or(Not(P[t][r4][x][y]),Not(P[t][r3][x+1][y-1]),Not(P[t+1][r4][x+1][y-1]),Not(P[t+1][r3][x][y])))


# In[23]:


def generate_diagonal_up_constraints():
    for x in range(N-1):
        for y in range(N-1):
            for t in range(L):
                diagonal_up_collisionAvoidance(t,x,y)


# In[24]:


def generate_diagonal_down_constraints():
    for x in range(N-1):
        for y in range(1,N):
            for t in range(L):
                diagonal_down_collisionAvoidance(t,x,y)


# In[25]:


def generate_grid_visit_constraints():
    for (x,y) in grid_tuples:
        visit = []
        for t in range(L+1):
            for r in range(R):
                if (x,y) not in obs:
                    visit.append(P[t][r][x][y])
        if(len(visit)!=0):
            constraints.append(Or(visit))
                


# In[26]:


def generate_only_at_one_place():
    for (x0,y0) in grid_tuples:
        for t in range(L+1):
            for r in range(R):
                for (x,y) in grid_tuples:
                    if x!=x0 or y!=y0:
                        constraints.append(Not(And(P[t][r][x0][y0],P[t][r][x][y])))
                


# In[27]:


def generate_constraints():
    generate_init_constraints()
    generate_final_constraints()
    generate_motion_primitives()
    generate_obstacle_constraints()
    generate_same_space_collision_constraints()
    generate_horizontal_headon_collision_constraints()
    generate_vertical_headon_collision_constraints()
    generate_diagonal_up_constraints()
    generate_diagonal_down_constraints()
    generate_grid_visit_constraints()
    generate_only_at_one_place()


# In[28]:


def calc_traj_cost():
    total_cost = 0
    cost = 0
    for r in range(R):
        cost = 0
        for t in range(L):
            for x in range(N):
                for y in range(N):
                    if P[t][r][x][y] in val:
                        if ( x!=N-1 and (P[t+1][r][x+1][y] in val)) or (x!=0 and (P[t+1][r][x-1][y] in val)) or (y!=N-1 and (P[t+1][r][x][y+1] in val)) or (y!=0 and (P[t+1][r][x][y-1] in val)):
                            cost = cost + 1
                        elif ((x!=N-1 and y!=0) and (P[t+1][r][x+1][y-1] in val)) or ((x!=N-1 and y!=N-1) and (P[t+1][r][x+1][y+1] in val)) or ((x!=0 and y!=N-1) and (P[t+1][r][x-1][y+1] in val)) or ((x!=0 and y!=0) and (P[t+1][r][x-1][y-1] in val)):
                            cost = cost + 1.5
                        elif (P[t+1][r][x][y] in val):
                            cost = cost + 0.5
        print("r:",r,"cost:",cost)
        total_cost = total_cost + cost
    print(total_cost)
    return total_cost
                    


# In[29]:


min_cost = 500000
constraints = []
s= Solver()
constraints.append((Sum([ C[t][r] for t in range(L) for r in range(R) ]) < cost))
generate_constraints()
s.add(cells_cost + constraints)
val=[]
if s.check() == sat:
    m = s.model()
    for x in m:
        if is_true(m[x]):
            val.append(x())
    print(val)
    cost = calc_traj_cost()
else:
    print("invalid installation profile")

    


# In[ ]:





# In[ ]:




