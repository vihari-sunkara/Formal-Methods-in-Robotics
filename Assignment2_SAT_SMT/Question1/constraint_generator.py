import os
R = 2
N = 5
G = 6
r1 = 0
r2 = 1
obs = {(2,0),(3,0),(1,2),(3,2),(1,4),(2,4)}
x0_init = 0
y0_init = 0
x1_init = 4
y1_init = 4
x1_final = 0
y1_final = 0
x0_final = 4
y0_final = 4
# L=8#satisfiable value
L=8


def dimacs(t,r,x,y):
    return t*R*(N**2) + r*(N**2) + x*N + y + 1


def initConstraints(t,r,x,y):
    if(r==0 and x==x0_init and y==y0_init) or (r==1 and x==x1_init and y==y1_init):
        return dimacs(0,r,x,y)
    else:
        return -1*dimacs(0,r,x,y)

def finalConstraints(L,r,x,y):
    if(r==0 and x==x0_final and y==y0_final) or (r==1 and x==x1_final and y==y1_final):
        return dimacs(L,r,x,y)
    else:
        return -1*dimacs(L,r,x,y)

def motionPrimitives(t,r,x,y):
    if x==0:
        if y==0:
            return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x+1,y),dimacs(t,r,x,y+1),-1.5,-1.5)
        elif y<N-1:
            return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x+1,y),dimacs(t,r,x,y+1),dimacs(t,r,x,y-1),-1.5)
        elif y==N-1:
            return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x+1,y),dimacs(t,r,x,y-1),-1.5,-1.5)
    elif y==0:
        if x<N-1:
            return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x-1,y),dimacs(t,r,x+1,y),dimacs(t,r,x,y+1),-1.5)
        elif x==N-1:
            return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x,y+1),dimacs(t,r,x-1,y),-1.5,-1.5)
    elif x==N-1:
        if y<N-1:
            return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x-1,y),dimacs(t,r,x,y+1),dimacs(t,r,x,y-1),-1.5)
        elif y==N-1:
            return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x,y-1),dimacs(t,r,x-1,y),-1.5,-1.5)
    elif y==N-1:
        return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x,y-1),dimacs(t,r,x+1,y),dimacs(t,r,x-1,y),-1.5)
    else:
        return (-1*dimacs(t+1,r,x,y),dimacs(t,r,x,y),dimacs(t,r,x+1,y),dimacs(t,r,x,y+1),dimacs(t,r,x-1,y),dimacs(t,r,x,y-1))


def obstacleAvoidance(t,r,x,y):
    return -1 * dimacs(t,r,x,y)


def sameSpaceCollisionAvoidance(t,x,y):
    return (-1 * dimacs(t,r1,x,y), -1 * dimacs(t,r2,x,y))


def hoizontal_headOnCollisionAvoidance(t,x,y):
    return (-1 * dimacs(t,r1,x,y), -1 * dimacs(t,r2,x+1,y), 
                          -1 * dimacs(t+1,r1,x+1,y), -1 * dimacs(t+1, r2, x, y))


# In[10]:


def vertical_headOnCollisionAvoidance(t,x,y):
    return (-1 * dimacs(t,r1,x,y), -1 * dimacs(t,r2,x,y+1), 
                          -1 * dimacs(t+1,r1,x,y+1), -1 * dimacs(t+1, r2, x, y))

#Generate init and final constraints
def generate_init_constraints(L):
    infile = open("in.txt","w+")
    a='0'

    for x in range(N):
        for y in range(N):
            for r in range(R):             
                a = str(initConstraints(0,r,x,y))
                a = a + ' ' + '0' + '\n'
                infile.write(a)
                a = str(finalConstraints(L,r,x,y))
                a = a + ' ' + '0' + '\n'
                infile.write(a)



    infile.close() 

#Generate motion primitives
def generate_final_constraints(L):
    infile = open("in.txt","a+")
    a=''

    for x in range(N):
        for y in range(N):
            for r in range(R):   
                for t in range(L):
                    (l1,l2,l3,l4,l5,l6) = motionPrimitives(t,r,x,y)
                    if l5==-1.5:
                        a = str(l1) + ' ' + str(l2) + ' ' + str(l3) + ' ' + str(l4) + ' '+ '0' + '\n'
                    elif l6==-1.5:
                        a = str(l1) + ' ' + str(l2) + ' ' + str(l3) + ' ' + str(l4) + ' ' + str(l5) + ' ' + '0' + '\n'
                    else:
                        a = str(l1) + ' ' + str(l2) + ' ' + str(l3) + ' ' + str(l4) + ' ' + str(l5) + ' ' + str(l6)+ ' '+ '0' + '\n'
                    infile.write(a)           
    infile.close() 

#Generate Obstacle avoidance constraints
def generate_obstacle_constraints(L):
    infile = open("in.txt","a+")
    a=''


    for r in range(R):   
        for t in range(L+1):
            for (x,y) in obs:
                a = str(obstacleAvoidance(t,r,x,y))
                a = a + ' ' + '0' + '\n'
                infile.write(a)



    infile.close() 
#Generate same space collision avoidance constraints
def generate_same_space_collision_constraints(L):
    infile = open("in.txt","a+")
    a=''

    for x in range(N):
        for y in range(N):  
            for t in range(L+1):
                l1,l2 = sameSpaceCollisionAvoidance(t,x,y)
                a = str(l1) + ' ' + str(l2) + ' ' + '0' + '\n'
                infile.write(a)

    infile.close() 

#Generate horizontal head on collision avoidance constraints
def generate_horizontal_headon_collision_constraints(L):
    infile = open("in.txt","a+")
    a=''

    for x in range(N-1):
        for y in range(N):  
            for t in range(L):
                l1,l2,l3,l4 = hoizontal_headOnCollisionAvoidance(t,x,y)
                a = str(l1) + ' ' + str(l2) + ' ' + str(l3) + ' ' + str(l4) + ' ' +'0' + '\n'
                infile.write(a)

    infile.close() 

#Generate vertical head on collision avoidance constraints
def generate_vertical_headon_collision_constraints(L):
    infile = open("in.txt","a+")
    a=''

    for x in range(N):
        for y in range(N-1):  
            for t in range(L):
                l1,l2,l3,l4 = hoizontal_headOnCollisionAvoidance(t,x,y)
                a = str(l1) + ' ' + str(l2) + ' ' + str(l3) + ' ' + str(l4) + ' ' +'0' + '\n'
                infile.write(a)

    infile.close() 
def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
        f.close()
    return i + 1
def generateConstraints(L):
    generate_init_constraints(L)
    generate_final_constraints(L)
    generate_obstacle_constraints(L)
    generate_same_space_collision_constraints(L)
    generate_horizontal_headon_collision_constraints(L)
    generate_vertical_headon_collision_constraints(L)
def copyContent(variable_count,clause_count):
    f = open("in.txt")
    f1 = open("infile.txt", "w")
    f1.write('p cnf '+str(variable_count) + ' ' + str(clause_count) + '\n')
    for line in f:
        f1.write(line)
    f.close()
    f1.close()
def findOptimalMakeSpan():
    variable_count = 0
    clause_count = 0
    loop_break = 0
    for P in range(100):
        generateConstraints(P)
        len = file_len("in.txt")
        variable_count = 50 * (P+1)
        clause_count = len
        copyContent(variable_count,clause_count)
        status = os.system('minisat infile.txt out.txt')
        if status==10:
            loop_break = 1
            break
    if loop_break == 1:
        print("final P value: ", P)
findOptimalMakeSpan()
with open('out.txt', 'r') as fin:
    data = fin.read().splitlines(True)
with open('out.txt', 'w') as fout:
    fout.writelines(data[1:])
fin.close()
fout.close()


# In[23]:


all_lines = []
with open("out.txt", 'r') as fobj:
    for line in fobj:
        for num in line.split():
            all_lines.append(int(num))


# In[24]:


pos_values = []
for i in all_lines:
    if i>0:
        pos_values.append(i)
print(pos_values)
index = 0
loop_break = 0
for t in range(L+1):
    for r in range(R):
        for x in range(N):
            for y in range(N):
                if dimacs(t,r,x,y)==pos_values[index]:
                    index = index+1
                    print("t: "+str(t)+" r: "+str(r)+" x: "+str(x)+" y: "+str(y)+" ind: "+str(index))
                    loop_break = 1
                    break
            if loop_break==1:
                loop_break = 0
                break




