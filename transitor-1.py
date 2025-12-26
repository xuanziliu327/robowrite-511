#比例400,200————>1:0.5
import numpy as np
import math
a=0

p=input().split()
n=[]
for i in p:
    t=int(i)
    n.append(t)
c=np.zeros((len(n)//2,3))
while a<len(n)/2:
    
    c[a][0]=(n[2*a]-49)/400-0.5
    c[a][1]=(n[2*a+1]-41)/400+0.1
    c[a][2]=0.1


    a+=1
print(c)


