#比例400,200————>1:0.5
import numpy as np
import math
a=0

p=input().split()
n=[]
for i in p:
    t=float(i)
    n.append(t)
c=np.zeros((len(n)//2,3))
while a<len(n)/2:
    c[a][0]=(n[2*a+1]-41)/800+0.25
    c[a][1]=(n[2*a]-90)/800+0.1
    
    c[a][2]=0.1


    a+=1

#90 41 490 41 490 241 90 241
#[[0.25 0.1  0.1 ]
 #[0.25 0.6  0.1 ]
 #[0.5  0.6  0.1 ]
 #[0.5  0.1  0.1 ]]
#边框数据测试
for i in range(49):
    if i==3 or i==6 or i==14 or i==19 or i==22 or i==25 or i==28 or i==36 or i==40 or i==43 or i==48:
        c[i][2] +=0.02
        print(f"q{i}=np.array([{c[i][0]},{c[i][1]},{c[i][2]}])")
    else:
        print(f"q{i}=np.array([{c[i][0]},{c[i][1]},{c[i][2]}])")
for i in range(52):
    if  i==3 or i==6 or i==10 or i==14 or i==17 or i==21 or i==27 or i==30 or i==33 or i==37 or i==40 or i==43 or i==46 or i==49:
        c[i+49][2] +=0.02
        print(f"a{i+1}=np.array([{c[i+49][0]},{c[i+49][1]},{c[i+49][2]}])")
    else:
        print(f"a{i+1}=np.array([{c[i+49][0]},{c[i+49][1]},{c[i+49][2]}])")
for i in range(105):
    if i ==8 or i==18 or i==26 or i==38 or i==50 or i==64 or i==78 or i==82 or i==94:
        c[i+101][2] +=0.02
        print(f"b{i+1}=np.array([{c[i+101][0]},{c[i+101][1]},{c[i+101][2]}])")
    else:
        print(f"b{i+1}=np.array([{c[i+101][0]},{c[i+101][1]},{c[i+101][2]}])")

