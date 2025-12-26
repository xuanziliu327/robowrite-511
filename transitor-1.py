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
    c[a][0]=(n[2*a+1]-41)/800+0.25
    c[a][1]=(n[2*a]-90)/800+0.1
    
    c[a][2]=0.1


    a+=1
print(c)
#90 41 490 41 490 241 90 241
#[[0.25 0.1  0.1 ]
 #[0.25 0.6  0.1 ]
 #[0.5  0.6  0.1 ]
 #[0.5  0.1  0.1 ]]
#边框数据测试
