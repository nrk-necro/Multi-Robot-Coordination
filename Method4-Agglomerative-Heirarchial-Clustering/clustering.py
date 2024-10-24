import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import AgglomerativeClustering
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
from copy import deepcopy
from sklearn import datasets
from math import atan
import yaml
from math import sqrt

def dist_calc(o,di):
    td=0
    for i in range(len(o)-1):
        td+=di[o[i]][o[i+1]]
    return td

figure,axis=plt.subplots(1,2)       
robots={}
map_size=20.0
X = np.unique(np.random.randint(1,map_size, size=(30,2)),axis=0)
n=4
start=[np.array([0.0,0.0]),np.array([map_size,map_size]),np.array([0.0,map_size]),np.array([map_size,0.0]),np.array([map_size/2,map_size/2])]
clustering = AgglomerativeClustering(n_clusters=n).fit(X)
labels = clustering.labels_
d={}
for i in range(n):
    d[i]=[]



# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

# Plot result

# Black removed and is used for noise instead.
unique_labels = set(labels)

colors = ['y', 'b', 'g', 'r','m']

for i in range(len(X)):
    d[labels[i]].append(X[i])
    col=colors[labels[i]]
    axis[0].plot(X[i][0], X[i][1], 'o', markerfacecolor='k',
    		markeredgecolor='k',
    		markersize=10)
    axis[1].plot(X[i][0], X[i][1], 'o', markerfacecolor=col,
    		markeredgecolor='k',
    		markersize=10)
    

com=[]
for i in range(len(d)):
    c=(sum(d[i])/len(d[i]))
    com.append(c)

overall_com=sum(com)/len(com)
axis[1].plot(overall_com[0],overall_com[1],'o',markerfacecolor='k',markersize=15,label='Final Goal')

robots['agents']=[]
incomplete=list(range(n))
for m in range(n):
    min_sd=10*sqrt(2)
    start_position=start[m]
    end_position=com[incomplete[0]]
    k=incomplete[0]
    for p in incomplete:
       sd=np.linalg.norm(start_position-com[p])
       if sd<min_sd:
           min_sd=sd
           end_position=com[p]
           k=p
    
    incomplete.remove(k)   
    points=d[k]
    points.insert(0,start_position)
    points.append(end_position)
    dist_list=[]
    points_order=[]      
    
    for i in range(len(points)):
        dist=[]
        for j in range(len(points)):
            if j<i:
                dist.append(dist_list[j][i])
            else:
                dist.append(np.linalg.norm(points[j]-points[i]))
        dist_list.append(dist)
        
    pseudo_order=list(range(len(points)))
    final_order=deepcopy(pseudo_order)
    min_dist=dist_calc(pseudo_order,dist_list)
    for i in range(1,len(points)-2):
        for j in range(i+1,len(points)-1):
            pseudo_order[i],pseudo_order[j]=pseudo_order[j],pseudo_order[i]
            total_dist=dist_calc(pseudo_order,dist_list)
            if total_dist<min_dist:
                min_dist=total_dist
                final_order=deepcopy(pseudo_order)
            pseudo_order[i],pseudo_order[j]=pseudo_order[j],pseudo_order[i]
    

    for f in final_order[1:-1]:
        points_order.append(points[f].tolist())
    
    agent={}
    init_yaw=atan(((map_size/2)-start_position[1])/((map_size/2)-start_position[0]))
    start_position=start_position.tolist()
    start_position.append(init_yaw)
    agent['name']='agent'+str(m)
    agent['start']=start_position
    agent['goal']=end_position.tolist()
    agent['objects']=points_order
    agent['colour']=colors[k]
    robots['agents'].append(agent)
    
robots['map']=dict()
robots['map']['dimensions']=[map_size,map_size] 
robots['map']['obstacles']=[[-1,-1]]
print(robots['agents'][0])
with open('output.yaml','w') as f:
    yaml.dump(robots, f)

 
axis[0].set_title('Before Clustering')
axis[1].set_title('After Clustering (n=%s)'%n_clusters_)
plt.show()
