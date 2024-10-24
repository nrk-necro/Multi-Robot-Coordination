import yaml
import copy
import numpy as np
from math import atan

def calc_yaw(direc):
    if direc[0]==0:
        return direc[1]*1.57
    elif direc[1]==0:
        if direc[0]>0:
            return 0.0
        else:
            return 3.14
    elif direc[0]<0:
        if direc[1]<0:
            return atan(direc[1]/direc[0])-3.14
        else:
            return atan(direc[1]/direc[0])+3.14
    else:
        return atan(direc[1]/direc[0])

with open('output.yaml','r') as f:
    try:
        param = yaml.load(f, Loader=yaml.FullLoader)
    except yaml.YAMLError as exc:
        print(exc)

v=0.5
time_interval=1
n_robots=len(param['agents'])

start=[]
com=[]
objects=[]
for i in param['agents']:
    start.append(np.array(i['start'][0:2]))
    com.append(np.array(i['goal']))
    object_arrays=[]
    for j in i['objects']:
        object_arrays.append(np.array([float(j[0]),float(j[1])]))
    object_arrays.append(np.array(i['goal']))
    objects.append(object_arrays)
final_goal=sum(com)/len(com)

goal_reached=[0]*n_robots
output={}
object_remove={}
output['schedule']=dict()
time_list=[]
t=0
current_position=copy.deepcopy(start)
dimension=param['map']['dimensions']

for i in range(n_robots):
    time_list.append([])
    init_yaw=atan(((dimension[1]/2)-current_position[i][1])/((dimension[0]/2)-current_position[i][0]))
    time_list[i].append({'t':t,'x':current_position[i].tolist()[0],'y':current_position[i].tolist()[1],'yaw':init_yaw})
    object_remove['agent'+str(i)]={}


object_no=[-1]*4
while goal_reached!=[1]*n_robots:
    t+=time_interval
    calc_position=[]
    yaw_list=[]
    object_change=[]
    for i in range(n_robots):

        position=current_position[i]
        if goal_reached[i]==1:
            calc_position.append(position)
            yaw_list.append(time_list[i][-1]['yaw'])
            object_change.append(0)
            continue
        
        current_object=objects[i][object_no[i]]

        if t==time_interval or np.linalg.norm(current_object-position)<0.01:
            if object_no[i]!=len(objects[i])-1:
                new_object=objects[i][object_no[i]+1]
            else:
                new_object=final_goal
            
            direction=(new_object-position)/np.linalg.norm(new_object-position)
            calc_position.append(position)
            object_change.append(1)

            yaw_list.append(calc_yaw(direction.tolist()))

            continue

        if np.linalg.norm(current_object-position)<v*time_interval:
            calc_position.append(current_object)
            yaw_list.append(time_list[i][-1]['yaw'])
            #time_list.append({'t':t,'x':current_position[0],'y':current_position[1]})
            object_change.append(0)
            continue

        direction=(current_object-position)/np.linalg.norm(current_object-position)
        yaw_list.append(calc_yaw(direction.tolist()))
        calc_position.append(position+v*direction*time_interval)
        object_change.append(0)
        #time_list.append({'t':t,'x':current_position[0],'y':current_position[1]})
        '''if np.linalg.norm(current_object-current_position)<0.01:
            object_change.append(1)
        else:
            object_change.append(0)'''

    for i in range(n_robots):
        s='agent'+str(i)
        current_position[i]=calc_position[i]
        time_list[i].append({'t':t,'x':current_position[i].tolist()[0],'y':current_position[i].tolist()[1],'yaw':yaw_list[i]})
        if object_change[i]:
            if -1<object_no[i]<(len(objects[i])-1):
                object_remove[s][t]=objects[i][object_no[i]].tolist()
            object_no[i]+=1
            if object_no[i]==len(objects[i]):
                goal_reached[i]=1
else:
    final_goal_reached=[0]*n_robots
    while final_goal_reached!=[1]*n_robots:
        t+=time_interval
        for i in range(n_robots):
            position=current_position[i]
            if final_goal_reached[i]==1:
                continue
            
            direction=(final_goal-position)/np.linalg.norm(final_goal-position)
            yaw=calc_yaw(direction.tolist())
            if np.linalg.norm(final_goal-position)<3*v*time_interval:
                current_position[i]=final_goal-2*v*time_interval*direction
                time_list[i].append({'t':t,'x':current_position[i].tolist()[0],'y':current_position[i].tolist()[1],'yaw':yaw})
                final_goal_reached[i]=1
                continue
            
            current_position[i]+=v*direction*time_interval
            time_list[i].append({'t':t,'x':current_position[i].tolist()[0],'y':current_position[i].tolist()[1],'yaw':yaw})

            
for i in range(n_robots):
    output['schedule']['agent'+str(i)]=time_list[i]

object_dict={}
object_dict['object']=object_remove    
with open('schedule.yaml','w') as f:
    yaml.dump(output, f)

with open('object.yaml','w') as f:
    yaml.dump(object_dict, f)
