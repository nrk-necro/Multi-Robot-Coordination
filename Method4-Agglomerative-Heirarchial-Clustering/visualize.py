import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math

Colors = ['yellow', 'blue', 'green', 'red','yellow']
size=0.5
framespermove=3

class Animation:
  def __init__(self, map, schedule, object_remove):
    self.map = map
    self.schedule = schedule
    self.object_remove=object_remove
    self.combined_schedule = {}
    self.combined_schedule.update(self.schedule["schedule"])

    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)

    self.patches = []
    self.artists = []
    self.object_list=[]

    self.objects={}
    self.object_names={}
    self.object_remove_dict=self.object_remove['object']
    self.agent_object={}
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    xmax = map["map"]["dimensions"][0] + 0.5
    ymax = map["map"]["dimensions"][1] + 0.5

    # self.ax.relim()
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # self.ax.set_xticks([])
    # self.ax.set_yticks([])
    # plt.axis('off')
    # self.ax.axis('tight')
    # self.ax.axis('off')

    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    '''for o in map["map"]["obstacles"]:
      x, y = o[0], o[1]
      self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))'''

    # create agents:
    self.T = 0
    # draw goals first
    '''for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      self.patches.append(Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=Colors[0], edgecolor='black', alpha=0.5))'''
    self.c=0
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      for j in d['objects']:
        self.objects[str(self.c)]=(Circle((j[0], j[1]), 0.1, facecolor=Colors[i], edgecolor='black'))
        self.patches.append(self.objects[str(self.c)])
        self.object_names[str(self.c)]=self.ax.text(j[0],j[1], '')
        self.artists.append(self.object_names[str(self.c)])
        self.object_list.append(j)
        self.c+=1

      name = d["name"]
      dist = (size/2)*math.sqrt(2)
      angle = d['start'][2]+math.atan(1)
      self.agents[name] = Rectangle((d["start"][0]-(dist*math.cos(angle)), d["start"][1]-(dist*math.sin(angle))),0.5,0.5,d['start'][2],facecolor='orange', edgecolor='black')
      self.agents[name].original_face_color = 'orange'
      self.patches.append(self.agents[name])
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], '')
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      self.artists.append(self.agent_names[name])

    # self.ax.set_axis_off()
    # self.fig.axes[0].set_visible(False)
    # self.fig.axes.get_yaxis().set_visible(False)

    # self.fig.tight_layout()

    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1)*framespermove,
                               interval=50,
                               repeat=False,
                               blit=True)

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):
    '''if i in self.object_remove_dict:
      for ob in self.object_remove_dict[i]:
        c=self.object_list.index(ob)
        self.objects[str(c)].center=(-1,-1)
        self.object_names[str(c)].set_position((-1,-1))'''
    for agent_name, agent in self.combined_schedule.items():
      j=i/framespermove
      pos = self.getState(i/framespermove, agent)
      d = (size/2)*math.sqrt(2)
      angle = pos[2]+math.atan(1)
      p = (pos[0]-(d*math.cos(angle)), pos[1]-(d*math.sin(angle)))
      if self.agents[agent_name].xy==p:
        stationary=1
      else:
        stationary=0
      self.agents[agent_name].set_xy(p)
      self.agents[agent_name].angle = pos[2] / math.pi * 180
      self.agent_names[agent_name].set_position(p)

      if agent_name not in self.agent_object:
        self.agent_object[agent_name]=[]

      if j in self.object_remove_dict[agent_name]:
        c=self.object_list.index(self.object_remove_dict[agent_name][j])
        self.agent_object[agent_name].append(str(c))
      

      if stationary:
        Y=np.linspace(-0.1,0.1,len(self.agent_object[agent_name]))
        z=0
        for ob in self.agent_object[agent_name]:
          self.objects[ob].center=(Y[z]+pos[0]+((size/2)*math.cos(pos[2])),Y[z]+pos[1]+((size/2)*math.sin(pos[2])))
          self.object_names[ob].set_position(p)
          z+=1
      else:
        for ob in self.agent_object[agent_name]:
          X=(-0.1+(0.2*np.random.random(size=(1,2)))).tolist()
          self.objects[ob].center=(X[0][0]+pos[0]+((size/2)*math.cos(pos[2])),X[0][1]+pos[1]+((size/2)*math.sin(pos[2])))
          self.object_names[ob].set_position(p)
      


    # reset all colors
    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)

    # check drive-drive collisions
    '''agents_array = [agent for _,agent in self.agents.items()]
    for i in range(0, len(agents_array)):
      for j in range(i+1, len(agents_array)):
        d1 = agents_array[i]
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 0.5:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))'''

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0]["x"]), float(d[0]["y"]), float(d[0]["yaw"])])
    elif idx < len(d):
      yawLast = float(d[idx-1]["yaw"])
      yawNext = float(d[idx]["yaw"])
      if (yawLast - yawNext) > math.pi:
        yawLast = yawLast - 2 * math.pi
      elif (yawNext - yawLast) > math.pi:
        yawLast = yawLast + 2 * math.pi
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"]), yawLast])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"]), yawNext])
    else:
      return np.array([float(d[-1]["x"]), float(d[-1]["y"]), float(d[-1]["yaw"])])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos



if __name__ == "__main__":
  '''
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()
  '''
  with open('output.yaml','r') as map_file:
    map = yaml.load(map_file, Loader=yaml.FullLoader)

  with open('schedule.yaml') as states_file:
    schedule = yaml.load(states_file, Loader=yaml.FullLoader)

  with open('object.yaml') as object_file:
    object_remove = yaml.load(object_file, Loader=yaml.FullLoader)

  animation = Animation(map, schedule, object_remove)


  animation.show()