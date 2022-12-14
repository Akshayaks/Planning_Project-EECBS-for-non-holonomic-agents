#!/usr/bin/env python3
import yaml
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import argparse
import math
import sys

Colors = ['orange', 'blue', 'green' , 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan' , 'black' , 'yellow' , 'magenta']

class Animation:

  def __init__(self, map, schedule):

    self.map = map
    self.schedule = schedule
    self.combined_schedule = {}
    self.combined_schedule.update(self.schedule["schedule"])

    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    self.fig = plt.figure(figsize=(18.5, 10.5))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()

    xmin = 0
    ymin = 0
    xmax = map["map"]["dimensions"][0] 
    ymax = map["map"]["dimensions"][1]
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    # Drawing the grid
    for x in range(0, map["map"]["dimensions"][0]+1):
      self.ax.plot([x, x], [ymin, ymax], color='black', linewidth=0.5)

    for y in range(0, map["map"]["dimensions"][1]+1):
      self.ax.plot([xmin, xmax], [y, y], color='black', linewidth=0.5)

    # Drawing the background
    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))

    # Drawing the obstacles
    for o in map["map"]["obstacles"]:
      x, y = o[0], o[1]
      self.patches.append(Rectangle((x+0.2, y+0.2), 0.6, 0.6, facecolor='red', edgecolor='red'))

    # Drawing the goals
    self.T = 0
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      color = [0,0,0]
      color[i%3] = 1.0 - 0.5*i/len(map["agents"])
      color[(i+1)%2] = 0.5*i/len(map["agents"])
      color = tuple(color)
      # color = Colors[i] # Can be used directly if the number of agents is less than 13
      self.patches.append(Rectangle((d["goal"][0]+0.1, d["goal"][1]+0.1), 0.8, 0.8, facecolor=color, edgecolor=color, alpha=0.5))
    
    # Drawing the agents
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      color = [0,0,0]
      color[i%3] = 1.0 - 0.5*i/len(map["agents"])
      color[(i+1)%2] = 0.5*i/len(map["agents"])
      color = tuple(color)
      # color = Colors[i] # Can be used directly if the number of agents is less than 13
      name = d["name"]
      self.agents[name] = Circle((d["start"][0], d["start"][1]), 0.2, facecolor=color, edgecolor=color, alpha=0.75)
      self.agents[name].heading = Circle((d["start"][0]+0.2, d["start"][1]+0.2), 0.1, facecolor=color, edgecolor=color, alpha=0.75)
      self.agents[name].original_face_color = color
      self.patches.append(self.agents[name])
      self.patches.append(self.agents[name].heading)
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''))
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      self.artists.append(self.agent_names[name])

    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * 10,
                               interval=100,
                               blit=True)

  def save(self, file_name, speed):
    self.anim.save(file_name, "ffmpeg", fps=10 * speed, dpi=200)

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):

    if i == ((self.T+1)* 10-1):

      if args.video:
        print(" \n **** Video saved to: "  + args.video + " ****")
        print("Exiting...")
        sys.exit()
      else:
        if input("Press 'ENTER' to visualize the output again or 'q' and then 'ENTER' to exit... \n") == 'q':
          plt.close()
          sys.exit()

    for agent_name, agent in self.combined_schedule.items():
      pos = self.getState(i/10, agent)
      # 0.5 is added to the position so that the agents are at the center of the grid
      p = (pos[0]+0.5, pos[1]+0.5) # center of the agents circle
      # The radius of the heading circle is 0.1 and the radius of the agents circle is 0.2, 
      # so the center of the heading circle is 0.3 away from the center of the agents circle
      ph = (pos[0]+0.5+0.3*math.cos(pos[2]), pos[1]+0.5+0.3*math.sin(pos[2])) # center of the heading circle
      self.agents[agent_name].heading.center = ph
      self.agents[agent_name].center = p
      self.ax.add_patch(self.agents[agent_name])
      self.ax.add_patch(self.agents[agent_name].heading)
      self.agent_names[agent_name].set_position(p)

    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)
      agent.set_edgecolor(agent.original_face_color)

    return self.patches + self.artists


  def getState(self, t, d): # include theta in the state
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1

    if idx == 0:
      theta = (float(d[0]["theta"])/180*math.pi + math.pi) % (2*math.pi) - math.pi
      return np.array([float(d[0]["x"]), float(d[0]["y"]), theta+math.pi/2])

    elif idx < len(d):
      posLast = np.array([float(d[idx-1]["x"]), float(d[idx-1]["y"])])
      posNext = np.array([float(d[idx]["x"]), float(d[idx]["y"])])

      # convert theta to [-pi, pi] and to radians 
      thetaLast = (float(d[idx-1]["theta"])/180*math.pi + math.pi) % (2*math.pi) - math.pi
      thetaNext = (float(d[idx]["theta"])/180*math.pi + math.pi) % (2*math.pi) - math.pi

    else:
      theta = (float(d[-1]["theta"])/180*math.pi + math.pi) % (2*math.pi) - math.pi
      return np.array([float(d[-1]["x"]), float(d[-1]["y"]), theta+math.pi/2])
    
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast

    if thetaNext - thetaLast > math.pi:
      thetaNext -= 2*math.pi
    elif thetaNext - thetaLast < -math.pi:
      thetaNext += 2*math.pi
    theta = (thetaNext - thetaLast) * t + thetaLast

    return np.array([pos[0], pos[1], theta+math.pi/2])

if __name__ == "__main__":

  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()

  with open(args.map) as map_file:
    map = yaml.load(map_file, Loader=yaml.FullLoader)

  with open(args.schedule) as states_file:
    schedule = yaml.load(states_file, Loader=yaml.FullLoader)

  animation = Animation(map, schedule)

  if args.video:
    print("\n\nSaving video...")
    print("This may take a while... \n")
    print("The program will exit automatically when the video is saved. \n")
    animation.save(args.video, args.speed)
  else:
    animation.show()