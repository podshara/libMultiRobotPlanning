#!/usr/bin/env python3
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

Colors = ['orange', 'blue', 'green', 'pink']


class Animation:
  def __init__(self, map, schedule):
    self.map = map
    self.schedule = schedule

    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    xmax = map["map"]["dimensions"][0] - 0.5
    ymax = map["map"]["dimensions"][1] - 0.5

    # self.ax.relim()
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)

    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    for o in map["map"]["obstacles"]:
      x, y = o[0], o[1]
      self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  args = parser.parse_args()


  with open(args.map) as map_file:
    map = yaml.load(map_file)

    a = []
    for o in map["map"]["obstacles"]:
      x, y = o[0], o[1]
      a.append([x, y])
    
    for o in map["agents"]:
      x, y = o["start"][0], o["start"][1]
      a.append([x, y])

    for o in map["potentialGoals"]:
      x, y = o["points"][0][0], o["points"][0][1]
      a.append([x, y])

    xmax = map["map"]["dimensions"][0]
    ymax = map["map"]["dimensions"][1]
    for i in range(xmax):
        line = ""
        for j in range(ymax):
            if [i, j] in a:
                line += "@"
            else:
                line += "."
        print(line)

    for i in range(xmax):
        line = ""
        for j in range(ymax):
            if not [i, j] in a:
                print("[{}, {}]".format(i, j))
