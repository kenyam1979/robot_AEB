import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
#import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import math

from sklearn.cluster import DBSCAN

rclpy.init()
node = Node("listener")
cnt = 0

db = DBSCAN(eps=0.1, min_samples=2)

fig = plt.figure()
def cb(msg):
  x = []
  y = []
  data = msg.ranges
  th = math.pi 
  # print(data)
  for r in data:
    if r < float('inf'):
      x.append(math.cos(th) * r)
      y.append(math.sin(th) * r)
    th = th + 2*math.pi / 1080

  X = np.stack([x, y], 1)

  db.fit(X)

  labels = db.labels_
  ax = fig.add_subplot(1,1,1)

  print(np.unique(labels))
  ax.scatter(x, y, s=5, c=labels)

  l_min = 0 #min(labels) 
  l_max = max(labels)
  for l in range(l_min, l_max+1):
    tmp = X[labels == l,]
    x_min = tmp[:,0].min()
    x_max = tmp[:,0].max()
    y_min = tmp[:,1].min()
    y_max = tmp[:,1].max()
    rect = patches.Rectangle(
              xy=(x_min,y_min), 
              width=(x_max-x_min), 
              height = (y_max-y_min),
              fill=False,
              alpha=0.5,
              edgecolor='red',
              linewidth=1)
    ax.add_patch(rect)

  ax.set_xlim(-3, 3)
  ax.set_ylim(-3, 3) 
  plt.pause(0.01)
  plt.clf()

node.create_subscription(LaserScan, 'scan', cb, 30)

rclpy.spin(node)


node.destroy_node()


rclpy.shutdown()


