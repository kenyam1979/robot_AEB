import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import matplotlib.pyplot as plt

rclpy.init()

node = Node("listener")
cnt = 0
def cb(msg):
  global cnt
  width = msg.info.width
  height = msg.info.height
  data = msg.data
  cnt += 1
  img = np.array(data)
  print(img.shape)
  img = (img > 99) * 255
  img.dtype = 'uint8'
  print(img.dtype)
  print(img.shape)
  img = img[::-1].reshape([60,60,1], order='F')
  print(img.shape)
  cv2.imshow('test', img)
  cv2.waitKey(100)

node.create_subscription(OccupancyGrid, 'local_costmap/costmap', cb, 10)


rclpy.spin(node)

cv2.destroyAllWindows()

node.destroy_node()


rclpy.shutdown()


