#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

# ROS
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# OpenCV
import cv2

import argparse
import os.path 
import re
import sys
import tarfile

import numpy as np
from six.moves import urllib

# Tensorflow
import tensorflow as tf

# Location of model's files
pkg_dir = os.path.expanduser("~/catkin_ws/src/robot_privacy_detection/nakedness_detection_googlenet/model/")

# Name of the files of the pre-trained model
model_graph = os.path.join(pkg_dir, 'nakedness_detection_graph.pb')
label_lookup = os.path.join(pkg_dir, 'nakedness_detection_label_map_proto.pbtxt')
uid_lookup = os.path.join(pkg_dir, 'nakedness_detection_labels.txt')

# RPS publisher of the nakedness detection
pub_prediction = rospy.Publisher("/nakedness_detection", String, queue_size=1)

sess = tf.Session()

class NodeLookup(object):
  """Converts integer node ID's to human readable labels."""

  def __init__(self):
    self.node_lookup = self.load(label_lookup, uid_lookup)

  def load(self, label_lookup, uid_lookup):
    """Loads a human readable English name for each softmax node."""
    if not tf.gfile.Exists(uid_lookup):
      tf.logging.fatal('File does not exist %s', uid_lookup)
    if not tf.gfile.Exists(label_lookup):
      tf.logging.fatal('File does not exist %s', label_lookup)

    # Loads mapping from string UID to human-readable string
    proto_as_ascii_lines = tf.gfile.GFile(uid_lookup).readlines()
    uid_to_human = {}
    p = re.compile(r'[n\d]*[ \S,]*')
    for line in proto_as_ascii_lines:
      parsed_items = p.findall(line)
      uid = parsed_items[0]
      human_string = parsed_items[2]
      uid_to_human[uid] = human_string

    # Loads mapping from string UID to integer node ID.
    node_id_to_uid = {}
    proto_as_ascii = tf.gfile.GFile(label_lookup).readlines()
    for line in proto_as_ascii:
      if line.startswith('  target_class:'):
        target_class = int(line.split(': ')[1])
      if line.startswith('  target_class_string:'):
        target_class_string = line.split(': ')[1]
        node_id_to_uid[target_class] = target_class_string[1:-2]

    # Loads the final mapping of integer node ID to human-readable string
    node_id_to_name = {}
    for key, val in node_id_to_uid.items():
      if val not in uid_to_human:
        tf.logging.fatal('Failed to locate: %s', val)
      name = uid_to_human[val]
      node_id_to_name[key] = name

    return node_id_to_name

  def id_to_string(self, node_id):
    if node_id not in self.node_lookup:
      return ''
    return self.node_lookup[node_id]

def run_inference_on_image(image):
  softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
  predictions = sess.run(softmax_tensor,
                           {'DecodeJpeg/contents:0': image})
  predictions = np.squeeze(predictions)

  # Creates node ID --> English string lookup.
  node_lookup = NodeLookup()

  top_k = predictions.argsort()[::-1]
  
  human_string = node_lookup.id_to_string(top_k[0])
  score = predictions[top_k[0]]
  
  print('Prediction: ')
  print('%s (score = %.5f)' % (human_string, score))
  
  if(score >= 0.8):
      pub_prediction.publish(node_lookup.id_to_string(top_k[0]))

def create_graph():
  """Creates a graph from saved GraphDef file and returns a saver."""
  # Creates graph from saved graph_def.pb.
  with tf.gfile.FastGFile(model_graph, 'rb') as f:
    graph_def = tf.GraphDef()
    graph_def.ParseFromString(f.read())
    _ = tf.import_graph_def(graph_def, name='')

def ImageCallback(image): 
  start = rospy.get_time() # Get time to compute FPS

  # Compressed image is already in JPEG format
  run_inference_on_image(image.data)

  end = rospy.get_time()
  elapsed_time = end - start
  fps = 1/elapsed_time
  print('FPS: ' + str(fps) + '\n')

if __name__ == '__main__':
  # Creates graph from saved GraphDef.
  create_graph()
  
  rospy.init_node('nakedness_detection')  
  rospy.Subscriber("/right/camera/image_raw/compressed", CompressedImage, ImageCallback, queue_size=1)
  rospy.spin()
