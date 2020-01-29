#!/usr/bin/env python
# utility to show cumulative size of all topics grabbed from:
# https://answers.ros.org/question/318667/using-rosbag-to-get-size-of-each-topic/

import rosbag
import sys

###

def sortSecond(val): 
  return val[1]  

###

topic_size_dict = {}

for topic, msg, time in rosbag.Bag(sys.argv[1], 'r').read_messages(raw=True):
  topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
  
topic_size = list(topic_size_dict.items())

topic_size.sort(key = sortSecond, reverse = True)

for topic, size in topic_size:
  print("{:12d} {}".format(size, topic))
