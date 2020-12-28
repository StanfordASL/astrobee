#!/usr/bin/python

import os
import time
import rospy

from sensor_msgs.msg import JointState

class GeckoNode(object):
  def __init__(self):
    self.gpg_n_doubles = 35

    self.file_is_open = False
    self.file_still_reading = False 
    self.file_written = False

    self.line_buffer = []
    self.buffer_len = 10

    self.fn = '65521.TXT'
    self.file_lines = []

    self.pub = rospy.Publisher('/joint_goals', JointState, queue_size=10)
    rospy.Subscriber("/gecko_states", JointState, self.callback)

  def callback(self, msg):
    position = msg.position
    if position[0] != 1.:
      self.file_is_open = False
      return
    self.file_is_open = True
    self.file_still_reading = True

    all_dummy = True
    for ii in range(5):
      if chr(int(position[1+ii])) != '-':
        all_dummy = False
    if all_dummy:
      return

    line_data = ''  
    for ii in range(self.gpg_n_doubles):
      line_data += chr(int(position[1+ii]))
    line_data += '\n'

    if len(self.line_buffer) == self.buffer_len:
      self.line_buffer = self.line_buffer[1:]
    self.line_buffer.append(line_data)
    self.file_lines.append(line_data)

    if len(self.line_buffer) == self.buffer_len and len(set(self.line_buffer)) == 1:
      self.file_still_reading = False
    return


  def send_open_exp(self, IDX):
    msg = JointState()
    msg.name = ['gecko_gripper_open_exp']
    msg.position.append(IDX)
    self.pub.publish(msg)

  def send_record(self):
    msg = JointState()
    msg.name = ['gecko_gripper_record']
    msg.position.append(0.0)
    self.pub.publish(msg) 

  def send_close_exp(self, IDX):
    msg = JointState()
    msg.name = ['gecko_gripper_close_exp']
    msg.position.append(0.0)
    self.pub.publish(msg) 

  def start(self):
      rate = rospy.Rate(10)

      while not rospy.is_shutdown():
        if not self.file_is_open and not self.file_written:
          self.send_open_exp(float(self.fn[:5]))
        elif self.file_is_open and self.file_still_reading:
          self.send_record()
        else:
          self.send_close_exp(float(self.fn[:5]))
          if not self.file_written:
            write_file = open(os.path.join(os.environ['SOURCE_PATH'],self.fn), 'w')
            write_file.writelines(self.file_lines[:-(self.buffer_len-1)])
            # write_file.writelines(self.file_lines)
            write_file.close()
            self.file_written = True
        rate.sleep()

if __name__ == '__main__':
    try:
      rospy.init_node('gecko_gripper_hub', anonymous=True)
      gecko_node = GeckoNode()
      gecko_node.start()
    except rospy.ROSInterruptException:
      pass
