#!/usr/bin/python

import os
import sys
import time
import signal
import rospy

from sensor_msgs.msg import JointState

class GeckoNode(object):
    def __init__(self):
      self.gpg_n_doubles = 35

      self.file_is_open = False
      self.file_still_reading = True
      self.file_written = False

      self.file_lines = []
      self.line_buffer = []
      self.buffer_len = 10

      self.pub = rospy.Publisher('/joint_goals', JointState, queue_size=10)
      rospy.Subscriber("/gecko_states", JointState, self.callback)

    def reset(self, fn):
      self.file_is_open = False
      self.file_still_reading = True
      self.file_written = False

      self.file_lines = []
      self.line_buffer = []

      self.fn = str(fn)+'.TXT'
      print(self.fn)

    def callback(self, msg):
      position = msg.position
      if position[0] != 1.:
        self.file_is_open = False
        return
      self.file_is_open = True

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

      if len(self.line_buffer) == self.buffer_len and len(set(self.line_buffer)) == 1:
        self.file_still_reading = False

      if self.file_still_reading:
        self.file_lines.append(line_data)
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

    def seek_record(self, RN):
      msg = JointState()
      msg.name = ['gecko_gripper_seek_record']
      msg.position.append(RN)
      self.pub.publish(msg)

    def send_close_exp(self): 
      msg = JointState()
      msg.name = ['gecko_gripper_close_exp']
      msg.position.append(0.0)
      self.pub.publish(msg) 

    def start(self):
      rate = rospy.Rate(30)

      write_file = open(os.path.join(os.environ['SOURCE_PATH'], self.fn), 'w')
      if os.path.isdir(os.path.join('/etc/robotname')):
        # File name if running on Astrobee robot
        write_file = open(os.path.join('data','gecko_data',self.fn), 'w')
      write_file.writelines('')
      write_file.close()
      lines_so_far = []

      record_num = 0    # 0-indexed
      skip = 50000

      ctr = 0
      while not rospy.is_shutdown():
        if not self.file_is_open and not self.file_written:
          if ctr < 10:
            self.send_open_exp(float(self.fn[:-4]))   # Omit '.TXT' from arg
            self.seek_record(record_num)
            ctr += 1
            rospy.loginfo('Sent command to open {}'.format(self.fn))
        elif self.file_is_open and self.file_still_reading and len(self.file_lines) < skip:
          ctr = 0
          self.send_record()
        else:
          for _ in range(5):
            self.send_close_exp()
          if not self.file_written and len(self.file_lines) == skip:
            write_file = open(os.path.join(os.environ['SOURCE_PATH'], self.fn), 'w')
            if os.path.isdir(os.path.join('/etc/robotname')):
              # File name if running on Astrobee robot
              write_file = open(os.path.join('data','gecko_data',self.fn), 'w')

            lines_so_far.extend(self.file_lines[:-(self.buffer_len-1)])

            write_file.writelines(lines_so_far)
            write_file.close()
            rospy.loginfo('Done writing file {}'.format(self.fn))
            self.file_written = True

        if len(self.file_lines) == skip:
          for _ in range(5):
            self.send_close_exp()
          rospy.loginfo('Read {} lines so far'.format(len(lines_so_far)))
          record_num += skip
          self.reset(self.fn.split('.TXT')[0])
        # elif self.file_written and self.file_is_open:
        #   self.send_close_exp()

        if self.file_written and not self.file_is_open:
          rospy.loginfo('Shutting down node')
          return

        if len(self.file_lines) > 0 and len(self.file_lines) % 500 == 0:
          rospy.loginfo('Number of lines read: {}'.format(len(self.file_lines)))
        rate.sleep()

def handler(signum, frame):
  # Interrupt to close any open file on the gripper in case script terminated
  msg = JointState()
  msg.name = ['gecko_gripper_close_exp']
  msg.position.append(0.0)
  close_pub = rospy.Publisher('/joint_goals', JointState, queue_size=10)
  for _ in range(5):
    close_pub.publish(msg) 
  exit()

if __name__ == '__main__':
  try:
    rospy.init_node('gecko_gripper_hub', anonymous=True)
    signal.signal(signal.SIGINT, handler)

    gecko_node = GeckoNode()
    filenames = sys.argv[1:]

    for fn in filenames:
      gecko_node.reset(fn)
      gecko_node.start()
  except rospy.ROSInterruptException:
    pass
