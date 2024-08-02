#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def getchar():
   #Returns a single character from standard input
   import tty, termios, sys
   fd = sys.stdin.fileno()
   old_settings = termios.tcgetattr(fd)
   try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
   finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
   return ch

def joy_emulator():

   print("KEYBOARD JOY EMULATOR.")
   print("-------------------------")
   print("TRANSLATION")
   print("     w     ")
   print("a    s     d")
   print("ROTATION")
   print("q          e")
   print("STOP = spacebar")
   print(" ")
   print("To stop the node hold Ctrl+Z")

   joy_pub = rospy.Publisher('/joy', Joy, queue_size=1)
   joy_msg = Joy()
   rospy.init_node('joy_emulator', anonymous=True)
   rate = rospy.Rate(10)
   
   while not rospy.is_shutdown():
      axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      key = getchar()
      if key == 'w':
         axes[1] = 0.5
      elif key == 'a':
         axes[0] = 0.5
      elif key == 's':
         axes[1] = -0.5
      elif key == 'd':
         axes[0] = -0.5
      elif key == 'q':
         axes[3] = 0.1
      elif key == 'e':
         axes[3] = -0.1
      elif key == ' ':
         pass
      
      joy_msg.axes = axes
      joy_pub.publish(joy_msg)
      rate.sleep()
      

if __name__ == '__main__':   
   try:
      joy_emulator()
   except rospy.ROSInterruptException:
      pass
    