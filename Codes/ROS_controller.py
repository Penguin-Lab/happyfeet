#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import numpy as np
import time

import ROS_penguinlib

# Main
robot = ROS_penguinlib.Robot("happy")

end_code = False
options = {0 : robot.go,
           1 : robot.save_home,
           2 : robot.go_home,
           3 : robot.sair
}
while (end_code == False):
    print("###################################################################")
    print("0 => go")
    print("1 => save_home")    
    print("2 => go_home")
    print("3 => sair")
    print("Select option:")
    aux = int(input())
    end_code = options[aux](robot)