import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8MultiArray, Int32, Bool

import datetime

class AudioAutonomous:

    # Setup listeners
    def __init__(self):
        print('ROS Setup')

        self.current_velocity = 0.

        # self.ROSTIME = datetime.datetime.now()

        rospy.Subscriber('/zio/odometry/rear', Odometry, self.odo_reader)



    # Callback to store velocity info
    def odo_reader(self, data):
        self.current_velocity = data.twist.twist.linear.x
        # print('Velocity = ', self.current_velocity)
        # self.ROSTIME = self.ROSTIME - datetime.datetime.now()
