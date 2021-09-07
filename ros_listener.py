import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8MultiArray, Int32, Bool


class AudioAutonomous:

    # Setup listeners
    def __init__(self):
        print('ROS Setup')

        self.current_velocity = 0.

        rospy.Subscriber('/zio/odometry/rear', Odometry, self.odo_reader)


    # Callback to store velocity info
    def odo_reader(self, data):
        self.current_velocity = data.twist.twist.linear.x
        print('Velocity = ', self.current_velocity)
