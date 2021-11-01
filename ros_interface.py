import rospy
from nav_msgs.msg import Odometry

import config
# from tone import Tone, ClassTones

# Function to set up listener object to vehicle velocity
# This can be called in a separate process to improve performance
def ros_start(vel_queue_ptr):

    rospy.init_node('audio_listener')
    vel_listener = VelocityListener(vel_queue_ptr)

    print('ROS Setup Completed')

    # Do nothing in main loop
    roscount=0
    while not rospy.is_shutdown():
        roscount += 1
        if roscount > 500000:
            roscount = 0
            # print('Spinning ros thread')


class VelocityListener:

    # Setup listeners
    def __init__(self, vel_queue):
        print('ROS Setup')

        self.current_velocity = 0.

        # Method for passing message back to main process
        self.vel_queue = vel_queue

        rospy.Subscriber('/zio/odometry/rear', Odometry, self.odo_reader)

    # Callback to store velocity info
    def odo_reader(self, data):

        # Read new velocity
        self.current_velocity = data.twist.twist.linear.x
        # print('NewVel')

        # Calculate frequency to be returned
        # newFreq = 25 * self.current_velocity + 120

        # Empty queue if needed
        if self.vel_queue.full():
            self.vel_queue.get()

        # Put new value into the queue, overwriting previous value
        self.vel_queue.put(self.current_velocity, False)

        # print('Velocity = ', self.current_velocity)#, ' frequency = ',newFreq)
