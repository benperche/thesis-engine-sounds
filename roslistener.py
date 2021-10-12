import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8MultiArray, Int32, Bool

import datetime

import multiprocessing as mp

import config
from tone import Tone, ClassTones

class AudioAutonomous:

    # Setup listeners
    def __init__(self, shared_fundFreq):
        print('ROS Setup')

        self.current_velocity = 0.

        self.shared_fundFreq = shared_fundFreq

        # self.ROSTIME = datetime.datetime.now()

        rospy.Subscriber('/zio/odometry/rear', Odometry, self.odo_reader)

        # # Setup frequency sweep values to be used in loop
        # self.count = 0
        # self.ascending = True
        #
        # # Temporary sweep parameters
        # self.UPPER_FREQ = 270
        # self.LOWER_FREQ = 175
        # self.STEP = 2#0.25

        # self.output_pipe = ros_pipe



    # Callback to store velocity info
    def odo_reader(self, data):

        # global ClassTones

        self.current_velocity = data.twist.twist.linear.x
        # print('NewVel')

        # Get any new values from the pipe
        # print(ClassTones[0].fundFreq)
        # if self.output_pipe.poll():
        #     print('Polling')
        #     ClassTones = self.output_pipe.recv()
        # print(ClassTones[0].fundFreq)

        # Tone.fundFreq = self.current_velocity * 100

        # Empty queue if needed
        if self.shared_fundFreq.full():
            self.shared_fundFreq.get()

        # Put new value into the queue, overwriting previous value
        newFreq = 25 * self.current_velocity + 120
        self.shared_fundFreq.put(newFreq, False)

        # for currentTone in self.ros_class_tones[0]:
        #         currentTone.updateFrequency()

        print('Velocity = ', self.current_velocity, ' frequency = ',newFreq)


        # self.output_pipe.send(ClassTones)
        # print('Output sent')
