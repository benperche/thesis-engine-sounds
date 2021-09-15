import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8MultiArray, Int32, Bool

import datetime

import config
from tone import Tone, ClassTones

class AudioAutonomous:

    # Setup listeners
    def __init__(self):
        print('ROS Setup')

        self.current_velocity = 0.

        # self.ROSTIME = datetime.datetime.now()

        rospy.Subscriber('/zio/odometry/rear', Odometry, self.odo_reader)

        # Setup frequency sweep values to be used in loop
        self.count = 0
        self.ascending = True

        # Temporary sweep parameters
        self.UPPER_FREQ = 400
        self.LOWER_FREQ = 175
        self.STEP = 0.25



    # Callback to store velocity info
    def odo_reader(self, data):

        # global tone.ClassTones

        self.current_velocity = data.twist.twist.linear.x
        # print('Velocity = ', self.current_velocity)

        # When we receive a new velocity, change the frequeny of the output sound
        # For now, gradually change direction of sine wave
        self.count += 1
        if self.count > 10:
            self.count = 0

            # Change direction when out of bounds
            if Tone.fundFreq > self.UPPER_FREQ:
                self.ascending = False
            elif Tone.fundFreq < self.LOWER_FREQ:
                self.ascending = True

            # Update frequency of fundamental
            if self.ascending:
                Tone.fundFreq += self.STEP
            else:
                Tone.fundFreq -= self.STEP
            print('new freq ', Tone.fundFreq)
            
            # Update frequencies of all other tones
            for currentTone in ClassTones:
                currentTone.updateFrequency()
