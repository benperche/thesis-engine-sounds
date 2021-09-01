#!/usr/bin/env python2

import sys
import math

import rospy
import struct
import argparse
import numpy as np
import threading

from threading import Thread

# Import playsound module
from playsound import playsound

from pynput.keyboard import Key, Listener

import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8MultiArray, Int32, Bool
from geometry_msgs.msg import PoseStamped

#from ackermann_msgs.msg import AckermannDrive
from actionlib_msgs.msg import GoalStatusArray

# from mbf_msgs.msg import ExePathActionFeedback
# from zio_obstacle_msgs.msg import InteractionsStamped

max_number_of_leds = 4
BYTES_PER_LED = 1


class CycleTimer:

    def __init__(self, duration, one_shot=False):

        self.cycle_start_time = rospy.Time.now()
        self.cycle_time = rospy.Duration(duration)
        self.one_shot = one_shot

    def update_timer(self):

        delta_time = rospy.Time.now() - self.cycle_start_time
        if delta_time > self.cycle_time:

            if not self.one_shot:

                while delta_time > self.cycle_time:
                    delta_time -= self.cycle_time

                self.cycle_start_time = rospy.Time.now() - delta_time

        percentage = delta_time.to_sec() / self.cycle_time.to_sec()
        return min(percentage, 1.)


    def reset_time(self):
        self.cycle_start_time = rospy.Time.now()

class LEDAutonomous:

    def __init__(self):

        #listen for keyboard
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        # make the timers global to the class
        self.low_speed_velocity_timer = CycleTimer(4.)
        self.high_speed_velocity_timer = CycleTimer(1.8)
        self.fast_timer = CycleTimer(3.2, one_shot=True)
        self.takeoff_timer = CycleTimer(2.0, one_shot=True)
        self.idle_timer = CycleTimer(2.0,)

        # For road area
        self.low_speed_changeover = 0.0  # speed at which the car goes from "breathing" to showing speed

        self.pullover_flag = True

        self.pub = rospy.Publisher("/zio/led_array", UInt8MultiArray, queue_size=0)

        self.last_key_pressed = ''

        self.bt_state = 0
        self.ped_cross_status = False

        self.modified_speed_request_value = 0.
        self.raw_speed_request_value = 0.
        self.current_velocity = 0.
        self.auto_status = 0
        self.current_goal = None
        self.path_frame = 'utm'

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.led_buffer = np.zeros(max_number_of_leds*BYTES_PER_LED, np.int16)

        #self.brightness_range_map = rospy.get_param("~brightness_range_map", [0., 2., 4., 5., 6., 10., 20., 50.])
        #self.brightness_value_map = rospy.get_param("~brightness_value_map", [255, 252, 10, 5, 3, 1, 0, 0])

        rospy.Subscriber('/zio/odometry/rear', Odometry, self.odo_reader)
        rospy.Subscriber('/bus_stop_bt/bt_state', Int32, self.handle_bus_stop_state)
        rospy.Subscriber('/zio_navigation_server/ped_cross_status', Bool, self.handle_ped_cross_state)

        self.led_angle_map = rospy.get_param("~led_angle_map", [-1., -25. / (147. / 2.), 0., 25. / (147. / 2.), 1.])
        self.led_number_map = rospy.get_param("~led_number_map", [0, 48, 48 + 25, 48 + 50, 48 + 51 + 47])

    def run_keyboard(self):
        with Listener(on_press=self.on_press) as listener:
            listener.join()


    def on_press(self, key):
        #print('{0} pressed'.format(key))
        self.last_key_pressed = str(key)
        if self.last_key_pressed == "u's'":
            self.takeoff_timer.reset_time()
        if self.last_key_pressed == "u'h'":
            self.fast_timer.reset_time()
        

    def modified_speed_request(self, msg):
        self.modified_speed_request_value = msg.speed

    def raw_speed_request(self, msg):
        self.raw_speed_request_value = msg.speed

    def goal_status(self, msg):
        if len(msg.status_list) > 0:
            self.auto_status = msg.status_list[0].status
            if self.auto_status > 1:
                # currently cancelling the auto state - reset the raw speed as this may not be published after the
                #  vehicle goes into this state
                self.raw_speed_request_value = 0
        else:
            self.auto_status = 0

        if self.auto_status == 0:
            self.current_goal = None

    def nav_stack_feedback(self, msg):
        #print("goal detail: ", msg.feedback.dist_to_goal, msg.feedback.angle_to_goal)
        self.dist_to_goal = msg.feedback.dist_to_goal

    def odo_reader(self, data):
        self.current_velocity = data.twist.twist.linear.x

    def handle_bus_stop_state(self, data):
        print("received bus stop state " + str(data.data))
        self.bt_state = data.data
        if self.bt_state == 2:
            self.takeoff_timer.reset_time()
            T = Thread(target=playsound) # create thread
            T.start() # Launch created thread
            playsound ('/home/ian/catkin_ws/src/zio_led_controller/Sound/ProsodicMusicQuestion - Norman.wav')

    def handle_ped_cross_state(self, data):
        print("received ped cross state " + str(data.data))
        self.ped_cross_status = data.data

    def clear_buffer(self, led_buffer):

        for i in range(max_number_of_leds):
            led_buffer[i * BYTES_PER_LED + 0] = int(0)
            # led_buffer[i * BYTES_PER_LED + 1] = int(0)
            # led_buffer[i * BYTES_PER_LED + 2] = int(0)

    def publish_buffer(self, light_0, light_1, light_2, light_3):
        buffer = struct.pack('BBBB', light_0, light_1, light_2, light_3)
        new_msg = UInt8MultiArray()
        new_msg.data = buffer
        self.pub.publish(new_msg)
   
    def send_led_messages(self, topic_name):
        #idle_scrole_cycle = CycleTimer(1.5)
        #startup_blink_cycle = CycleTimer(1.0, one_shot=True)



        while not rospy.is_shutdown():
            rospy.loginfo_throttle(0.5, "auto: " + str(self.auto_status) + ", raw_speed_request: " + str(self.raw_speed_request_value) + ", modified_speed_request: " + str(self.modified_speed_request_value) + ", current_velocity: " + str(self.current_velocity))


            # print (self.last_key_pressed, type(self.last_key_pressed))

        #Quick Flash: to indicate vehicle will start moving
        #Triggered by: Bus stop state "2"
            # self.clear_buffer(self.led_buffer)
            if self.takeoff_timer.update_timer() < .999999:
                print("Take Off")
                cycle_percentage = self.takeoff_timer.update_timer()
                if (cycle_percentage < 0.3):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                elif (cycle_percentage < 0.35):
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)
                elif (cycle_percentage < 0.4):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                elif (cycle_percentage < 0.45):
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)
                elif (cycle_percentage < 0.5):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                elif (cycle_percentage < 0.55):
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)
                elif (cycle_percentage < 0.6):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                elif (cycle_percentage < 0.65):
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)
                else:
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)

              #  if (cycle_percentage >= 1.):
               #     self.last_key_pressed = ""

        #Hazard: Fast strobe to indicate a Hazard
        #Triggered by: key Press "H"
            elif self.last_key_pressed == "u'h'":

                print("Hazard")
                cycle_percentage = self.fast_timer.update_timer()
                if (cycle_percentage < 0.2):
                    self.publish_buffer(light_0=0, light_1=1, light_2=1, light_3=0)
                elif (cycle_percentage < 0.4):
                    self.publish_buffer(light_0=1, light_1=0, light_2=0, light_3=1)
                elif (cycle_percentage < 0.6):
                    self.publish_buffer(light_0=0, light_1=1, light_2=1, light_3=0)
                elif (cycle_percentage < 0.8):
                    self.publish_buffer(light_0=1, light_1=0, light_2=0, light_3=1)
                elif (cycle_percentage < 0.83):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                elif (cycle_percentage < 0.86):
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)
                elif (cycle_percentage < 0.89):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                elif (cycle_percentage < 0.92):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                elif (cycle_percentage < 0.95):
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)
                elif (cycle_percentage < 0.98):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                else:
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)

                if (cycle_percentage >=1.):
                    self.last_key_pressed = ""
 
        #Rolling: Rolling idle to indicate safe to cross
        #Triggered by: Predestrian crossing 'True'
            #self.clear_buffer(self.led_buffer) 
            #elif self.last_key_pressed == "u's'":
            elif self.ped_cross_status == True :
                print("Crossing")
                cycle_percentage = self.low_speed_velocity_timer.update_timer()
                if (cycle_percentage < 0.1666):
                    self.publish_buffer(light_0=1, light_1=0, light_2=0, light_3=0)
                elif (cycle_percentage < 0.3333):
                    self.publish_buffer(light_0=0, light_1=1, light_2=0, light_3=0)
                elif (cycle_percentage < 0.4999):
                    self.publish_buffer(light_0=0, light_1=0, light_2=1, light_3=0)
                elif (cycle_percentage < 0.6665):
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=1)
                elif (cycle_percentage < 0.8331):
                    self.publish_buffer(light_0=0, light_1=0, light_2=1, light_3=0)
                else:
                    self.publish_buffer(light_0=0, light_1=1, light_2=0, light_3=0)
        
        #Steady pulse: to indicate vehicle is operating normaly
        #Triggered by: Bus stop state "1" 
            elif self.bt_state == 1 :
                cycle_percentage = self.high_speed_velocity_timer.update_timer()
                if (cycle_percentage < 0.5):
                    self.publish_buffer(light_0=1, light_1=0, light_2=0, light_3=1)
                else:
                    self.publish_buffer(light_0=0, light_1=1, light_2=1, light_3=0)
        
        #Slow steady pulse: to indicate vehicle is on but inactive
        #Triggered by: Bus stop state "0" 
            elif self.bt_state == 1 :
                cycle_percentage = self.idle_timer.update_timer()
                if (cycle_percentage < 0.1):
                    self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)
                else:
                    self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)


            """
            if self.current_velocity <= self.low_speed_changeover:

                self.publish_buffer(light_0=0, light_1=1, light_2=1, light_3=0)

                rospy.sleep(1)

                self.publish_buffer(light_0=1, light_1=0, light_2=0, light_3=1)

                rospy.sleep(1)
            
            elif self.current_velocity > self.low_speed_changeover:

                self.publish_buffer(light_0=1, light_1=1, light_2=1, light_3=1)

                rospy.sleep(1.5)

                self.publish_buffer(light_0=0, light_1=0, light_2=0, light_3=0)

                rospy.sleep(1.5)
            """
            rospy.sleep(0.05)

if __name__ == '__main__':

    rospy.init_node('test_vehicle_pattern')
    led_autonomous = LEDAutonomous()
    led_autonomous.send_led_messages("led_array")

    rospy.spin()
    led_autonomous.keyboard_thread.stop()

