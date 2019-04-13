#! /usr/bin/python

# apt install sox

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import pyttsx
import os
import serial
from scipy.interpolate import interp1d


def beep(duration=0.2, freq=800):
    os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))


class ArduinoInterface:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self._min_cmd = 800
        self._max_cmd = 2200

    def move_to(self, mus, sleep_ms=100):
        assert (self.ser.is_open)
        # prevent motor from moving out of range
        pos = min(self._max_cmd, max(self._min_cmd, mus))
        self.ser.write('%i\n' % pos)
        rospy.sleep(sleep_ms/1000.0)


class Logic():
    def __init__(self):
        self.sub_yaw = rospy.Subscriber('/yaw', Float32, self.sub_yaw, queue_size=1)
        self.sub_points = rospy.Subscriber('/eyes', Marker, self.sub_eyes, queue_size=1)
        self.yaw = None
        self.invalid_start = None

        self.center_yaw = 0  # 20  # compensates for camera not central in front of user

        self.yaw_threshold = 20  # relative to center_yaw
        self.yaw_threshold_left_look = -25  # relative to center_yaw
        self.last_left_view = None

        self.motor = ArduinoInterface()
        self.yaw_to_mus = interp1d([35, -35], [self.motor._min_cmd, self.motor._max_cmd])
        self.row_to_mus = interp1d([0, 480], [self.motor._max_cmd, self.motor._min_cmd])

        self.yaw_control = False

        self.engine = pyttsx.init()
        self.engine.say("a")
        self.engine.runAndWait()
        self.engine.say("a")
        self.engine.runAndWait()

    def sub_yaw(self, msg):
        # return
        assert isinstance(msg, Float32)
        rospy.loginfo("new yaw:  %.2f" % msg.data)
        self.yaw = msg.data - self.center_yaw
        if self.yaw_control:
            self.motor.move_to(self.yaw_to_mus(int(self.yaw)))

        now = rospy.Time.now()

        # if self.last_left_view is not None:
        #     dt = (now - self.last_left_view).to_sec()
        #     rospy.loginfo("Time since last left view: %.1f s" % dt)
        #
        # if self.yaw < self.yaw_threshold_left_look:
        #     self.last_left_view = now

        if abs(self.yaw) > self.yaw_threshold:
            if self.invalid_start is None:
                # rospy.logwarn("Large yaw detected")
                self.invalid_start = now
                return

            dt = (now - self.invalid_start).to_sec()
            rospy.logwarn("%f" % dt)
            if dt > 5:
                if not self.engine.isBusy():
                    pass
                    # self.print("ASDSADSADSASDD")
                    self.engine.say("Look back, you idiot!")
                return

            if dt > 2:
                beep(1, 600)

        if abs(self.yaw) < self.yaw_threshold:
            if self.invalid_start is not None:
                pass
                # rospy.logwarn("Ending Large yaw ##################")
            self.invalid_start = None

    def sub_eyes(self, msg):
        # return
        assert isinstance(msg, Marker)
        assert len(msg.points) == 2
        # rospy.loginfo("got marker")
        y = (msg.points[0].y +  msg.points[1].y)/2.0
        print(y)
        if not self.yaw_control:
            self.motor.move_to(self.row_to_mus(y))


if __name__ == '__main__':
    rospy.init_node("logic")
    l = Logic()
    rospy.spin()