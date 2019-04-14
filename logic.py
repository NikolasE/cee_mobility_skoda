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


class ExternalMirror:
    def __init__(self):
        car_arduino = '/dev/serial/by-id/usb-Adafruit_Adafruit_Feather_M0_245098A9504B5957372E314AFF02291C-if00'
        self.ser = serial.Serial(car_arduino, 115200)
        self.filename = '/tmp/window.yaml'

        self.vertical = self.horizontal = -1
        self.load_position()

        if self.vertical < 0:
            self.run_homing()

        self.max_hor = 6
        self.max_vert = 6

    def test(self):
        self.vertical = 42
        self.horizontal = 18
        self.save_position()

        self.vertical = self.horizontal = 0
        self.load_position()
        assert self.vertical == 42
        assert self.horizontal == 18

    def run_homing(self):
        rospy.loginfo("Homing horizontal")

        self.move_rel_horizontal(-2, force=True)
        self.move_rel_horizontal(-2, force=True)
        self.move_rel_horizontal(-2, force=True)

        self.horizontal = 0

        rospy.loginfo("Homing vertical")
        self.move_rel_vertical(-2, force=True)
        # self.move_rel_vertical(-2, force=True)
        # self.move_rel_vertical(-2, force=True)


        self.vertical = 0
        self.save_position()

    def stop(self):
        self.ser.write('X')

    def move_abs_horizontal(self, new_pos):
        # print("newpos: ")
        dt = new_pos - self.horizontal
        return self.move_rel_horizontal(dt)

    def move_rel_horizontal(self, dt=0.5, force=False):
        new_pos = self.horizontal + dt

        if (not force) and not (0 <= new_pos <= self.max_hor):
            rospy.logwarn("Current pos: %.1f, can't move %.1f (max at %.1f)" % (self.horizontal, dt, self.max_hor))
            return False

        self.ser.write('l' if dt < 0 else 'r')
        rospy.sleep(abs(dt))
        self.stop()
        self.horizontal = new_pos
        self.save_position()
        return True

    def move_abs_vertical(self, new_pos):
        dt = new_pos - self.vertical
        return self.move_rel_vertical(dt)

    def move_rel_vertical(self, dt=0.5, force=False):
        new_pos = self.vertical + dt

        if (not force) and not (0 <= new_pos <= self.max_vert):
            rospy.logwarn("Current pos: %.1f, can't move %.1f (max at %.1f)" % (self.vertical, dt, self.max_vert))
            return

        self.ser.write('u' if dt < 0 else 'd')
        rospy.sleep(abs(dt))
        self.stop()
        self.vertical = new_pos
        self.save_position()

    def load_position(self):
        if not os.path.exists(self.filename):
            rospy.logwarn("No file at %s" % self.filename)
            return -1, -1

        f = open(self.filename, 'r')
        spl = f.readline().split(',')
        assert len(spl) == 2
        print(spl)
        self.vertical, self.horizontal = map(float, spl)

    def save_position(self):
        f = open(self.filename, 'w')
        f.write("%s, %s" % (self.vertical, self.horizontal))


class InternalMirror:
    def __init__(self):
        servo_arduino = '/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_014A1259-if00-port0'
        self.ser = serial.Serial(servo_arduino, 115200)
        self._min_cmd = 800
        self._max_cmd = 2200

    def move_to(self, mus, sleep_ms=100):
        assert self.ser.is_open
        # prevent motor from moving out of range
        pos = min(self._max_cmd, max(self._min_cmd, mus))
        self.ser.write('%i\n' % pos)
        rospy.sleep(sleep_ms/1000.0)


class Logic:
    def __init__(self):
        self.sub_yaw = rospy.Subscriber('/yaw', Float32, self.sub_yaw, queue_size=1)
        self.sub_points = rospy.Subscriber('/eyes', Marker, self.sub_eyes, queue_size=1)
        self.yaw = None
        self.invalid_start = None

        self.center_yaw = 10  # 20  # compensates for camera not central in front of user

        self.yaw_threshold = 20  # relative to center_yaw
        self.yaw_threshold_left_look = -25  # relative to center_yaw
        self.last_left_view = None

        self.use_external_mirror = True
        self.use_internal_mirror = False

        if self.use_external_mirror:
            self.external_mirror = ExternalMirror()
            self.last_z_cmd = None

        if self.use_internal_mirror:
            self.internal_mirror = InternalMirror()

        self.yaw_control = False

        self.use_sound_interface = False

        # self.yaw_to_mus = interp1d([35, -35], [self.internal_mirror._min_cmd, self.internal_mirror._max_cmd])
        # self.row_to_mus = interp1d([0, 480], [self.internal_mirror._max_cmd, self.internal_mirror._min_cmd])

        self.engine = pyttsx.init()
        self.engine.say("a")
        self.engine.runAndWait()
        self.engine.say("a")
        self.engine.runAndWait()

    def sub_yaw(self, msg):
        # return
        assert isinstance(msg, Float32)
        self.yaw = msg.data - self.center_yaw
        # rospy.loginfo("new yaw:  %.2f" % self.yaw)

        if self.yaw_control:
            assert not self.use_external_mirror
            self.internal_mirror.move_to(self.yaw_to_mus(int(self.yaw)))

        now = rospy.Time.now()

        # if self.last_left_view is not None:
        #     dt = (now - self.last_left_view).to_sec()
        #     rospy.loginfo("Time since last left view: %.1f s" % dt)
        #
        # if self.yaw < self.yaw_threshold_left_look:
        #     self.last_left_view = now

        if abs(self.yaw) > self.yaw_threshold:
            if self.invalid_start is None:
                self.invalid_start = now
                return

            dt = (now - self.invalid_start).to_sec()
            # rospy.logwarn("%f" % dt)
            if dt > 5 and self.use_sound_interface:
                if not self.engine.isBusy():
                    self.engine.say("Look back, you idiot!")
                return

            if dt > 2 and self.use_sound_interface:
                beep(1, 600)

        if abs(self.yaw) < self.yaw_threshold:
            if self.invalid_start is not None:
                pass
            self.invalid_start = None

    def process_eye_row(self, row):
        return
        cmd = self.row_to_mus(row)
        self.internal_mirror.move_to(cmd)


    def sub_eyes(self, msg):
        assert isinstance(msg, Marker)
        assert len(msg.points) == 2

        y = (msg.points[0].y + msg.points[1].y)/2.0
        self.process_eye_row(y)

        z = (msg.points[0].z + msg.points[1].z)/2.0
        assert z > 0

        if self.use_external_mirror:
            if self.last_z_cmd is None:
                rospy.loginfo("Got first eye-message")
                self.last_z_cmd = z
                return

            dz = abs(z - self.last_z_cmd)

            if dz < 20:
                rospy.loginfo("small movement (%i mm after last move)" % dz)
                # self.last_z_cmd = z
                return

            self.last_z_cmd = z

        print("drive")

        # distance in mm!
        min_z = 400
        max_z = 700

        if self.use_external_mirror:
            mm2cmd = interp1d([min_z, max_z], [2.5, 1.5], fill_value='extrapolate')
            cmd = mm2cmd(z)
            rospy.loginfo("Z=%i, moving to %.1f" % (z, cmd))
            self.external_mirror.move_abs_horizontal(cmd)


        # 350, 1200
        # 260, 1420
        if self.use_internal_mirror:
            try:
                # px2rot = interp1d([290, 340], [1300, 1240], fill_value='extrapolate')
                px2rot = interp1d([300, 340], [1290, 1230], fill_value='extrapolate')
            except ValueError:
                return

            cmd = px2rot(y)
            print("%i %i" % (y, cmd))

            self.internal_mirror.move_to(cmd)


if __name__ == '__main__':
    rospy.init_node("logic")
    l = Logic()
    rospy.spin()

    # em = ExternalMirror()
    # em
    # em.move_abs_vertical(0.9)
    # em.test()
