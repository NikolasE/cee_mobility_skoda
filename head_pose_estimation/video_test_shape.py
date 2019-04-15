#! /usr/bin/python

import cv2
import dlib
import numpy as np
from imutils import face_utils
import rospy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sys

face_landmark_path = 'shape_predictor_68_face_landmarks.dat'

K = [6.5308391993466671e+002, 0.0, 3.1950000000000000e+002,
     0.0, 6.5308391993466671e+002, 2.3950000000000000e+002,
     0.0, 0.0, 1.0]

D = [7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000]

cam_matrix = np.array(K).reshape(3, 3).astype(np.float32)
dist_coeffs = np.array(D).reshape(5, 1).astype(np.float32)

object_pts = np.float32([[6.825897, 6.760612, 4.402142],
                         [1.330353, 7.122144, 6.903745],
                         [-1.330353, 7.122144, 6.903745],
                         [-6.825897, 6.760612, 4.402142],
                         [5.311432, 5.485328, 3.987654],
                         [1.789930, 5.393625, 4.413414],
                         [-1.789930, 5.393625, 4.413414],
                         [-5.311432, 5.485328, 3.987654],
                         [2.005628, 1.409845, 6.165652],
                         [-2.005628, 1.409845, 6.165652],
                         [2.774015, -2.080775, 5.048531],
                         [-2.774015, -2.080775, 5.048531],
                         [0.000000, -3.116408, 6.097667],
                         [0.000000, -7.415691, 4.070434]])

reprojectsrc = np.float32([[10.0, 10.0, 10.0],
                           [10.0, 10.0, -10.0],
                           [10.0, -10.0, -10.0],
                           [10.0, -10.0, 10.0],
                           [-10.0, 10.0, 10.0],
                           [-10.0, 10.0, -10.0],
                           [-10.0, -10.0, -10.0],
                           [-10.0, -10.0, 10.0]])

line_pairs = [[0, 1], [1, 2], [2, 3], [3, 0],
              [4, 5], [5, 6], [6, 7], [7, 4],
              [0, 4], [1, 5], [2, 6], [3, 7]]


def get_head_pose(shape):
    image_pts = np.float32([shape[17], shape[21], shape[22], shape[26], shape[36],
                            shape[39], shape[42], shape[45], shape[31], shape[35],
                            shape[48], shape[54], shape[57], shape[8]])

    _, rotation_vec, translation_vec = cv2.solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs)

    reprojectdst, _ = cv2.projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix,
                                        dist_coeffs)

    reprojectdst = tuple(map(tuple, reprojectdst.reshape(8, 2)))

    # calc euler angle
    rotation_mat, _ = cv2.Rodrigues(rotation_vec)
    pose_mat = cv2.hconcat((rotation_mat, translation_vec))
    _, _, _, _, _, _, euler_angle = cv2.decomposeProjectionMatrix(pose_mat)

    return reprojectdst, euler_angle


class FaceLocalizer:
    def __init__(self, use_subscriber=True, debug=True):
        self.with_output = debug
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(face_landmark_path)

        self.pub_yaw = rospy.Publisher("/yaw", Float32, queue_size=1)
        self.pub_eyes = rospy.Publisher("/eyes", Marker, queue_size=1)
        self.bridge = CvBridge()

        self.with_ir = False
        self.with_depth = True

        if not use_subscriber:
            self.with_depth = False

        self.current_depth = None

        if use_subscriber:
            topic = '/ir/image' if self.with_ir else '/rgb/image'
            self.sub_image = rospy.Subscriber(topic, Image, self.image_cb, queue_size=1)
        else:
            self.local_capture_loop()

        if self.with_depth:
            self.sub_depth = rospy.Subscriber('/depth/image_raw', Image, self.depth_image_cb, queue_size=1)

    def depth_image_cb(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg)

    def image_cb(self, img_msg):
        cv_frame = self.bridge.imgmsg_to_cv2(img_msg)
        if self.with_ir:
            cv_frame = cv2.cvtColor(cv_frame, cv2.COLOR_GRAY2BGR)
            cv_frame = cv2.convertScaleAbs(cv_frame)

        cv_frame = cv2.cvtColor(cv_frame, cv2.COLOR_RGB2BGR)
        self.process_image(cv_frame)

    def process_image(self, frame):
        face_rects = self.detector(frame, 0)

        m = Marker()

        if len(face_rects) > 0:
            shape = self.predictor(frame, face_rects[0])

            shape = face_utils.shape_to_np(shape)

            reprojectdst, euler_angle = get_head_pose(shape)

            lx = ly = 0
            rx = ry = 0

            for i, (x, y) in enumerate(shape):
                if i in [36, 39]:
                    lx += x
                    ly += y
                if i in [42, 45]:
                    rx += x
                    ry += y
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            lx /= 2
            ly /= 2
            rx /= 2
            ry /= 2

            cv2.circle(frame, (lx, ly), 5, (255, 0, 0), -1)
            cv2.circle(frame, (rx, ry), 5, (0, 255, 0), -1)

            z = 0

            if self.with_depth:
                if self.current_depth is None:
                    rospy.loginfo("No depth received yet")
                    return

                d = 20
                # print("%i %i, %i %i" % (lx, ly, rx, ry))

                # sub_img = self.current_depth[ly-d:ly+d, lx-d:lx+d]
                # cv2.imwrite('/tmp/eye.png', sub_img)
                z = np.mean(self.current_depth[ly-d:ly+d, lx:rx])
                rospy.loginfo("dist: %f" % z)

            m.points.append(Point(x=lx, y=ly, z=z))
            m.points.append(Point(x=rx, y=ry, z=z))

            # rospy.loginfo("y: %i" % ((ly+ry)/2))

            self.pub_eyes.publish(m)

            # for start, end in line_pairs:
            #     cv2.line(frame, reprojectdst[start], reprojectdst[end], (0, 255, 0), 2)

            yaw = euler_angle[1, 0]

            self.pub_yaw.publish(yaw)

            if self.with_output:
                # color = (255, 255, 255) if self.with_ir else (0, 0, 0)
                #
                color = (0, 255, 0) if abs(yaw) < 15 else (0, 0, 255)

                # cv2.putText(frame, "X: " + "{:7.2f}".format(euler_angle[0, 0]), (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                #             0.75, color, thickness=2)
                # cv2.putText(frame, "Rot: " + "{:3.0f} deg".format(yaw), (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                #             1, color, thickness=2)
                # cv2.putText(frame, "Z: " + "{:7.2f}".format(euler_angle[2, 0]), (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                #             0.75, color, thickness=2)

        if self.with_output:
            cv2.imshow("", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                sys.exit(0)

    def local_capture_loop(self):
        # return
        cap = cv2.VideoCapture(0)

        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not cap.isOpened():
            print("Unable to connect to camera.")
            return False

        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                self.process_image(frame)


if __name__ == '__main__':
    rospy.init_node("face")
    external_cam = False
    fl = FaceLocalizer(use_subscriber=external_cam, debug=True)
    if external_cam:
        rospy.spin()
