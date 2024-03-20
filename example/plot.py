# import matplotlib.pyplot as plt

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

topic_name = '/tracker/bounding_boxes_3d'
root_frame = 'base_frame'
camera_frame = 'camera_link'

class Plotter(Node):
    def __init__(self):
        super().__init__('plotter')
        self.subscription = self.create_subscription(Detection3DArray, topic_name, self.listener_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def offset_callback(self, x, y, offset, scale=50):
        return (int(x * scale + offset[0]), int(y * scale + offset[1]))

    def get_frame(self, root_frame, target_frame, flip=False) -> tf2_ros.TransformStamped:
        try:
            if (flip):
                return self.tf_buffer.lookup_transform(target_frame, root_frame, rclpy.time.Time())
            return self.tf_buffer.lookup_transform(root_frame, target_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def listener_callback(self, msg):
        x, y, z = [], [], []

        for detection in msg.detections:
            x.append(detection.bbox.center.position.x)
            y.append(detection.bbox.center.position.y)
            z.append(detection.bbox.center.position.z)

        tf = self.get_frame(camera_frame, root_frame, False)
        if tf:
            q = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
            yaw = np.arctan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])) + np.pi / 2 # -90 degree
            pitch = np.arcsin(2.0 * (q[3] * q[1] - q[2] * q[0]))
            roll = np.arctan2(2.0 * (q[3] * q[0] + q[1] * q[2]), 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]))
        else:
            return

        width = 500
        window_img = np.zeros((width, width, 3), dtype=np.uint8)
        cv2.circle(window_img, (width//2, width//2), 5, (0, 255, 0), -1)
        arrow_length = width//2
        cv2.arrowedLine(window_img, (width//2, width//2), (int(arrow_length - 100 * np.cos(yaw)), int(arrow_length - 100 * np.sin(yaw))), (0, 255, 0), 2)

        scale = 100
        for i in range(len(x)):
            _x, _y =  x[i]* scale, y[i]* scale

            _y = _x * np.cos(yaw) - _y * np.sin(yaw)
            _x = _x * np.sin(yaw) + _y * np.cos(yaw)

            cv2.circle(window_img, (int(_x) + width//2, int(_y) + width//2), 10, (0, 0, 255), -1)

        cv2.imshow('image', window_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    plotter = Plotter()

    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
