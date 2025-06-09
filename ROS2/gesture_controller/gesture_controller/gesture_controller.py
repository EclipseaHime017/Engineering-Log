import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import cv2
import mediapipe as mp

class Gesturecontroller(Node):

    def __init__(self):
        super().__init__('gesture_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)
        self.frame_count = 0
        self.process_frame_interval = 3
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.model_path = '/home/chlorine/ros2_ws/src/gesture_controller/model/gesture_recognizer.task'
        self.options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=self.model_path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO
        )
        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(self.options)
        
    def timer_callback(self):
        ret, img = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        if self.frame_count % self.process_frame_interval != 0:
            return

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
        recognition_result = self.recognizer.recognize_for_video(mp_image, self.frame_count)

        if recognition_result and recognition_result.gestures:
            t = recognition_result.gestures[0][0].category_name
        else:
            t = "none"
        print(t)

        twist = Twist()
        if t == 'Open_Palm':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif t == 'Pointing_Up':
            twist.linear.x = 0.1
            twist.angular.z = 0.0
        elif t == 'Victory':
            twist.linear.x = -0.1
            twist.angular.z = 0.0
        elif t == 'Thumb_Up':
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        elif t == 'Thumb_Down':
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        else:
            pass
        self.publisher_.publish(twist)

        cv2.putText(img, t, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('img', img)
        cv2.waitKey(10)
    	
def main(args=None):
    rclpy.init(args=args)

    gesture_controller = Gesturecontroller()

    rclpy.spin(gesture_controller)

    gesture_controller.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()    	