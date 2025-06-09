import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from aip import AipSpeech
import speech_recognition as sr

class VocalController(Node):
    def __init__(self):
        super().__init__('vocal_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.APP_ID = '116679617'
        self.API_KEY = 'EUFfWkDgXQjcmjvDVmoMTUEM'
        self.SECRET_KEY = 'jvbEnbGLipCERgLKr22kDLbvMDIRC7ix'
        self.client = AipSpeech(self.APP_ID, self.API_KEY, self.SECRET_KEY)

    def get_text(self,wav_bytes):
        result = self.client.asr(wav_bytes, 'wav', 16000, {'dev_pid': 1537,})
        try:
            text = result['result'][0]
        except Exception as e:
            print(e)
            text = ""
        return text

    def timer_callback(self):
        twist = Twist()
        r = sr.Recognizer()
        mic = sr.Microphone()
        print("请说话...")
        with mic as source:
            r.adjust_for_ambient_noise(source)
            audio = r.listen(source)
        audio_data = audio.get_wav_data(convert_rate=16000)
        print("\n正在分析...")
        text = self.get_text(audio_data)
        text = list(text) 
        for item in text:
            if item == '前':
                twist.linear.x = 0.1
            elif item == '后':
                twist.linear.x = -0.1
            elif item == '左':
                twist.angular.z = 0.5
            elif item == '右':
                twist.angular.z = -0.5   
            elif item == '停':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main():
    rclpy.init()
    vocal_controller = VocalController()
    rclpy.spin(vocal_controller)
    vocal_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()