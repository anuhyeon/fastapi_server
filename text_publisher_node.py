import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import time

class TextPublisher(Node):
    def __init__(self):
        super().__init__('text_publisher')
        self.publisher_ = self.create_publisher(String, 'user_text_input', 10)
        self.server_url = "http://localhost:8000/get_text"  # 서버 URL
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 실행

    def timer_callback(self):
        try:
            response = requests.get(self.server_url) # 서버한테 데이터 요청 1초마다
            response.raise_for_status() # 요청 상태 확인 (에러 발생 시 예외 처리)
            text_data = response.json().get("text", "") # JSON 응답에서 "text" 필드 추출
            if text_data:
                self.publish_text(text_data) # 해당 토픽으로 텍스트데이터 퍼블리싱
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to fetch text from server: {e}")

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published text: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    text_publisher = TextPublisher()
    rclpy.spin(text_publisher)
    text_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
