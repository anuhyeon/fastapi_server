import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import struct

class PointCloudNode(Node):
    def __init__(self):
        super().__init__('point_cloud_node')

        # 구독자 설정
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self.info_callback, 10
        )

        # 퍼블리셔 설정
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/points', 10)

        # 데이터 저장
        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None
        self.camera_intrinsics = None

        self.get_logger().info("PointCloudNode initialized.")

    def depth_callback(self, msg):
        # 깊이 이미지 수신
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        # 컬러 이미지 수신
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def info_callback(self, msg):
        # 카메라 내부 매트릭스 수신
        self.camera_intrinsics = np.array(msg.k).reshape(3, 3)

    def generate_pointcloud(self):
        if self.depth_image is None or self.color_image is None or self.camera_intrinsics is None:
            return

        height, width = self.depth_image.shape
        fx, fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
        cx, cy = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]

        # 픽셀 좌표 생성
        u = np.tile(np.arange(width), height)
        v = np.repeat(np.arange(height), width)

        # 깊이 값을 가져옴
        z = self.depth_image.flatten() * 0.001  # 깊이 값을 미터 단위로 변환
        valid = z > 0  # 유효한 깊이 값 필터링
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # RGB 값 가져오기
        colors = self.color_image[v[valid], u[valid]]

        # 포인트 클라우드 데이터 생성
        points = np.column_stack((x[valid], y[valid], z[valid], colors))

        # PointCloud2 메시지 생성
        pc_msg = self.create_pointcloud2(points)
        self.pc_pub.publish(pc_msg)
        self.get_logger().info("Published PointCloud2 message.")

    def create_pointcloud2(self, points):
        # PointCloud2 메시지 필드 정의
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
        ]

        # 메시지 생성
        pc_msg = PointCloud2()
        pc_msg.header.frame_id = "camera_depth_optical_frame"
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.is_dense = True
        pc_msg.is_bigendian = False
        pc_msg.fields = fields
        pc_msg.point_step = 16  # x, y, z, r, g, b (총 16바이트)
        pc_msg.row_step = pc_msg.point_step * len(points)

        # 데이터 변환
        pc_data = []
        for point in points:
            pc_data.append(struct.pack('fffBBB', *point))
        pc_msg.data = b''.join(pc_data)

        return pc_msg

    def timer_callback(self):
        # 주기적으로 포인트 클라우드 생성
        self.generate_pointcloud()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()

    # 10Hz로 주기적으로 포인트 클라우드 생성
    timer = node.create_timer(0.1, node.timer_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
