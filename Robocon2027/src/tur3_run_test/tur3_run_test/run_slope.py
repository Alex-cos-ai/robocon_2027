import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion  # 四元数转欧拉角函数
import time
# -0.162
# ros2 run tur3_run_test run_slope
class ImuYawListener(Node):
    def __init__(self):
        super().__init__('imu_yaw_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.pitch = 0
        self.bool_slope = 0
        self.get_logger().info('imu_yaw_listener started, waiting for /imu ...')
        self.start_time = time.time()
        self.out_time_slop = None
        self.start_time_slop = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self.timer1 = self.create_timer(0.1,self.timer1_callback)
        self.timer2 = self.create_timer(0.1,self.timer2_callback)
        self.twist = Twist()
        self.twist.linear.x = 0.5

    def timer2_callback(self):
        if self.bool_slope == 2:
            self.twist.linear.x = 0.0
            time.sleep(1)
            self.publisher.publish(self.twist)
        self.publisher.publish(self.twist)
            


    def timer1_callback(self):
        if self.start_time_slop != None and time.time() - self.start_time_slop > 0.5:
            self.bool_slope = 1
        if self.pitch < -0.15:
            if self.start_time_slop == None:
                self.start_time_slop = time.time()
        if self.bool_slope == 1 and self.pitch > -0.05:
            self.bool_slope = 2
            


    def imu_callback(self, msg):
        q = msg.orientation

        # 四元数 -> 欧拉角 (roll, pitch, yaw)，单位：弧度
        rotation_euler = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w,
        ])

        roll, pitch, yaw = rotation_euler
        self.pitch = pitch

        # self.get_logger().info(f'四元数: x={q.x:.4f}, y={q.y:.4f}, z={q.z:.4f}, w={q.w:.4f}')
        if time.time() - self.start_time > 0.2:
            self.get_logger().info(f'rool={rotation_euler[0]:.3f},pithch={rotation_euler[1]:.3f},yaw={rotation_euler[2]:.3f}')
            self.start_time = time.time()
        # self.get_logger().info(f'Yaw: {yaw:.4f} rad ({yaw * 180.0 / 3.141592653589793:.2f} deg)')


def main():
    rclpy.init()
    node = ImuYawListener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()