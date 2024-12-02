import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from tf_transformations import quaternion_from_euler
import serial

class WT901Node(Node):
    def init(self):
        super().init('wt901_node')

        # Объявление параметров
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("orientation_covariance", [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("linear_acceleration_covariance", [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("angular_velocity_covariance", [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Получение параметров
        self.device = self.get_parameter("port").get_parameter_value().string_value
        self.orientation_covariance = self.get_parameter("orientation_covariance").get_parameter_value().double_array_value
        self.linear_acceleration_covariance = self.get_parameter("linear_acceleration_covariance").get_parameter_value().double_array_value
        self.angular_velocity_covariance = self.get_parameter("angular_velocity_covariance").get_parameter_value().double_array_value

        self.g = 9.80665
        self.PI = 3.14159

        # Инициализация serial
        try:
            self.ser = serial.Serial(self.device, 115200, timeout=None)
            self.get_logger().info(f"Connected to IMU at {self.device}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to IMU: {e}")
            exit(1)

        # Создание публикации
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.05, self.update)  # 50 Гц

    def update(self):
        try:
            data = self.ser.read(2)
            if data[1] == bytes.fromhex('61')[0]:
                data = self.ser.read(18)

                axL, axH, ayL, ayH, azL, azH, wxL, wxH, wyL, wyH, wzL, wzH, RollL, RollH, PitchL, PitchH, YawL, YawH = data[:]

                ax = int.from_bytes([axH, axL], byteorder="big", signed=True) / 32768 * 16 * self.g
                ay = int.from_bytes([ayH, ayL], byteorder="big", signed=True) / 32768 * 16 * self.g
                az = int.from_bytes([azH, azL], byteorder="big", signed=True) / 32768 * 16 * self.g

                wx = int.from_bytes([wxH, wxL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI
                wy = int.from_bytes([wyH, wyL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI
                wz = int.from_bytes([wzH, wzL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI

                Roll = int.from_bytes([RollH, RollL], byteorder="big", signed=True) / 32768 * self.PI
                Pitch = int.from_bytes([PitchH, PitchL], byteorder="big", signed=True) / 32768 * self.PI
                Yaw = int.from_bytes([YawH, YawL], byteorder="big", signed=True) / 32768 * self.PI

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu"

                imu_msg.orientation = Quaternion(*quaternion_from_euler(Roll, -Pitch, -Yaw))
                imu_msg.orientation_covariance = self.orientation_covariance

                imu_msg.angular_velocity = Vector3(wx, -wy, -wz)
                imu_msg.angular_velocity_covariance = self.angular_velocity_covariance

                imu_msg.linear_acceleration = Vector3(ax, -ay, -az - 2 * self.g)
                imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance

                self.publisher_imu.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WT901Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.ser.close()
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == 'main':
    main()
