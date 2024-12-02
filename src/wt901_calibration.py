import rclpy
from rclpy.node import Node
import serial

g = 9.80665
PI = 3.14159
timeout = 0.2

def send(ser, command: str, delay=timeout):
    ser.write(bytes.fromhex(command))  # Отправка команды
    rclpy.sleep(delay)

class WT901CalibrationNode(Node):
    def init(self):
        super().init('wt901_calibration_node')
        self.declare_parameter("port", "/dev/ttyUSB0")  # Установка параметра
        self.device = self.get_parameter("port").get_parameter_value().string_value

        try:
            self.ser = serial.Serial(self.device, 115200, timeout=None)
            self.get_logger().info(f"Successfully connected to {self.device}. Waiting for {timeout} sec")
        except Exception as e:
            self.get_logger().error(f"Cannot connect to IMU: {e}")
            exit(1)

        self.run_calibration()

    def run_calibration(self):
        self.get_logger().info("Accelerometer Calibration. Waiting for 5 sec...")
        send(self.ser, 'FF AA 01 01 00', 5)  # Accelerometer Calibration
        send(self.ser, 'FF AA 01 00 00')  # Quit Calibration
        send(self.ser, 'FF AA 03 00 00')  # Save
        self.get_logger().info("Calibration finished")
        send(self.ser, 'FF AA 03 08 00')  # Set to 50 Hz
        self.get_logger().info("Hardware rate has been set to 50 Hz")
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = WT901CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()