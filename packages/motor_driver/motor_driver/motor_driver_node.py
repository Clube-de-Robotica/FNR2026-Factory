import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
import serial
import time

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Serial connection to Arduino
        self.serial_port = '/dev/ttyACM0'  # Update this to your Arduino's serial port
        self.baud_rate = 115200
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        time.sleep(2)  # Wait for serial connection to initialize

        # Subscribers for motor commands
        self.cmd_subscribers = []
        for i in range(4):
            topic = f'/motor{i}/cmd'
            sub = self.create_subscription(
                Int16,
                topic,
                lambda msg, i=i: self.motor_cmd_callback(msg, i),
                10
            )
            self.cmd_subscribers.append(sub)

        # Publishers for encoder values
        self.encoder_publishers = []
        for i in range(4):
            topic = f'/motor{i}/encoder'
            pub = self.create_publisher(Int16, topic, 10)
            self.encoder_publishers.append(pub)

        # Timer for publishing encoder values
        self.timer = self.create_timer(0.1, self.publish_encoders)

        self.get_logger().info("Motor driver node initialized.")

    def motor_cmd_callback(self, msg, motor_id):
        speed = msg.data
        cmd = f"M{motor_id} {speed}\n".encode('utf-8')
        self.serial_connection.write(cmd)
        self.get_logger().info(f"Set motor {motor_id} speed to {speed}")

    def publish_encoders(self):
        for i in range(4):
            self.serial_connection.write(f"E{i}\n".encode('utf-8'))
            time.sleep(0.01)  # Small delay for serial response
            response = self.serial_connection.readline().decode('utf-8').strip()
            if response.startswith(f"ENCODER{i}:"):
                encoder_value = int(response.split(":")[1].strip())
                msg = Int16()
                msg.data = encoder_value
                self.encoder_publishers[i].publish(msg)
                self.get_logger().debug(f"Published encoder {i} value: {encoder_value}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
