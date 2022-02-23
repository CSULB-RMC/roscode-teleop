import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from std_msgs.msg import Byte

from sensor_msgs.msg import Joy

class JoyToTeensyPublisher(Node):
    def __init__(self):
        super().__init__('joyToTeensyPublisher')
        self.dt_left_pub = self.create_publisher(Byte, 'dt_left', 1)
        self.dt_right_pub = self.create_publisher(Byte, 'dt_right', 1)

        self.dumper_up_pub = self.create_publisher(Empty, 'dumper_up', 10)
        self.dumper_down_pub = self.create_publisher(Empty, 'dumper_down', 10)
        self.stop_all_pub = self.create_publisher(Empty, 'stop_all_arduino', 10)

        self.test_led_pub = self.create_publisher(Empty, 'test_led', 10)

        self.auger_up_pub = self.create_publisher(Empty, 'auger_up', 10)
        self.auger_down_pub = self.create_publisher(Empty, 'auger_down', 10)

        self.auger_dig_pub = self.create_publisher(Empty, 'auger_dig', 10)
        self.auger_dig_rev_pub = self.create_publisher(Empty, 'auger_dig_reverse', 10)

        self.telescope_forward_pub = self.create_publisher(Empty, 'telescope_forward', 10)
        self.telescope_backward_pub = self.create_publisher(Empty, 'telescope_backward', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joy_sub #supress unused var warning
    
    def joy_callback(self, msg):
        self.get_logger().info("JOY CALLBACK! %s" % msg)

        # Y/A button 
        if msg.buttons[2] == 1: #Y - dumper up
            self.dumper_up_pub.publish(Empty())
        if msg.buttons[0] == 1: #A - dumper down
            self.dumper_down_pub.publish(Empty())

        if msg.buttons[8] == 1: #back button is emergency stop everything
            self.stop_all_pub.publish(Empty())

        if msg.buttons[9] == 1: #start button - toggle led
            self.test_led_pub.publish(Empty())


        curbed_left_b = Byte()
        curbed = int(64+(msg.axes[1]*64))
        if curbed > 127:
            curbed = 127
        curbed_left_b.data = curbed.to_bytes(1, 'little')
        self.dt_left_pub.publish(curbed_left_b)


        curbed_right_b = Byte()
        curbed = int(64+(msg.axes[3]*64))
        if curbed > 127:
            curbed = 127
        curbed_right_b.data = curbed.to_bytes(1, 'little')
        self.dt_right_pub.publish(curbed_right_b)

        if msg.buttons[1] == 1:
            self.auger_up_pub.publish(Empty())

        if msg.buttons[3] == 1:
            self.auger_down_pub.publish(Empty())

        if msg.buttons[4] == 1:
            self.telescope_forward_pub.publish(Empty())

        if msg.buttons[5] == 1:
            self.telescope_backward_pub.publish(Empty())

        if msg.buttons[6] == 1:
            self.auger_dig_pub.publish(Empty())

        if msg.buttons[7] == 1:
            self.auger_dig_rev_pub.publish(Empty())

def main(args=None):
    rclpy.init(args=args)

    joytoteensypublisher = JoyToTeensyPublisher()

    rclpy.spin(joytoteensypublisher)

    joytoteensypublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
