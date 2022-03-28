import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from std_msgs.msg import Int8

from sensor_msgs.msg import Joy

class JoyToTeensyPublisher(Node):
    def __init__(self):
        super().__init__('joyToTeensyPublisher')
        self.dt_left_pub = self.create_publisher(Int8, 'dt_left', 1)
        self.dt_right_pub = self.create_publisher(Int8, 'dt_right', 1)

        self.dumper_pub = self.create_publisher(Int8, 'dumper_control', 10)
        self.bucketladder_lifter_pub = self.create_publisher(Int8, 'bucketladder_lifter_control', 10)
        self.bucketladder_telescope_pub = self.create_publisher(Int8, 'bucketladder_telescope_control', 10)
        self.bucketladder_digger_pub = self.create_publisher(Int8, 'bucketladder_digger_control', 10)

        self.stop_all_pub = self.create_publisher(Empty, 'stop_all_arduino', 10)

        self.test_led_pub = self.create_publisher(Empty, 'test_led', 10)
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joy_sub #supress unused var warning
    
    def joy_callback(self, msg):
        self.get_logger().info("JOY CALLBACK! %s" % msg)

        # Y/A button 
        dp_msg = Int8()
        if msg.buttons[3] == 1: #Y - dumper up   
            dp_msg.data = 1 # dumper up
        if msg.buttons[0] == 1: #A - dumper down
            dp_msg.data = 0 # dumper down
        if msg.buttons[0] == 0 and msg.buttons[3] == 0: #if not A or Y
            dp_msg.data = 3
        if msg.buttons[0] == 1 and msg.buttons[3] == 1: #if A and Y
            dp_msg.data = 3
        self.dumper_pub.publish(dp_msg)

        dp_sift_msg = Int8()
        if msg.buttons[5] == 1: #right bumper
            dp_sift_msg.data = 4 #sift on
        elif msg.buttons[5] == 0:
            dp_sift_msg.data = 5 #sift off
        self.dumper_pub.publish(dp_sift_msg)

        bl_msg = Int8()
        if msg.axes[5] < 0: #right trigger
            bl_msg.data = 1 #forward
        if msg.axes[2] < 0: #left trigger
            bl_msg.data = 0 #backward
        if msg.axes[5] > 0 and msg.axes[2] > 0:
            bl_msg.data = 3 #stop
        if msg.axes[5] < 0 and msg.axes[2] < 0:
            bl_msg.data = 3 #also stop
        self.bucketladder_digger_pub.publish(bl_msg)


        bl_lift_msg = Int8()
        if msg.buttons[2] == 1: # X button
            bl_lift_msg.data = 1 #move up
        if msg.buttons[1] == 1: # B button
            bl_lift_msg.data = 0 #move down
        if msg.buttons[2] == 0 and msg.buttons[1] == 0:
            bl_lift_msg.data = 3 #stop
        if msg.buttons[2] == 1 and msg.buttons[1] == 1:
            bl_lift_msg.data = 3
        self.bucketladder_lifter_pub.publish(bl_lift_msg)

        bl_tele_msg = Int8()
        if msg.axes[7] < 0: #dpad up/down
            bl_tele_msg.data = 1 #move forward
        elif msg.axes[7] > 0:
            bl_tele_msg.data = 0 #move backward
        else:
            bl_tele_msg.data = 3 #stop
        self.bucketladder_telescope_pub.publish(bl_tele_msg)


        if msg.buttons[6] == 1: #back button is emergency stop everything
            self.stop_all_pub.publish(Empty())

        if msg.buttons[7] == 1: #start button - toggle led
            self.test_led_pub.publish(Empty())


        curbed_left_b = Int8() #left drivetrain controlled by left thumbstick forward/back
        curbed_left_b.data = int(msg.axes[1]*100) #make the axes value from -100 to 100
        if curbed_left_b.data > 100:
            curbed_left_b.data = 100
        if curbed_left_b.data < -100:
            curbed_left_b.data = -100
        self.dt_left_pub.publish(curbed_left_b)


        curbed_right_b = Int8() #right drivetrain controlled by right thumbstick forward/back
        curbed_right_b.data = int(msg.axes[4]*100) #ditto as above
        if curbed_right_b.data > 100:
            curbed_right_b.data = 100
        if curbed_right_b.data < -100:
            curbed_right_b.data = -100
        self.dt_right_pub.publish(curbed_right_b)




def main(args=None):
    rclpy.init(args=args)

    joytoteensypublisher = JoyToTeensyPublisher()

    rclpy.spin(joytoteensypublisher)

    joytoteensypublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
