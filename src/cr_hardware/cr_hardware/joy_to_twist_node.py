#!/usr/bin/env python3

# ros2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool, Trigger

# text gui
import curses
import threading
import signal
import sys

xbox_btn_names = ('a', 'b', 'y', 'x', 'lb', 'rb', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')

class JoyToTwistNode(Node):
    def __init__(self, stdscr=None):
        super().__init__('joy_to_twist_node')

        # Constants for button and axis names mapped to index
        self.btn_names = ('a', 'b', 'x', 'y', 'lb', 'rb', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')
        self.axes_names = ('left_x', 'left_y', 'lt', 'right_x', 'right_y', 'rt', 'dpad_x', 'dpad_y')
        self.prev_buttons = tuple([0] * 20)
        self.prev_dpad = tuple([0] * 10)

        # ros subscribers, publishers, service callers
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, '/differential_drive_controller/cmd_vel', 1)
        self.lower_mechanism_client = self.create_client(SetBool, 'lower_probe')
        self.lower_mechanism_future = None # for the service call

        # for data collection from ml script
        self.data_collection_client = self.create_client(Trigger, 'start_data_collection')
        self.data_collection_future = None
        
        # Parameters for scaling joystick input to twist output (m/s)
        self.declare_parameter('linear_scale', 0.16)
        self.declare_parameter('angular_scale', 0.16)
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        # Initialize curses
        # self.stdscr = stdscr
        # curses.curs_set(0)
        # self.stdscr.nodelay(True)
        # # Start curses display thread
        # self.curses_thread = threading.Thread(target=self.display, daemon=True)
        # self.curses_thread.start()

    
    def joy_callback(self, msg):
        # ========== Mapping buttons and axes to pressed, started, and released ==========
        dpad = dict(zip(self.axes_names[6:], msg.axes[6:]))
        pressed = dict(zip(self.btn_names, msg.buttons))
        started = dict(zip(self.btn_names, (btn[0] == 1 and btn[1] == 0 for btn in zip(msg.buttons, self.prev_buttons))))
        released = dict(zip(self.btn_names, (btn[0] == 0 and btn[1] == 1 for btn in zip(msg.buttons, self.prev_buttons))))
        axes = dict(zip(self.axes_names, msg.axes))
        self.axes_temp = dict(zip(self.axes_names, msg.axes))
        dpad_started = dict(zip(self.axes_names[6:], (dpad_temp[0] != 0 and dpad_temp[1] == 0 for dpad_temp in zip(msg.axes[6:], self.prev_dpad))))
        dpad_released = dict(zip(self.axes_names[6:], (dpad_temp[0] == 0 and dpad_temp[1] != 0 for dpad_temp in zip(msg.axes[6:], self.prev_dpad))))

        # ========== Speed controls for linear and angular velocity ==========
        if dpad_started['dpad_y']:
            if dpad['dpad_y'] > 0:
                self.linear_scale = self.linear_scale + 0.01
            else:
                self.linear_scale = self.linear_scale - 0.01
        if dpad_started['dpad_x']:
            if dpad['dpad_x'] < 0:
                self.angular_scale = self.angular_scale + 0.01
            else:
                self.angular_scale = self.angular_scale - 0.01
        if started['b'] and pressed['lb'] and pressed['rb']:
            # start data point collection
            self.try_send_data_collection_call(Trigger.Request())
        if axes['right_y'] > 0.5:
            # raise probe, start new thread for service call and print results
            self.try_queue_lower_mechanism_call(SetBool.Request(data=False))
        elif axes['right_y'] < -0.5:
            # lower probe, start new thread for service call and print results
            self.try_queue_lower_mechanism_call(SetBool.Request(data=True))
        
        # ========= Publish twist message =========
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        if pressed['x']:
            twist.twist.linear.x = msg.axes[1] * self.linear_scale  # Assuming axes[1] is for linear movement
            twist.twist.angular.z = msg.axes[0] * self.angular_scale  # Assuming axes[0] is for angular movement
            self.publisher.publish(twist)
        else:
            self.publisher.publish(twist)
            

        # ========= Update previous values =========
        self.prev_dpad = msg.axes[6:]
        self.prev_buttons = msg.buttons

    def try_queue_lower_mechanism_call(self, request: SetBool.Request):
        if self.lower_mechanism_future is None or self.lower_mechanism_future.done():
            self.lower_mechanism_future = self.lower_mechanism_client.call_async(request)
            self.lower_mechanism_future.add_done_callback(self.handle_lower_mechanism_response)

    def try_send_data_collection_call(self, request: Trigger.Request):
        # need to be very careful about calling the data collection button
        if self.data_collection_future is None or self.data_collection_future.done():
            self.data_collection_future = self.data_collection_client.call_async(request)
            self.data_collection_future.add_done_callback(self.handle_data_collection_response)

    def handle_lower_mechanism_response(self, future):
        response = future.result()
        self.get_logger().info(f"Lower mechanism response: {response.message}")
        self.get_logger().info(f"    Message: {response.message}")
        self.get_logger().info(f"    Success: {response.success}")
    
    def handle_data_collection_response(self, future):
        response = future.result()
        self.get_logger().info(f"Data collection response: {response.message}")
        self.get_logger().info(f"    Message: {response.message}")
        self.get_logger().info(f"    Success: {response}")
    # do not call this function, should run in thread
    # def display(self):
    #     while True:
    #         key = self.stdscr.getch()
    #         if key == ord('q'):
    #             rclpy.shutdown()
    #         self.stdscr.clear()
    #         self.stdscr.addstr(0, 0, "Press Ctrl+C to exit")
    #         self.stdscr.addstr(2, 0, f"Linear speed: {self.linear_scale:.2f} m/s")
    #         self.stdscr.addstr(3, 0, f"Angular speed: {self.angular_scale:.2f} m/s")
    #         self.stdscr.refresh()
    #         curses.napms(100)

def main(args=None):
    rclpy.init(args=args)

    node = JoyToTwistNode()
    def signal_handler(sig, frame):
        rclpy.shutdown()
        node.destroy_node()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

    # set up sigint callback before creating node and running curses
    # def run_curses(stdscr):
    #     node = JoyToTwistNode(stdscr)
    #     def signal_handler(sig, frame):
    #         rclpy.shutdown()
    #         node.destroy_node()
    #         sys.exit(0)
    #     signal.signal(signal.SIGINT, signal_handler)
    #     try:
    #         rclpy.spin(node)
    #     finally:
    #         node.destroy_node()
    # try:
    #     curses.wrapper(run_curses)
    # except Exception as e:
    #     curses.endwin()
    #     print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()
