#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(Int32, 'bazylkhanov_kostyukova_saparova_kenzhebek', 10)
        
        self.first_student_digits = [2, 0, 1, 8, 4, 1, 0, 5, 0]
        self.second_student_name = [2, 0, 1, 8, 9, 7, 1, 4, 9] 
        self.third_student_digits = [2, 0, 1, 8, 4, 5, 4, 1, 5]
        self.fourth_student_digits = [2, 0, 1, 9, 5, 8, 1, 9, 7]
        
        self.sequence = [
            (self.first_student_digits, 1),  # 1 Hz
            (self.second_student_name, 50),  # 50 Hz
            (self.third_student_digits, 1),  # 1 Hz
            (self.fourth_student_digits, 50)  # 50 Hz
        ]
        
        self.seq_index = 0  
        self.index = 0  
        self.rate = self.sequence[self.seq_index][1]  
        
        self.publish_digit(self.rate)

    def publish_digit(self, rate):
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.seq_index >= len(self.sequence):
            self.get_logger().info('All data published. Stopping.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()
            return
        
        msg = Int32()
        msg.data = self.sequence[self.seq_index][0][self.index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing at {self.rate} Hz: {msg.data}')
        self.index += 1

        if self.index >= len(self.sequence[self.seq_index][0]):
            self.index = 0
            self.seq_index += 1

            if self.seq_index < len(self.sequence):
                self.rate = self.sequence[self.seq_index][1]
                self.get_logger().info(f'Switching to {self.rate} Hz')
                self.timer.cancel()
                self.publish_digit(self.rate)


def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)


if __name__ == '__main__':
    main()
