#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FakeTF(Node):
    def __init__(self):
        super().__init__('fake_tf_pub')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.send_tf)

    def send_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


class LaserTF(Node):
    def __init__(self):
        super().__init__('laser_tf_pub')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.send_tf)

    def send_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


def main():
    rclpy.init()
    fake_tf = FakeTF()
    laser_tf = LaserTF()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(fake_tf)
    executor.add_node(laser_tf)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        #fake_tf.destroy_node()
        laser_tf.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()