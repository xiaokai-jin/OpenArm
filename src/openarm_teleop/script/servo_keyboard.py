#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty
import threading

msg = """
双臂笛卡尔空间连续控制 (MoveIt Servo)
---------------------------
切换控制手臂:
  Tab: 切换 左臂/右臂

平移 (相对于World坐标系):
  w/s : +X / -X (前后)
  a/d : 左右运动
  q/e : 上下运动

旋转:
  u/j : +Roll / -Roll
  i/k : +Pitch / -Pitch
  o/l : +Yaw / -Yaw

空格键(Space) / x : 停止运动
Ctrl-C : 退出
"""

bindings = {
    'w':(1,0,0,0,0,0), 's':(-1,0,0,0,0,0),
    'a':(0,0,1,0,0,0), 'd':(0,0,-1,0,0,0),
    'q':(0,1,0,0,0,0), 'e':(0,-1,0,0,0,0),
    'u':(0,0,0,1,0,0), 'j':(0,0,0,-1,0,0),
    'i':(0,0,0,0,1,0), 'k':(0,0,0,0,-1,0),
    'o':(0,0,0,0,0,1), 'l':(0,0,0,0,0,-1),
}

class KeyboardServoTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_servo_teleop')
        self.pub_left = self.create_publisher(TwistStamped, '/servo_node_left/delta_twist_cmds', 10)
        self.pub_right = self.create_publisher(TwistStamped, '/servo_node_right/delta_twist_cmds', 10)
        self.active_arm = 'left' # 默认控制左臂
        self.speed = 0.1         # 线速度缩放 m/s
        self.turn = 0.5          # 角速度缩放 rad/s
        self.timer = self.create_timer(0.01, self.timer_callback) # 提高发布频率至100Hz
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 尝试激活伺服
        from std_srvs.srv import Trigger
        self.client_left = self.create_client(Trigger, '/servo_node_left/start_servo')
        self.client_right = self.create_client(Trigger, '/servo_node_right/start_servo')
        
        self.get_logger().info("尝试激活伺服控制器...")
        if self.client_left.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            self.client_left.call_async(req)
        if self.client_right.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            self.client_right.call_async(req)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer_callback(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'world'
        twist.twist.linear.x = self.cmd[0] * self.speed
        twist.twist.linear.y = self.cmd[1] * self.speed
        twist.twist.linear.z = self.cmd[2] * self.speed
        twist.twist.angular.x = self.cmd[3] * self.turn
        twist.twist.angular.y = self.cmd[4] * self.turn
        twist.twist.angular.z = self.cmd[5] * self.turn

        if self.active_arm == 'left':
            self.pub_left.publish(twist)
        else:
            self.pub_right.publish(twist)

    def run(self):
        print(msg)
        print(f"当前激活机械臂: {self.active_arm}")
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in bindings:
                    self.cmd = bindings[key]
                elif key == '\t':
                    self.active_arm = 'right' if self.active_arm == 'left' else 'left'
                    print(f"\r\n当前激活机械臂: {self.active_arm}")
                    self.cmd = [0.0]*6
                elif key == '\x03': # Ctrl-C
                    break
                else:
                    # 如果超时(没按键)或按了不认识的键，停下！
                    self.cmd = [0.0]*6
        finally:
            self.cmd = [0.0]*6
            self.timer_callback()

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardServoTeleop()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
