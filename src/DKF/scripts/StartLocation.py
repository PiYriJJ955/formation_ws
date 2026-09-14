#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool
import sys, select, termios, tty
import signal
import os

class LocationController:
    def __init__(self):
        rospy.init_node('location_control', disable_signals=True)  # 禁用ROS内置信号处理
        self.pub = rospy.Publisher('StartLocation', Bool, queue_size=1)
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 绑定信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 确保退出前恢复终端设置
        rospy.on_shutdown(self.cleanup)

    def signal_handler(self, sig, frame):
        """处理系统信号"""
        print("\n接收到退出信号，正在关闭...")
        self.cleanup()  # 手动执行清理
        rospy.signal_shutdown("User interrupt")
        sys.exit(0)

    def get_key(self):
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            return sys.stdin.read(1) if rlist else ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def cleanup(self):
        """资源清理函数"""
        self.pub.publish(Bool(False))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("定位退出")

    def run(self):
        print("就绪（按 L 开始定位，Ctrl+C 退出）...")
        try:
            while not rospy.is_shutdown():
                key = self.get_key().lower()
                if key == 'l':
                    self.pub.publish(Bool(True))
                    print("开始定位")
                rospy.sleep(0.05)
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    controller = LocationController()
    controller.run()