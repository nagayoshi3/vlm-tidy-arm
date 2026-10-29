#!/usr/bin/env python3
"""購読が届くかを段階的に確かめる診断スクリプト（tidy_node と同じ構成を少しずつ再現）。

  python3 sub_test.py 1   # joint_states / camera を購読するだけ
  python3 sub_test.py 2   # + tf の購読（TransformListener）
  python3 sub_test.py 3   # + 2 Hz でパネル画像を送信（tidy_node と同じ）
各段階で 5 秒ごとに受信回数を表示する。Ctrl+C で終了。
"""
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

stage = int(sys.argv[1]) if len(sys.argv) > 1 else 1


class T(Node):
    def __init__(self):
        super().__init__('sub_test')
        self.n_js = self.n_img = self.n_tf = 0
        self.create_subscription(JointState, '/joint_states', self.js, 20)
        self.create_subscription(Image, '/tidy/camera/image', self.img, 2)
        if stage >= 2:
            from tf2_ros import Buffer, TransformListener
            self.buf = Buffer()
            self.tl = TransformListener(self.buf, self)
        if stage >= 3:
            self.pub = self.create_publisher(Image, '/tidy/panel_test', 2)
            threading.Thread(target=self.pub_loop, daemon=True).start()

    def js(self, m): self.n_js += 1
    def img(self, m): self.n_img += 1

    def pub_loop(self):
        import array
        while rclpy.ok():
            m = Image(); m.height, m.width, m.encoding, m.step = 600, 800, 'rgb8', 2400
            m.data = array.array('B', bytes(600 * 2400))
            self.pub.publish(m)
            time.sleep(0.5)


def main():
    rclpy.init()
    n = T()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(f'stage {stage}: 購読テスト開始（Ctrl+C で終了）')
    try:
        while True:
            time.sleep(5)
            tf = ''
            if stage >= 2:
                try:
                    n.buf.lookup_transform('crane_plus_base', 'crane_plus_link_endtip', rclpy.time.Time())
                    tf = ', tf OK'
                except Exception as e:
                    tf = f', tf NG ({type(e).__name__})'
            print(f'  joint_states {n.n_js} 回, camera {n.n_img} 回{tf}', flush=True)
    except KeyboardInterrupt:
        pass
    import os
    os._exit(0)


if __name__ == '__main__':
    main()
