#!/usr/bin/env python3
"""
极简键控 UI：WS 加速/减速（含后退），A/D 转向，空格立即停车；按钮与键盘等效。
启动前请确保已 source ROS2 环境、仿真运行且桥接了 /cmd_vel。
"""

import tkinter as tk
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


BG = "#1f2335"
PANEL = "#2b3050"
ACCENT = "#5ac8fa"
TEXT = "#e5e7ef"


class TeleopUI:
    def __init__(self, master):
        self.master = master
        master.title("TIAGo 仿真控制")
        master.geometry("360x360")
        master.configure(bg=BG)

        self.node = None
        self.pub_cmd = None
        self.pub_mobile = None
        self.pub_model = None
        self._repeat_job = None
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.MAX_LIN = 1.2
        self.MIN_LIN = -0.6
        self.ANG_SPEED = 1.2
        self.LIN_STEP = 0.1

        title = tk.Label(master, text="TIAGo Teleop", fg=TEXT, bg=BG,
                         font=("Helvetica", 16, "bold"))
        title.pack(pady=(10, 6))

        info = tk.Label(master, text="W 加速 / S 减速或后退 / A 左转 / D 右转 / 空格停车", fg="#bfc4d9",
                        bg=BG, font=("Helvetica", 10))
        info.pack()

        btn_frame = tk.Frame(master, bg=BG)
        btn_frame.pack(pady=12)

        self.make_btn(btn_frame, "W", lambda: self.bump_lin(1), 0, 1)
        self.make_btn(btn_frame, "A", lambda: self.turn_left(), 1, 0)
        self.make_btn(btn_frame, "停", self.stop, 1, 1)
        self.make_btn(btn_frame, "D", lambda: self.turn_right(), 1, 2)
        self.make_btn(btn_frame, "S", lambda: self.bump_lin(-1), 2, 1)

        self.status = tk.Label(master, text="等待 ROS2 初始化...", fg=ACCENT, bg=BG,
                               font=("Helvetica", 11))
        self.status.pack(pady=4)

        self.status_vel = tk.Label(master, text="线 0.00 m/s | 角 0.00 rad/s",
                                   fg="#a8acc4", bg=BG, font=("Helvetica", 10))
        self.status_vel.pack(pady=2)

        # 键盘绑定：W/S 调节线速度，A/D 转向，空格停
        master.bind("<KeyPress-w>", lambda e: self.bump_lin(1))
        master.bind("<KeyPress-s>", lambda e: self.bump_lin(-1))
        master.bind("<KeyPress-a>", lambda e: self.turn_left())
        master.bind("<KeyPress-d>", lambda e: self.turn_right())
        master.bind("<KeyRelease-a>", lambda e: self.clear_turn())
        master.bind("<KeyRelease-d>", lambda e: self.clear_turn())
        master.bind("<space>", lambda e: self.stop())

        master.protocol("WM_DELETE_WINDOW", self.on_close)

        self.init_ros()
        self.start_loop()

    def make_btn(self, frame, text, action, r, c):
        btn = tk.Button(frame, text=text, width=6, height=2, bg=PANEL, fg=TEXT,
                        activebackground=ACCENT, activeforeground=BG,
                        relief=tk.FLAT, font=("Helvetica", 12, "bold"),
                        command=action)
        btn.grid(row=r, column=c, padx=6, pady=6)

    def init_ros(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node("tiago_teleop_ui")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.pub_cmd = self.node.create_publisher(Twist, "/cmd_vel", qos)
        self.pub_mobile = self.node.create_publisher(
            Twist, "/mobile_base_controller/cmd_vel_unstamped", qos)
        self.pub_model = self.node.create_publisher(
            Twist, "/model/tiago/cmd_vel", qos)
        self.status.config(text="已连接 /cmd_vel, /mobile_base_controller/cmd_vel_unstamped, /model/tiago/cmd_vel")

    def send_once(self, lin_dir=None, ang_dir=None):
        if lin_dir is not None:
            self.lin_vel = lin_dir
        if ang_dir is not None:
            self.ang_vel = ang_dir
        msg = Twist()
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        if self.pub_cmd is not None:
            self.pub_cmd.publish(msg)
        if self.pub_mobile is not None:
            self.pub_mobile.publish(msg)
        if self.pub_model is not None:
            self.pub_model.publish(msg)
        self.status_vel.config(
            text=f"线 {msg.linear.x:.2f} m/s | 角 {msg.angular.z:.2f} rad/s")

    def bump_lin(self, direction):
        self.lin_vel += direction * self.LIN_STEP
        self.lin_vel = max(self.MIN_LIN, min(self.MAX_LIN, self.lin_vel))
        self.send_once()

    def turn_left(self):
        self.ang_vel = self.ANG_SPEED
        self.send_once()

    def turn_right(self):
        self.ang_vel = -self.ANG_SPEED
        self.send_once()

    def clear_turn(self):
        self.ang_vel = 0.0
        self.send_once()

    def start_loop(self):
        self.send_once()
        self._repeat_job = self.master.after(100, self.start_loop)

    def stop(self):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        msg = Twist()
        if self.pub_cmd is not None:
            self.pub_cmd.publish(msg)
        if self.pub_mobile is not None:
            self.pub_mobile.publish(msg)
        if self.pub_model is not None:
            self.pub_model.publish(msg)
        self.status_vel.config(text="线 0.00 m/s | 角 0.00 rad/s")

    def on_close(self):
        try:
            self.stop()
        except Exception:
            pass
        if self._repeat_job:
            self.master.after_cancel(self._repeat_job)
        if self.node is not None:
            self.node.destroy_node()
        rclpy.shutdown()
        self.master.destroy()


def main():
    root = tk.Tk()
    app = TeleopUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
