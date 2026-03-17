#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import sys

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot System Control Panel")
        self.root.geometry("500x600")
        self.root.configure(bg="#2c3e50")

        # หัวข้อ
        label = tk.Label(root, text="Robot Control Center", font=("Arial", 18, "bold"),
                         bg="#2c3e50", fg="#ecf0f1", pady=20)
        label.pack()

        # สไตล์ปุ่ม
        button_style = {
            "font": ("Arial", 11, "bold"),
            "width": 40,
            "height": 2,
            "bd": 3,
            "relief": "raised"
        }

        # รายการปุ่ม (Label, Command)
        self.commands = [
            ("ESP Sensors", "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"),
            ("ESP Motor", "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200"),
            ("Controller", "ros2 run my_control controller_node"),
            ("Mapping", "ros2 launch my_control launch_rtabmap_mapping.launch.py"),
            ("Localize", "ros2 launch my_control launch_rtabmap_localize.launch.py"),
            ("Navigation", (
                "ros2 launch nav2_bringup navigation_launch.py "
                "use_sim_time:=false "
                "params_file:=/home/noone/Robotics_Project/src/my_control/config/nav2_params.yaml "
                "use_amcl:=false "
                "map:=/rtabmap/map"
            )),
            ("Voice Control", "ros2 launch my_voice_control launch_voice_control.launch.py")
        ]

        for text, cmd in self.commands:
            btn = tk.Button(root, text=text, command=lambda c=cmd: self.run_command(c),
                            bg="#3498db", fg="white", **button_style)
            btn.pack(pady=5)
            btn.bind("<Enter>", lambda e, b=btn: b.configure(bg="#2980b9"))
            btn.bind("<Leave>", lambda e, b=btn: b.configure(bg="#3498db"))

    def run_command(self, cmd):
        try:
            full_command = f'gnome-terminal -- bash -c "{cmd}; exec bash"'
            subprocess.Popen(full_command, shell=True)
        except Exception as e:
            messagebox.showerror("Error", f"ไม่สามารถรันคำสั่งได้: {e}")

# --- ส่วนสำคัญที่ต้องมีเพื่อให้ ros2 run ทำงานได้ ---
def main(args=None):
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()