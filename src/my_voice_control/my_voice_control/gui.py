#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import tkinter as tk
from tkinter import font
import threading  # <--- เพิ่ม Library นี้

class VoiceGuiNode(Node):
    def __init__(self, gui_app):
        super().__init__('voice_gui_node')
        self.gui_app = gui_app
        
        self.create_subscription(String, '/voice_cmd', self.cmd_callback, 10)
        self.create_subscription(String, '/robot_feedback', self.feedback_callback, 10)

    def cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # --- THREAD SAFETY FIX ---
            # เราอยู่ใน Thread ของ ROS จะแก้ GUI ตรงๆ ไม่ได้
            # ต้องฝากงานไปทำที่ Main Thread ผ่าน root.after
            self.gui_app.root.after(0, lambda: self.gui_app.update_user_ui(
                data.get('raw_text', ''),
                data.get('action', '-'),
                data.get('room', '-')
            ))
        except Exception as e:
            self.get_logger().error(f"JSON Error: {e}")

    def feedback_callback(self, msg):
        # --- THREAD SAFETY FIX ---
        self.gui_app.root.after(0, lambda: self.gui_app.update_robot_ui(msg.data))


class VoiceApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Voice Control")
        self.root.geometry("600x500")
        self.root.configure(bg="#2C3E50")

        # --- FONT ---
        self.header_font = font.Font(family="Helvetica", size=16, weight="bold")
        self.text_font = font.Font(family="Helvetica", size=12)
        self.highlight_font = font.Font(family="Helvetica", size=14, weight="bold")

        # --- UI ELEMENTS ---
        frame_user = tk.Frame(root, bg="#ECF0F1", bd=2, relief="groove")
        frame_user.pack(pady=20, padx=20, fill="x")

        tk.Label(frame_user, text="👤 User Said:", bg="#ECF0F1", fg="#7F8C8D", font=self.header_font).pack(anchor="w", padx=10, pady=5)
        
        self.lbl_user_text = tk.Label(frame_user, text="...", bg="#ECF0F1", fg="#2C3E50", font=self.highlight_font, wraplength=550, justify="left")
        self.lbl_user_text.pack(anchor="w", padx=20, pady=5)

        frame_details = tk.Frame(frame_user, bg="#ECF0F1")
        frame_details.pack(fill="x", padx=20, pady=10)
        
        self.lbl_action = tk.Label(frame_details, text="Action: -", bg="#BDC3C7", width=20, font=self.text_font)
        self.lbl_action.pack(side="left", padx=5)
        
        self.lbl_room = tk.Label(frame_details, text="Room: -", bg="#BDC3C7", width=20, font=self.text_font)
        self.lbl_room.pack(side="left", padx=5)

        frame_robot = tk.Frame(root, bg="#3498DB", bd=2, relief="groove")
        frame_robot.pack(pady=20, padx=20, fill="both", expand=True)

        tk.Label(frame_robot, text="🤖 Robot Response:", bg="#3498DB", fg="white", font=self.header_font).pack(anchor="w", padx=10, pady=5)
        
        self.lbl_robot_text = tk.Label(frame_robot, text="Waiting...", bg="#3498DB", fg="white", font=("Helvetica", 18, "bold"), wraplength=550)
        self.lbl_robot_text.pack(expand=True)

        tk.Button(root, text="EXIT", command=self.on_close, bg="#E74C3C", fg="white", font=("Helvetica", 10, "bold")).pack(side="bottom", pady=10)

    def update_user_ui(self, text, action, room):
        self.lbl_user_text.config(text=f'"{text}"')
        self.lbl_action.config(text=f"Action: {action}")
        self.lbl_room.config(text=f"Room: {room}")

    def update_robot_ui(self, text):
        self.lbl_robot_text.config(text=text)

    def on_close(self):
        self.root.quit() # ใช้ quit() เพื่อหยุด mainloop อย่างละมุนละม่อม

def main():
    rclpy.init()
    root = tk.Tk()
    app = VoiceApp(root)
    ros_node = VoiceGuiNode(app)

    # --- วิธีแก้ไข: แยก ROS ไปรันใน Thread ต่างหาก ---
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # เมื่อปิด GUI ก็มาเคลียร์ ROS
        if rclpy.ok():
            ros_node.destroy_node()
            rclpy.shutdown()
        # thread ที่เป็น daemon=True จะถูกฆ่าเองเมื่อโปรแกรมจบ

if __name__ == '__main__':
    main()